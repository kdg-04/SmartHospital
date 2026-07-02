// ===================================================================
//  Digital Twin · Multi-Robot Fleet 지원 클래스
//  (기존 구조 유지 — Component가 사용하는 보조 클래스들)
//  SmartHospital.dc.html의 initTwin()이 <script> 태그로 이 파일을 주입해 로드합니다
//  (file:// 로 직접 열어도 동작하도록 ES 모듈 import 대신 일반 스크립트 + window 전역 등록 방식 사용).
// ===================================================================

// ---- Robot 상태 정의: Charging / Standby / Moving / Scanning / Returning / Emergency ----
const CG_ROBOT_STATE = {
  charging:   { ko: "충전 중",       color: "#2ad0c0" },
  standby:    { ko: "출동 대기",     color: "#ffd166" },
  moving:     { ko: "회진 이동",     color: "#4d9aff" },
  scanning:   { ko: "환자 스캔",     color: "#2ad0c0" },
  returning:  { ko: "충전 복귀",     color: "#7c8cff" },
  emergency:  { ko: "긴급 출동",     color: "#ff5d6c" },
  monitoring: { ko: "환자 모니터링", color: "#ff8a5c" },
  waiting:    { ko: "입구 대기",     color: "#ff8a5c" },
};

// ---- Grid Navigation 맵: Walkable / Obstacle / Wall / Corridor / Room / Door / Bed / Dock ----
class TwinGrid {
  static T = { OBSTACLE: 0, CORRIDOR: 1, ROOM: 2, DOOR: 3, DOCK: 4, BED: 5, WALL: 6 };
  constructor(cell, minX, minZ, maxX, maxZ) {
    this.CELL = cell; this.gMinX = minX; this.gMinZ = minZ;
    this.gCols = Math.ceil((maxX - minX) / cell);
    this.gRows = Math.ceil((maxZ - minZ) / cell);
    this.walk = new Uint8Array(this.gCols * this.gRows);   // Robot은 walkable만 이동
    this.type = new Uint8Array(this.gCols * this.gRows);   // 셀 속성
  }
  idx(c, r) { return r * this.gCols + c; }
  inb(c, r) { return c >= 0 && c < this.gCols && r >= 0 && r < this.gRows; }
  wk(c, r) { return this.inb(c, r) && this.walk[this.idx(c, r)] === 1; }
  cellOf(x, z) { return { c: Math.floor((x - this.gMinX) / this.CELL), r: Math.floor((z - this.gMinZ) / this.CELL) }; }
  pointOf(k) { return { x: this.gMinX + ((k % this.gCols) + 0.5) * this.CELL, z: this.gMinZ + (Math.floor(k / this.gCols) + 0.5) * this.CELL }; }
  // 페인터: 나중에 칠한 속성이 우선 (벽/침대 → 통로 순서로 덮어쓰기)
  paint(type, walkable, testFn) {
    for (let r = 0; r < this.gRows; r++) for (let c = 0; c < this.gCols; c++) {
      const x = this.gMinX + (c + 0.5) * this.CELL, z = this.gMinZ + (r + 0.5) * this.CELL;
      if (testFn(x, z)) { const k = this.idx(c, r); this.type[k] = type; this.walk[k] = walkable ? 1 : 0; }
    }
  }
  nearestWalk(cell) {
    if (this.wk(cell.c, cell.r)) return cell;
    for (let rad = 1; rad < 12; rad++) for (let dc = -rad; dc <= rad; dc++) for (let dr = -rad; dr <= rad; dr++)
      if (this.wk(cell.c + dc, cell.r + dr)) return { c: cell.c + dc, r: cell.r + dr };
    return cell;
  }
  // A* + line-of-sight smoothing — 침대·벽·시설을 절대 통과하지 않는다
  findPath(from, to) {
    const s = this.nearestWalk(this.cellOf(from.x, from.z)), e = this.nearestWalk(this.cellOf(to.x, to.z));
    const key = (c, r) => this.idx(c, r), sk = key(s.c, s.r), ek = key(e.c, e.r);
    const h = (c, r) => Math.hypot(c - e.c, r - e.r);
    const gsc = new Map([[sk, 0]]), came = new Map(), open = new Map([[sk, { c: s.c, r: s.r, f: h(s.c, s.r) }]]);
    const dirs = [[1, 0], [-1, 0], [0, 1], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]];
    let found = false, iter = 0;
    while (open.size && iter++ < 9000) {
      let bk = null, bf = Infinity;
      for (const [k, v] of open) if (v.f < bf) { bf = v.f; bk = k; }
      const cur = open.get(bk); open.delete(bk);
      if (bk === ek) { found = true; break; }
      for (const [dc, dr] of dirs) {
        const nc = cur.c + dc, nr = cur.r + dr;
        if (!this.wk(nc, nr)) continue;
        if (dc && dr && (!this.wk(cur.c + dc, cur.r) || !this.wk(cur.c, cur.r + dr))) continue;  // 코너 컷 금지
        const nk = key(nc, nr), ng = (gsc.get(bk) || 0) + Math.hypot(dc, dr);
        if (ng < (gsc.has(nk) ? gsc.get(nk) : Infinity)) { came.set(nk, bk); gsc.set(nk, ng); open.set(nk, { c: nc, r: nr, f: ng + h(nc, nr) }); }
      }
    }
    const raw = [];
    if (found) { let k = ek; while (k !== undefined) { raw.unshift(this.pointOf(k)); if (k === sk) break; k = came.get(k); } }
    const los = (a, b) => {
      const n = Math.ceil(Math.hypot(b.x - a.x, b.z - a.z) / (this.CELL * 0.5));
      for (let i = 1; i < n; i++) { const t = i / n, x = a.x + (b.x - a.x) * t, z = a.z + (b.z - a.z) * t; const c = this.cellOf(x, z); if (!this.wk(c.c, c.r)) return false; }
      return true;
    };
    const sm = [];
    if (raw.length) { sm.push(raw[0]); let i = 0; while (i < raw.length - 1) { let j = raw.length - 1; while (j > i + 1 && !los(raw[i], raw[j])) j--; sm.push(raw[j]); i = j; } }
    sm.push({ x: to.x, z: to.z });
    return sm.length > 1 ? sm : [{ x: to.x, z: to.z }];
  }
}

// ---- Robot 유닛: 이동 물리(Ease In/Out·회전) + 환자 스캔 시퀀스 ----
class RobotUnit {
  constructor(def, parts) {
    this.id = def.id; this.short = def.short; this.name = def.name; this.roleKo = def.roleKo;
    this.idx = def.idx; this.accent = def.accent; this.accentCss = def.css;
    this.zone = def.zone;
    this.team = def.team; this.isMain = !!def.main;   // 팀(0/1) + 메인/서브 — 배터리 부족 시 같은 팀 파트너와 교대
    Object.assign(this, parts);           // group / visor / light / scanRing / beamL / beamR / head / antenna / chargeRing / labelSprite
    this.state = "charging"; this.stateSince = 0;
    this.battery = def.battery;
    this.yaw = 0; this.v = 0;
    this.wps = null; this.wpi = 0; this.holdUntil = 0; this.pathOrigin = null;
    this.vmax = 2.1; this.accel = 1.65; this.decel = 2.25; this.turnRate = 3.6;   // 기본 이동 속도 1.5배
    this.task = null; this.patrolIdx = 0;
    this.scanJob = null; this.monitorUntil = 0; this.standbyUntil = 0;
    this.distTravelled = 0; this.bedsScanned = 0; this.roomsDone = 0;
    this._blockedSince = 0; this._col = null; this._dimOn = true;
  }
  setState(s, now) { if (this.state !== s) { this.state = s; this.stateSince = now || 0; } }
  stateKo() { return (CG_ROBOT_STATE[this.state] || {}).ko || this.state; }
  stateColor() { return (CG_ROBOT_STATE[this.state] || {}).color || "#8693a8"; }
  pos() { return this.group.position; }
  setPath(wps, vmax, now) {
    this.wps = wps && wps.length ? wps : null; this.wpi = 0;
    this.v = Math.min(this.v, 0.35); this.vmax = vmax || 2.1;
    this.holdUntil = (now || 0) + 620;                     // 출발 전 잠시 정지 (자연스러운 정차)
    const p = this.pos();
    this.pathOrigin = this.wps ? { x: p.x, z: p.z } : null;  // 출발점 고정 마커용 (이동 중 변하지 않음)
  }
  remaining() {
    if (!this.wps) return 0;
    let rem = 0, prev = this.pos();
    for (let i = this.wpi; i < this.wps.length; i++) { rem += Math.hypot(this.wps[i].x - prev.x, this.wps[i].z - prev.z); prev = this.wps[i]; }
    return rem;
  }
  turnTo(ty, dt) {
    let d = ty - this.yaw;
    while (d > Math.PI) d -= Math.PI * 2; while (d < -Math.PI) d += Math.PI * 2;
    const mx = this.turnRate * dt;
    this.yaw += Math.max(-mx, Math.min(mx, d));
    this.group.rotation.y = this.yaw;
    return Math.abs(d);
  }
  faceToward(x, z, dt) { const p = this.pos(); return this.turnTo(Math.atan2(x - p.x, z - p.z), dt); }
  // 이동: 가속 제한(Ease In) + 제동 거리(Ease Out) + 방향 전환 시 제자리 회전 → 순간이동/점프 없음
  move(dt, factor, now) {
    if (!this.wps || this.wpi >= this.wps.length) { this.v = 0; return { arrived: true, moving: false }; }
    if (now < this.holdUntil) { this.v = 0; return { arrived: false, moving: false }; }
    const p = this.pos();
    let wp = this.wps[this.wpi], dx = wp.x - p.x, dz = wp.z - p.z, d = Math.hypot(dx, dz);
    while (d < 0.34 && this.wpi < this.wps.length - 1) { this.wpi++; wp = this.wps[this.wpi]; dx = wp.x - p.x; dz = wp.z - p.z; d = Math.hypot(dx, dz); }
    if (this.wpi >= this.wps.length - 1 && d < 0.14) { this.v = 0; this.wps = null; return { arrived: true, moving: false }; }
    let rem = d;
    for (let i = this.wpi + 1; i < this.wps.length; i++) rem += Math.hypot(this.wps[i].x - this.wps[i - 1].x, this.wps[i].z - this.wps[i - 1].z);
    const derr = this.turnTo(Math.atan2(dx, dz), dt);
    const align = derr > 1.02 ? 0.05 : Math.max(0.12, Math.cos(derr));
    const vT = Math.min(this.vmax * factor, Math.sqrt(2 * this.decel * Math.max(rem - 0.06, 0))) * align;
    const dv = Math.max(-this.decel * dt * 1.7, Math.min(this.accel * dt, vT - this.v));
    this.v = Math.max(0, this.v + dv);
    const step = Math.min(this.v * dt, d);
    p.x += (dx / d) * step; p.z += (dz / d) * step;
    this.distTravelled += step;
    return { arrived: false, moving: this.v > 0.02 };
  }
  // ---- 병실 스캔 시퀀스: 진입 → 통로 정지 → 좌/우 환자 스캔 → 다음 위치 → 전 침대 확인 → 복도 복귀 ----
  startScan(room, now, fleet) {
    const mo = room.aisleMouth, fa = room.aisleFar;
    const avx = fa.x - mo.x, avz = fa.z - mo.z, L2 = avx * avx + avz * avz || 1;
    const rows = new Map();
    for (const b of room.beds) {
      if (!b.occupied) continue;
      const t = Math.max(0.1, Math.min(1, ((b.aimX - mo.x) * avx + (b.aimZ - mo.z) * avz) / L2));
      const k = Math.round(t * 12);
      if (!rows.has(k)) rows.set(k, { t, beds: [] });
      rows.get(k).beds.push(b);
    }
    const stops = Array.from(rows.values()).sort((a, b) => a.t - b.t)
      .map((s) => ({ x: mo.x + avx * s.t, z: mo.z + avz * s.t, beds: s.beds }));
    if (!stops.length) stops.push({ x: fa.x, z: fa.z, beds: [] });    // 빈 병실은 통과 점검만
    this.scanJob = {
      room, stops, si: 0, phase: "move", queue: [], bi: 0, tScan: 0,
      done: 0, total: Math.max(1, stops.reduce((s, x) => s + x.beds.length, 0)),
      exit: { x: mo.x, z: mo.z }, side: "L", curBed: null, letters: [], abort: false,
      stageIdx: 0, lastBed: null, completeFlashUntil: 0,   // AI Scan HUD: 단계 진행 · 완료 플래시
    };
    fleet.route(this, stops[0], 1.05, now);
  }
  tickScan(dt, now, fleet) {
    const j = this.scanJob; if (!j) return { done: true, scanned: 0, letters: [] };
    if (j.phase === "move") {
      if (this.move(dt, fleet.speedFactor(this, now), now).arrived) {
        j.queue = j.abort ? [] : j.stops[j.si].beds.slice();
        j.bi = 0; j.tScan = 0;
        j.phase = j.queue.length ? "scan" : "advance";
      }
    } else if (j.phase === "scan") {
      const b = j.queue[j.bi];
      this.faceToward(b.wx, b.wz, dt);                     // 침대를 향해 회전
      const offX = b.wx - this.pos().x, offZ = b.wz - this.pos().z;
      j.side = (offX * Math.cos(this.yaw) - offZ * Math.sin(this.yaw)) > 0 ? "R" : "L";
      j.curBed = b;
      j.tScan += dt;
      j.stageIdx = Math.min(5, Math.floor((j.tScan / 1.35) * 6));   // AI Detect→Face→Pose→PCI→Risk→Complete
      if (j.tScan >= 1.35 || j.abort) {
        j.done++; this.bedsScanned++; j.letters.push(b.letter);
        j.stageIdx = 5; j.lastBed = b; j.completeFlashUntil = now + 650;
        j.bi++; j.tScan = 0; j.curBed = null;
        if (j.bi >= j.queue.length || j.abort) j.phase = "advance";
      }
    } else if (j.phase === "advance") {
      j.si++;
      if (j.si < j.stops.length && !j.abort) { this.setPath([{ x: j.stops[j.si].x, z: j.stops[j.si].z }], 1.05, now); j.phase = "move"; }
      else { this.setPath([{ x: j.exit.x, z: j.exit.z }], 1.35, now); j.phase = "exit"; }
    } else if (j.phase === "exit") {
      if (this.move(dt, fleet.speedFactor(this, now), now).arrived) {
        const out = { done: true, scanned: j.done, letters: j.letters.slice(), roomId: j.room.id };
        this.scanJob = null;
        return out;
      }
    }
    return { done: false };
  }
}

// ---- Fleet Manager: 상태·배터리·충전 큐·회진 스케줄링·Emergency Dispatch·임무 큐·우선순위 ----
class FleetManager {
  constructor(o) {
    this.robots = o.robots; this.rooms = o.rooms; this.grid = o.grid; this.docks = o.docks;
    this.ownerRef = o.ownerRef;
    this.roomLocks = new Map();          // 병실 입구 대기 (Door Waiting)
    this.cooldown = {}; this.prevWorst = null;
    this.manualKey = null;
    this.missionQueue = [];              // 가용 로봇이 없을 때의 긴급 임무 대기열
    this.chargingQueue = [];             // Charging Dock 대기열 (레거시 — 팀별 전용 Dock 도입 후 사실상 미사용)
    this.CHARGE_RATE = 2.4; this.DRAIN_MOVE = 0.34; this.DRAIN_SCAN = 0.24; this.LOW = 20;
    // ---- 팀 교대(Main/Sub) 관리: 팀별로 항상 1대만 "근무 중(On-duty)" — 근무 로봇 배터리가 부족해지면 같은 팀 파트너가 교대 투입된다 ----
    this.onDuty = {};
    for (const r of this.robots) if (r.isMain) this.onDuty[r.team] = r.id;
  }
  isOnDuty(r) { return this.onDuty[r.team] === r.id; }
  teamMembers(team) { return this.robots.filter((r) => r.team === team); }
  // 같은 팀 파트너에게 회진 임무를 인계한다 (배터리 부족으로 근무 로봇이 충전소로 복귀할 때 호출)
  // 파트너가 대기/충전 중이었다면 그 즉시(다음 tick을 기다리지 않고) 회진에 투입해 병동 공백을 없앤다.
  handoff(team, now) {
    const cur = this.onDuty[team];
    const next = this.teamMembers(team).find((r) => r.id !== cur);
    if (!next || next.id === cur) return;
    this.onDuty[team] = next.id;
    this.log("Fleet", next.short + " 교대 투입 · " + (this.robotById(cur) || {}).short + " 회진 인계", "#ffd166");
    if (next.state === "standby") {
      this.assignPatrol(next, now);
    } else if ((next.state === "charging" || next.state === "returning") && next.battery >= 25) {
      // 완충 전이라도 즉시 투입 — 배터리 여유(25% 이상)가 있다면 공백 방지가 우선
      next.task = null;
      this.assignPatrol(next, now);
    }
  }
  owner() { try { return this.ownerRef(); } catch (e) { return null; } }
  log(type, msg, color) {
    const ow = this.owner();
    if (ow) { try { ow.logEvent(type, msg); if (color) ow.pushNotif(type, msg, color); } catch (e) {} }
  }
  robotById(id) { return this.robots.find((r) => r.id === id); }
  roomOf(id) { return this.rooms.find((r) => r.id === id); }
  // 병실 잠금
  tryLock(roomId, id) { const o = this.roomLocks.get(roomId); if (!o || o === id) { this.roomLocks.set(roomId, id); return true; } return false; }
  unlock(roomId, id) { if (this.roomLocks.get(roomId) === id) this.roomLocks.delete(roomId); }
  unlockAll(id) { for (const [k, v] of Array.from(this.roomLocks)) if (v === id) this.roomLocks.delete(k); }
  // 우선순위 (Emergency > Returning > Patrol > Standby) — Intersection Yield 기준
  priorityOf(r) {
    const p = { emergency: 5, monitoring: 5, returning: 3, moving: 2, scanning: 2, waiting: 1, standby: 0, charging: 0 }[r.state] || 0;
    return p * 10 + (3 - r.idx);
  }
  // ---- Collision Avoidance: 최소 거리 유지 + 우선순위 양보 + 교착 방지 ----
  speedFactor(r, now) {
    let f = 1;
    const p = r.pos(), my = this.priorityOf(r);
    const hx = Math.sin(r.yaw), hz = Math.cos(r.yaw);
    for (const o of this.robots) {
      if (o === r) continue;
      const q = o.pos(), dx = q.x - p.x, dz = q.z - p.z, d = Math.hypot(dx, dz);
      if (d > 4.6) continue;
      const ahead = (dx * hx + dz * hz) / (d || 1);
      const op = this.priorityOf(o);
      if (d < 1.8) f = Math.min(f, my <= op ? 0 : 0.2);                       // 최소 거리 — 서로 통과 금지
      else if (ahead > 0.3) f = Math.min(f, my <= op ? (d < 3 ? 0.12 : 0.45) : 0.6);  // 전방 양보
    }
    if (f === 0) {
      if (!r._blockedSince) r._blockedSince = now;
      else if (now - r._blockedSince > 3200) f = 0.14;                        // 교착 시 저속 회피
    } else r._blockedSince = 0;
    return f;
  }
  route(r, pt, vmax, now) { r.setPath(this.grid.findPath({ x: r.pos().x, z: r.pos().z }, pt), vmax, now); }
  waitPoint(room, r) {
    // 병실 입구 밖 대기 지점 (복도 쪽으로 비켜서 대기)
    const nx = room.center.x - room.door.x, nz = room.center.z - room.door.z;
    const L = Math.hypot(nx, nz) || 1, ux = nx / L, uz = nz / L;
    const side = r.idx % 2 === 0 ? 1 : -1;
    return { x: room.door.x - ux * 1.6 - uz * 2.0 * side, z: room.door.z - uz * 1.6 + ux * 2.0 * side };
  }
  missionLabel(r) {
    const t = r.task;
    if (r.state === "charging") return "충전 중 (" + Math.round(r.battery) + "%)";
    if (r.state === "standby") return "회진 대기";
    if (!t) return "임무 대기";
    if (t.type === "patrol") return t.roomId + "호 회진";
    if (t.type === "charge") return "충전소 복귀";
    if (t.type === "emergency") return t.roomId + "호 " + (t.manual ? "수동 파견" : "긴급 출동") + (r.state === "monitoring" ? " · 모니터링" : "");
    return "-";
  }
  scanPct(r) {
    const j = r.scanJob; if (!j) return 0;
    return Math.min(100, 100 * ((j.done + (j.phase === "scan" ? Math.min(1, j.tScan / 1.35) : 0)) / j.total));
  }
  // ---- Patrol Scheduling: 구역 분담 (중복 회진 없음) + Empty Room Skip (빈 병실은 건너뛴다) ----
  // 팀당 항상 1대만 근무(On-duty)한다 — 근무 로봇이 아니면 자기 Dock에서 충전/대기하며 교대를 기다린다.
  assignPatrol(r, now) {
    if (!this.isOnDuty(r)) {
      if (r.state !== "charging" && r.battery < 99) { this.assignCharge(r, now); }
      else { r.setState("standby", now); r.standbyUntil = now + 1500; }
      return;
    }
    let room = null;
    for (let tries = 0; tries < r.zone.length; tries++) {
      const cand = this.roomOf(r.zone[r.patrolIdx % r.zone.length]);
      r.patrolIdx++;
      if (cand && cand.occ > 0) { room = cand; break; }   // 환자가 없는 병실은 회진 대상에서 제외
    }
    if (!room) { r.setState("standby", now); r.standbyUntil = now + 2000; return; }  // 구역 전체가 공실이면 잠시 대기 후 재시도
    r.task = { type: "patrol", roomId: room.id, room };
    this.route(r, room.aisleMouth, 2.1, now);
    r.setState("moving", now);
  }
  assignCharge(r, now) {
    const dock = this.docks[r.idx];                        // 로봇마다 전용 Dock을 사용 (Dock 충돌 없음)
    r.task = { type: "charge" };
    this.route(r, { x: dock.x, z: dock.z }, 2.1, now);
    r.setState("returning", now);
    const qi = this.chargingQueue.indexOf(r.id); if (qi >= 0) this.chargingQueue.splice(qi, 1);
    this.log("Fleet", r.short + " 배터리 " + Math.round(r.battery) + "% · 충전소 자동 복귀", "#7c8cff");
  }
  chargingCount() { return this.robots.filter((r) => r.state === "charging" || r.state === "returning" && r.task && r.task.type === "charge").length; }
  // 충전 요청: 로봇마다 전용 Dock이 있으므로 대기열 없이 즉시 충전소로 복귀한다 (팀 파트너가 곧바로 교대 투입되어 병동 공백을 막는다)
  requestCharge(r, now) {
    this.assignCharge(r, now);
    return true;
  }
  processChargeQueue(now) {
    if (this.chargingQueue.length === 0 || this.chargingCount() > 0) return;
    const id = this.chargingQueue.shift();
    const r = this.robotById(id);
    if (!r || r.battery > this.LOW * 1.6) return;                        // 이미 회복했거나 사라짐 → 스킵
    if (r.state === "moving" || r.state === "standby" || r.state === "waiting") this.assignCharge(r, now);
    else this.chargingQueue.push(id);                                     // 스캔/긴급 중이면 다음 기회에
  }
  // ---- Emergency Dispatch: 가장 가까운 로봇 자동 출동 ----
  assignEmergency(r, roomId, bedId, manual, now) {
    const room = this.roomOf(roomId); if (!room) return;
    if (r.scanJob) { r.scanJob = null; }
    this.unlockAll(r.id);
    const wasCharging = r.state === "charging";
    if (wasCharging) { const qi = this.chargingQueue.indexOf(r.id); if (qi >= 0) this.chargingQueue.splice(qi, 1); }
    let bed = bedId ? room.beds.find((b) => b.id === bedId) || null : null;
    if (!bed) bed = room.beds.filter((b) => b.occupied).sort((a, b) => b.pci - a.pci)[0] || null;
    r.task = { type: "emergency", roomId, room, bed, manual: !!manual, stage: "go" };
    this.route(r, room.aisleMouth, 2.7, now);
    r.holdUntil = now + 120;                               // 긴급은 즉시 출발
    r.setState("emergency", now);
    this.cooldown[roomId] = now + 90000;
    if (wasCharging) this.log("Emergency Battery Policy", r.short + " 충전 중단 · 긴급 출동 → " + roomId + "호", "#ff5d6c");
    this.log("Emergency Dispatch", r.short + " → " + roomId + "호 " + (manual ? "수동 파견" : "긴급 자동 출동"), "#ff5d6c");
  }
  nearestAvailable(pt) {
    const c = this.robots.filter((r) => r.state !== "emergency" && r.state !== "monitoring" && r.battery > 25);
    if (!c.length) return null;
    return c.sort((a, b) =>
      Math.hypot(a.pos().x - pt.x, a.pos().z - pt.z) - Math.hypot(b.pos().x - pt.x, b.pos().z - pt.z))[0];
  }
  finishEmergency(r, now, silent) {
    if (!silent) this.log("Fleet", r.short + " 임무 완료 · 기존 회진 복귀", "#2ad0c0");
    this.unlockAll(r.id); r.task = null; r.scanJob = null;
    const next = this.missionQueue.shift();
    if (next) { this.assignEmergency(r, next.roomId, next.bedId, next.manual, now); return; }
    if (r.battery <= this.LOW) this.lowBatteryReturn(r, now);
    else this.assignPatrol(r, now);
  }
  // 배터리 부족으로 충전소 복귀 — 근무 중이던(On-duty) 로봇이면 같은 팀 파트너에게 즉시 인계한다
  lowBatteryReturn(r, now) {
    this.requestCharge(r, now);
    if (this.isOnDuty(r)) this.handoff(r.team, now);
  }
  // 수동 파견 (기존 robotMode/robotRoomId state와 연동)
  syncManual(st, now) {
    if (st.robotMode === "dispatch" && st.robotRoomId) {
      const key = st.robotRoomId + "|" + (st.robotBedId || "");
      if (this.manualKey !== key) {
        this.manualKey = key;
        const room = this.roomOf(st.robotRoomId);
        for (const o of this.robots) if (o.task && o.task.type === "emergency" && o.task.manual) this.finishEmergency(o, now, true);
        const r = this.nearestAvailable(room ? room.door : { x: 0, z: 0 }) || this.robots[0];
        if (r) this.assignEmergency(r, st.robotRoomId, st.robotBedId, true, now);
      }
    } else if (this.manualKey) {
      this.manualKey = null;
      for (const o of this.robots) if (o.task && o.task.type === "emergency" && o.task.manual) this.finishEmergency(o, now, true);
    }
  }
  // 위험 환자 발생(PCI 6 진입) → 자동 Emergency Dispatch
  autoDispatch(now) {
    if (!this.prevWorst) { this.prevWorst = {}; for (const rm of this.rooms) this.prevWorst[rm.id] = rm.worst; return; }
    for (const rm of this.rooms) {
      const prev = this.prevWorst[rm.id] || 0;
      this.prevWorst[rm.id] = rm.worst;
      if (rm.worst >= 6 && prev < 6 && rm.occ > 0 && now > (this.cooldown[rm.id] || 0)) {
        if (this.robots.some((r) => r.task && r.task.type === "emergency" && r.task.roomId === rm.id)) continue;
        const r = this.nearestAvailable(rm.door);
        if (r) this.assignEmergency(r, rm.id, null, false, now);
        else this.missionQueue.push({ roomId: rm.id, bedId: null, manual: false });
      }
    }
  }
  onArrive(r, now) {
    const t = r.task;
    if (!t) { r.setState("standby", now); r.standbyUntil = now + 900; return; }
    if (t.type === "charge") { r.setState("charging", now); return; }
    if (t.type === "patrol") {
      if (this.tryLock(t.roomId, r.id)) { r.setState("scanning", now); r.startScan(t.room, now, this); }
      else { this.route(r, this.waitPoint(t.room, r), 1.35, now); r.setState("waiting", now); }   // 병실 입구 대기
      return;
    }
    if (t.type === "emergency") {
      if (t.stage === "go") {
        if (this.tryLock(t.roomId, r.id)) {
          t.stage = "enter";
          this.route(r, t.bed ? { x: t.bed.aimX, z: t.bed.aimZ } : t.room.aisleFar, 1.425, now);
        } else {
          const holder = this.robotById(this.roomLocks.get(t.roomId));
          if (holder && holder.scanJob) holder.scanJob.abort = true;          // 스캔 로봇에게 양보 요청
          this.route(r, this.waitPoint(t.room, r), 1.35, now);
          r.setState("waiting", now);
        }
      } else {
        r.setState("monitoring", now);
        r.monitorUntil = now + 10000;
        this.log("Fleet", r.short + " " + t.roomId + "호 도착 · 환자 모니터링 시작", "#ff8a5c");
      }
      return;
    }
  }
  tickRobot(r, dt, now) {
    // Battery 관리
    if (r.state === "charging") r.battery = Math.min(100, r.battery + this.CHARGE_RATE * dt);
    else if (r.state === "standby" || r.state === "waiting") r.battery = Math.max(2, r.battery - 0.015 * dt);
    else r.battery = Math.max(2, r.battery - (r.state === "scanning" ? this.DRAIN_SCAN : this.DRAIN_MOVE) * dt);

    switch (r.state) {
      case "charging":
        if (r.battery >= 100) {                             // Battery 100% → Dock 이탈 → 자동 회진
          r.battery = 100;
          r.setState("standby", now); r.standbyUntil = now + 1400;
          this.log("Fleet", r.short + " 충전 완료 · 자동 회진 시작", "#2ad0c0");
        }
        break;
      case "standby":
        if (now >= r.standbyUntil) this.assignPatrol(r, now);
        break;
      case "moving": {
        // 회진 이동 중에도(스캔 완료를 기다리지 않고) 배터리가 부족해지면 즉시 충전소로 복귀 + 팀 교대 — 빈틈 최소화
        if (r.task && r.task.type === "patrol" && r.battery <= this.LOW) { this.lowBatteryReturn(r, now); break; }
        if (r.move(dt, this.speedFactor(r, now), now).arrived) this.onArrive(r, now);
        break;
      }
      case "returning": case "emergency": {
        if (r.move(dt, this.speedFactor(r, now), now).arrived) this.onArrive(r, now);
        break;
      }
      case "waiting": {
        if (r.wps) { r.move(dt, this.speedFactor(r, now), now); break; }      // 대기 지점으로 이동 중
        const t = r.task;
        if (!t) { this.assignPatrol(r, now); break; }
        if (this.tryLock(t.roomId, r.id)) {
          if (t.type === "emergency") {
            t.stage = "enter";
            this.route(r, t.bed ? { x: t.bed.aimX, z: t.bed.aimZ } : t.room.aisleFar, 1.425, now);
            r.setState("emergency", now);
          } else {
            r.setState("scanning", now);
            r.startScan(t.room, now, this);
          }
        }
        break;
      }
      case "scanning": {
        const res = r.tickScan(dt, now, this);
        if (res.done) {
          this.unlock(r.task && r.task.roomId, r.id);
          r.roomsDone++;
          const ow = this.owner();
          if (ow && ow.recordRobotVisit && res.letters && res.letters.length) ow.recordRobotVisit(res.roomId, res.letters, r.short);
          r.task = null;
          // 20% 이하 → 충전소 복귀 + (근무 중이었다면) 같은 팀 파트너에게 회진 인계
          if (r.battery <= this.LOW) this.lowBatteryReturn(r, now);
          else this.assignPatrol(r, now);
        }
        break;
      }
      case "monitoring": {
        const t = r.task;
        if (t && t.bed) r.faceToward(t.bed.wx, t.bed.wz, dt);
        if (t && !t.manual && now > r.monitorUntil) this.finishEmergency(r, now);
        else if (r.battery <= 12) this.finishEmergency(r, now);
        break;
      }
    }
  }
  // 상태별 시각 효과: LED · Scan Ring · Beam · Charging Effect
  animate(r, now, THREE) {
    if (!r._col) r._col = new THREE.Color(r.accent);
    const s = r.state;
    const dimF = r._dimOn === false ? 0.12 : 1;
    const target = (s === "emergency" || s === "monitoring") ? 0xff5d6c
      : (s === "scanning" || s === "charging") ? 0x2ad0c0
      : s === "returning" ? 0x7c8cff : r.accent;
    r._col.lerp(new THREE.Color(target), 0.08);
    r.visor.material.emissive.copy(r._col);
    r.light.color.copy(r._col);
    const fast = s === "emergency" || s === "scanning";
    r.visor.material.emissiveIntensity = (1.15 + 0.55 * Math.sin(now * (fast ? 0.012 : 0.004) + r.idx * 2)) * dimF;
    r.antenna.material.emissiveIntensity = (1.2 + 0.8 * Math.sin(now * (fast ? 0.014 : 0.005) + r.idx)) * dimF;
    // Charging Animation: 떠오르는 에너지 링 + Dock LED
    const dock = this.docks[r.idx];
    if (s === "charging") {
      const t = ((now * 0.0009) + r.idx * 0.33) % 1;
      r.chargeRing.visible = true;
      r.chargeRing.position.y = 0.25 + t * 1.7;
      r.chargeRing.material.opacity = 0.55 * (1 - t) * dimF;
      r.chargeRing.scale.setScalar(1.05 - 0.35 * t);
      dock.ring.material.emissiveIntensity = 1.0 + 0.8 * Math.sin(now * 0.006);
      dock.setStatus("충전 " + Math.round(r.battery) + "%", "#2ad0c0");
    } else {
      r.chargeRing.visible = false;
      const home = Math.hypot(r.pos().x - dock.x, r.pos().z - dock.z) < 2.2;
      dock.ring.material.emissiveIntensity = home ? 0.9 : 0.3;
      dock.setStatus(home ? "대기" : r.short + " 회진 중", home ? "#ffd166" : "#5a6678");
    }
    // Scan 효과: Scanner Beam(좌/우) + Scan Ring + Pulse + 침대 하이라이트
    const j = r.scanJob;
    const scanBeam = s === "scanning" && j && j.phase === "scan";
    const inAisle = s === "scanning" && j && (j.phase === "move" || j.phase === "scan");
    r.scanRing.visible = (inAisle || s === "monitoring") && dimF === 1;
    if (r.scanRing.visible) {
      const w = (Math.sin(now * 0.008) + 1) / 2;
      r.scanRing.scale.setScalar(1 + 0.5 * w);
      r.scanRing.material.opacity = 0.55 - 0.35 * w;
      r.scanRing.material.color.copy(r._col);
    }
    let hy = 0;
    r.beamL.visible = scanBeam && j.side === "L" && dimF === 1;
    r.beamR.visible = scanBeam && j.side === "R" && dimF === 1;
    if (scanBeam) {
      const beam = j.side === "L" ? r.beamL : r.beamR;
      beam.material.opacity = 0.1 + 0.2 * ((Math.sin(now * 0.01) + 1) / 2);
      beam.scale.set(1, 0.75 + 0.5 * ((Math.sin(now * 0.006) + 1) / 2), 1);
      hy = j.side === "L" ? 0.85 : -0.85;                  // 스캔 방향으로 헤드 회전
      // Patient Highlight: 스캔 중인 환자 침대 — Blue Pulse Glow + Bounding-box 느낌의 스케일 확대
      if (j.curBed) {
        const pulse = 0.5 + 0.5 * Math.sin(now * 0.012);
        j.curBed.dotMat.emissiveIntensity += 1.6 + 1.0 * pulse;
        j.curBed.dot.scale.setScalar(1.18 + 0.3 * pulse);
      }
    }
    if (s === "monitoring" && r.task && r.task.bed) r.task.bed.dotMat.emissiveIntensity += 0.9;
    r.head.rotation.y += (hy - r.head.rotation.y) * 0.1;
  }
  statusLine() {
    const em = this.robots.find((r) => r.state === "emergency" || r.state === "monitoring");
    if (em) {
      const t = em.task || {};
      return { text: (em.state === "emergency" ? "🚑 " : "🩺 ") + em.short + " · " + (t.roomId || "?") + "호 " + (em.state === "emergency" ? "긴급 출동 중" : "환자 모니터링 중"), color: "#ff5d6c" };
    }
    const sc = this.robots.find((r) => r.state === "scanning" && r.scanJob);
    if (sc) {
      const j = sc.scanJob;
      return { text: "🤖 " + sc.short + " · " + j.room.id + "호 " + (j.phase === "scan" ? ((j.side === "L" ? "좌측" : "우측") + " 환자 스캔") : "통로 스캔") + " 중 (" + Math.round(this.scanPct(sc)) + "%)", color: "#2ad0c0" };
    }
    const mv = this.robots.find((r) => r.state === "moving" && r.task);
    if (mv) return { text: "🤖 " + mv.short + " · " + mv.task.roomId + "호로 회진 이동 중", color: "#4d9aff" };
    const ch = this.robots.filter((r) => r.state === "charging").length;
    return { text: "🤖 Fleet 운영 중 · 충전 " + ch + "대", color: "#8693a8" };
  }
  tick(dt, now, ctx) {
    this.syncManual(ctx.state || {}, now);
    this.autoDispatch(now);
    this.processChargeQueue(now);
    for (const r of this.robots) { this.tickRobot(r, dt, now); this.animate(r, now, ctx.THREE); }
    if (ctx.statusEl) {
      const s = this.statusLine();
      ctx.statusEl.textContent = s.text;
      ctx.statusEl.style.color = s.color;
    }
  }
}

// ---- window 전역 등록 ----
// ES 모듈(export)이 아니라 일반 <script>로 로드되므로(파일을 file://로 직접 열어도 동작),
// 클래스들을 window에 노출해 SmartHospital.dc.html의 initTwin()이 읽어가게 한다.
window.CG_ROBOT_STATE = CG_ROBOT_STATE;
window.TwinGrid = TwinGrid;
window.RobotUnit = RobotUnit;
window.FleetManager = FleetManager;
