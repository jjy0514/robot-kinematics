// 로봇 기구학 계산 모듈 (DOM/Three.js 의존성 없음 → Node에서도 테스트 가능)
// 단위: 길이 mm, 각도는 UI/테이블에서 deg, 내부 계산은 rad
// 4x4 동차변환행렬은 row-major 중첩 배열 [[r00,r01,r02,px], ...] 로 표현

const { cos, sin, atan2, sqrt, abs, acos, PI } = Math;
export const DEG = PI / 180;
export const RAD = 180 / PI;

// ---------------------------------------------------------------- 행렬 유틸
export function identity() {
  return [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]];
}

export function mul(A, B) {
  const C = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]];
  for (let i = 0; i < 4; i++)
    for (let j = 0; j < 4; j++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += A[i][k] * B[k][j];
      C[i][j] = s;
    }
  return C;
}

// 강체변환 역행렬: [Rᵀ, −Rᵀp]
export function invT(T) {
  const Rt = transpose3(rot(T));
  const p = pos(T);
  return fromRotPos(Rt, Rt.map((r) => -(r[0] * p[0] + r[1] * p[1] + r[2] * p[2])));
}

export function transformPoint(T, p) {
  return [0, 1, 2].map((i) => T[i][0] * p[0] + T[i][1] * p[1] + T[i][2] * p[2] + T[i][3]);
}

export function rot(T) {
  return [T[0].slice(0, 3), T[1].slice(0, 3), T[2].slice(0, 3)];
}

export function pos(T) {
  return [T[0][3], T[1][3], T[2][3]];
}

export function fromRotPos(R, p) {
  return [
    [R[0][0], R[0][1], R[0][2], p[0]],
    [R[1][0], R[1][1], R[1][2], p[1]],
    [R[2][0], R[2][1], R[2][2], p[2]],
    [0, 0, 0, 1],
  ];
}

export function mul3(A, B) {
  const C = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
  return C;
}

export function transpose3(R) {
  return [[R[0][0], R[1][0], R[2][0]], [R[0][1], R[1][1], R[2][1]], [R[0][2], R[1][2], R[2][2]]];
}

const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (v) => sqrt(v.reduce((s, x) => s + x * x, 0));

export const Rx = (t) => [[1, 0, 0], [0, cos(t), -sin(t)], [0, sin(t), cos(t)]];
export const Ry = (t) => [[cos(t), 0, sin(t)], [0, 1, 0], [-sin(t), 0, cos(t)]];
export const Rz = (t) => [[cos(t), -sin(t), 0], [sin(t), cos(t), 0], [0, 0, 1]];

// ---------------------------------------------------------------- DH 변환
// Standard DH (Denavit–Hartenberg, Spong/Corke):
//   A_i = Rz(θ_i) · Tz(d_i) · Tx(a_i) · Rx(α_i)
export function dhStandard(a, alpha, d, theta) {
  const ct = cos(theta), st = sin(theta), ca = cos(alpha), sa = sin(alpha);
  return [
    [ct, -st * ca, st * sa, a * ct],
    [st, ct * ca, -ct * sa, a * st],
    [0, sa, ca, d],
    [0, 0, 0, 1],
  ];
}

// Modified DH (Craig):
//   A_i = Rx(α_{i-1}) · Tx(a_{i-1}) · Rz(θ_i) · Tz(d_i)
export function dhModified(a, alpha, d, theta) {
  const ct = cos(theta), st = sin(theta), ca = cos(alpha), sa = sin(alpha);
  return [
    [ct, -st, 0, a],
    [st * ca, ct * ca, -sa, -sa * d],
    [st * sa, ct * sa, ca, ca * d],
    [0, 0, 0, 1],
  ];
}

// ---------------------------------------------------------------- 오일러 각
// 'ZYX' : R = Rz(rz)·Ry(ry)·Rx(rx)   (Roll-Pitch-Yaw, 고정축 X→Y→Z)
// 'XYZ' : R = Rx(rx)·Ry(ry)·Rz(rz)   (Stäubli 방식, 이동축 X→Y'→Z'')
// 'ZYZ' : R = Rz(a)·Ry(b)·Rz(c)      (고전 오일러각, 값은 [a, b, c] 순서)
// 모든 함수는 [e1, e2, e3] (rad) 를 주고받는다.
//   ZYX/XYZ → [rx, ry, rz],  ZYZ → [a, b, c]
export const EULER_TYPES = {
  ZYX: { label: 'RPY  (Rz·Ry·Rx)', names: ['Rx', 'Ry', 'Rz'] },
  XYZ: { label: 'Stäubli XYZ  (Rx·Ry·Rz)', names: ['Rx', 'Ry', 'Rz'] },
  ZYZ: { label: 'ZYZ 오일러  (Rz·Ry·Rz)', names: ['φ (Z)', 'θ (Y)', 'ψ (Z)'] },
};

export function eulerToRot(type, e) {
  const [e1, e2, e3] = e;
  switch (type) {
    case 'ZYX': return mul3(mul3(Rz(e3), Ry(e2)), Rx(e1));
    case 'XYZ': return mul3(mul3(Rx(e1), Ry(e2)), Rz(e3));
    case 'ZYZ': return mul3(mul3(Rz(e1), Ry(e2)), Rz(e3));
    default: throw new Error('unknown euler type ' + type);
  }
}

const EPS_GIMBAL = 1e-9;

export function rotToEuler(type, R) {
  const r = R;
  switch (type) {
    case 'ZYX': {
      const cb = sqrt(r[0][0] ** 2 + r[1][0] ** 2);
      const b = atan2(-r[2][0], cb);
      if (cb < EPS_GIMBAL) {
        // 짐벌락: rz = 0 으로 두고 rx 만 결정
        const a = b > 0 ? atan2(r[0][1], r[1][1]) : atan2(-r[0][1], r[1][1]);
        return [a, b, 0];
      }
      return [atan2(r[2][1], r[2][2]), b, atan2(r[1][0], r[0][0])];
    }
    case 'XYZ': {
      const cb = sqrt(r[1][2] ** 2 + r[2][2] ** 2);
      const b = atan2(r[0][2], cb);
      if (cb < EPS_GIMBAL) {
        const a = b > 0 ? atan2(r[1][0], r[1][1]) : atan2(-r[1][0], r[1][1]);
        return [a, b, 0];
      }
      return [atan2(-r[1][2], r[2][2]), b, atan2(-r[0][1], r[0][0])];
    }
    case 'ZYZ': {
      const sb = sqrt(r[0][2] ** 2 + r[1][2] ** 2);
      const b = atan2(sb, r[2][2]);
      if (sb < EPS_GIMBAL) {
        if (r[2][2] > 0) return [atan2(r[1][0], r[0][0]), 0, 0];
        return [atan2(-r[1][0], -r[0][0]), PI, 0];
      }
      return [atan2(r[1][2], r[0][2]), b, atan2(r[2][1], -r[2][0])];
    }
    default: throw new Error('unknown euler type ' + type);
  }
}

// 회전행렬 → 회전벡터 (axis * angle), so(3) log map
export function rotLog(R) {
  const tr = R[0][0] + R[1][1] + R[2][2];
  const c = Math.min(1, Math.max(-1, (tr - 1) / 2));
  const angle = acos(c);
  if (angle < 1e-9) return [0, 0, 0];
  if (PI - angle < 1e-5) {
    // 180° 근처: 대각 성분으로 축 계산
    const xx = (R[0][0] + 1) / 2, yy = (R[1][1] + 1) / 2, zz = (R[2][2] + 1) / 2;
    let ax;
    if (xx >= yy && xx >= zz) {
      const x = sqrt(Math.max(xx, 0));
      ax = [x, (R[0][1] + R[1][0]) / (4 * x), (R[0][2] + R[2][0]) / (4 * x)];
    } else if (yy >= zz) {
      const y = sqrt(Math.max(yy, 0));
      ax = [(R[0][1] + R[1][0]) / (4 * y), y, (R[1][2] + R[2][1]) / (4 * y)];
    } else {
      const z = sqrt(Math.max(zz, 0));
      ax = [(R[0][2] + R[2][0]) / (4 * z), (R[1][2] + R[2][1]) / (4 * z), z];
    }
    const n = norm(ax);
    return ax.map((v) => (v / n) * angle);
  }
  const k = angle / (2 * sin(angle));
  return [(R[2][1] - R[1][2]) * k, (R[0][2] - R[2][0]) * k, (R[1][0] - R[0][1]) * k];
}

// ---------------------------------------------------------------- 툴 변환
// tool = { x, y, z (mm), rx, ry, rz (deg, RPY: Rz·Ry·Rx) }
export function toolMatrix(tool) {
  if (!tool) return identity();
  const R = eulerToRot('ZYX', [tool.rx * DEG, tool.ry * DEG, tool.rz * DEG]);
  return fromRotPos(R, [tool.x, tool.y, tool.z]);
}

export function isIdentityTool(tool) {
  return !tool || ['x', 'y', 'z', 'rx', 'ry', 'rz'].every((k) => abs(tool[k] || 0) < 1e-12);
}

// ---------------------------------------------------------------- 정기구학
// robot = { convention: 'standard'|'modified', rows: [{a, alpha, d, offset, min, max}], tool }
// q: 관절각 배열 (deg)
// 반환: A[i] (i=0..n-1, 즉 A_1..A_n), T[k] (k=0..n, T_0^k), Ttool, Tend (= T_0^n · Ttool)
export function forward(robot, q) {
  const dhFn = robot.convention === 'modified' ? dhModified : dhStandard;
  const A = [];
  const T = [identity()];
  robot.rows.forEach((r, i) => {
    const Ai = dhFn(r.a, r.alpha * DEG, r.d, (q[i] + r.offset) * DEG);
    A.push(Ai);
    T.push(mul(T[i], Ai));
  });
  const Ttool = toolMatrix(robot.tool);
  const Tend = mul(T[T.length - 1], Ttool);
  return { A, T, Ttool, Tend };
}

// 관절 i (0-based) 의 회전축이 놓인 프레임
//   standard: 관절 i+1 은 z_{i} 축 (프레임 {i})
//   modified: 관절 i+1 은 z_{i+1} 축 (프레임 {i+1})
export function jointFrame(robot, fkResult, i) {
  return robot.convention === 'modified' ? fkResult.T[i + 1] : fkResult.T[i];
}

// 기하학적 자코비안 (6xn). 선속도 행은 mm/rad, 각속도 행은 rad/rad
export function jacobian(robot, fkResult) {
  const n = robot.rows.length;
  const pe = pos(fkResult.Tend);
  const J = [[], [], [], [], [], []];
  for (let i = 0; i < n; i++) {
    const F = jointFrame(robot, fkResult, i);
    const z = [F[0][2], F[1][2], F[2][2]];
    const o = pos(F);
    const jv = cross(z, [pe[0] - o[0], pe[1] - o[1], pe[2] - o[2]]);
    for (let r = 0; r < 3; r++) {
      J[r][i] = jv[r];
      J[r + 3][i] = z[r];
    }
  }
  return J;
}

// ---------------------------------------------------------------- 역기구학 (수치해)
// 연립방정식 풀이 (가우스 소거, 부분 피벗)
function solveLinear(M, b) {
  const n = b.length;
  const A = M.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (abs(A[r][c]) > abs(A[p][c])) p = r;
    [A[c], A[p]] = [A[p], A[c]];
    const piv = A[c][c];
    if (abs(piv) < 1e-14) continue;
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = A[r][c] / piv;
      for (let k = c; k <= n; k++) A[r][k] -= f * A[c][k];
    }
  }
  return A.map((row, i) => (abs(row[i]) < 1e-14 ? 0 : row[n] / row[i]));
}

// 목표 대비 자세 오차: [위치오차(mm) 3, 회전오차(rad) 3]
export function poseError(Ttarget, Tcur) {
  const pt = pos(Ttarget), pc = pos(Tcur);
  const ep = [pt[0] - pc[0], pt[1] - pc[1], pt[2] - pc[2]];
  const eo = rotLog(mul3(rot(Ttarget), transpose3(rot(Tcur))));
  return { ep, eo, posErr: norm(ep), oriErr: norm(eo) };
}

const wrapRad = (x) => atan2(sin(x), cos(x));

function wrapDeg(x) {
  return ((((x + 180) % 360) + 360) % 360) - 180;
}

// 범위 [min,max] 안에서 x 와 360° 차이 나는 등가각 중 가장 가까운 값
function fitToLimits(x, min, max) {
  if (x >= min && x <= max) return x;
  for (const k of [-360, 360, -720, 720]) {
    const y = x + k;
    if (y >= min && y <= max) return y;
  }
  return Math.min(max, Math.max(min, x));
}

// Damped Least Squares (Levenberg–Marquardt) 역기구학
//   Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
// opts: { maxIter, tolPos(mm), tolOri(rad), lambda, restarts, respectLimits, rng, trace }
//   trace: 배열을 넘기면 반복마다 { attempt, it, q, posErr, oriErr, [J, e, dq, scale] } 를 기록
export function inverse(robot, Ttarget, q0, opts = {}) {
  const {
    maxIter = 300,
    tolPos = 0.01,
    tolOri = 1e-4,
    lambda = 0.05,
    restarts = 10,
    respectLimits = true,
    rng = Math.random,
    trace = null,
  } = opts;
  const n = robot.rows.length;
  const L = 1000; // 위치 오차 스케일 (mm → m) : 위치/회전 오차 단위를 비슷하게 맞춤
  let best = null;
  let totalIters = 0;

  for (let attempt = 0; attempt <= restarts; attempt++) {
    let q = attempt === 0
      ? q0.slice()
      : robot.rows.map((r) => r.min + rng() * (r.max - r.min));
    let res = null;
    for (let it = 0; it < maxIter; it++) {
      totalIters++;
      const fk = forward(robot, q);
      const err = poseError(Ttarget, fk.Tend);
      res = { q: q.slice(), posErr: err.posErr, oriErr: err.oriErr };
      const rec = trace ? { attempt, it, q: q.slice(), posErr: err.posErr, oriErr: err.oriErr } : null;
      if (rec) trace.push(rec);
      if (err.posErr < tolPos && err.oriErr < tolOri) {
        res.success = true;
        break;
      }
      const J = jacobian(robot, fk);
      for (let c = 0; c < n; c++) for (let r = 0; r < 3; r++) J[r][c] /= L;
      const e = [...err.ep.map((v) => v / L), ...err.eo];
      // (J Jᵀ + λ² I) y = e,  Δq = Jᵀ y
      const JJt = [];
      for (let r = 0; r < 6; r++) {
        JJt.push([]);
        for (let c = 0; c < 6; c++) {
          let s = 0;
          for (let k = 0; k < n; k++) s += J[r][k] * J[c][k];
          JJt[r].push(s + (r === c ? lambda * lambda : 0));
        }
      }
      const y = solveLinear(JJt, e);
      const dq = [];
      for (let k = 0; k < n; k++) {
        let s = 0;
        for (let r = 0; r < 6; r++) s += J[r][k] * y[r];
        dq.push(s);
      }
      const maxStep = 0.3; // rad
      const m = Math.max(...dq.map(abs));
      const scale = m > maxStep ? maxStep / m : 1;
      if (rec) Object.assign(rec, { J: J.map((r) => r.slice()), e, dq, scale, lambda, L });
      q = q.map((v, k) => v + dq[k] * scale * RAD);
      if (respectLimits) q = q.map((v, k) => fitToLimits(v, robot.rows[k].min, robot.rows[k].max));
      else q = q.map(wrapDeg);
    }
    if (!res.success) {
      // 루프 종료 직후 상태 재평가
      const err = poseError(Ttarget, forward(robot, q).Tend);
      res = { q: q.slice(), posErr: err.posErr, oriErr: err.oriErr, success: err.posErr < tolPos && err.oriErr < tolOri };
    }
    if (!best || res.success || res.posErr + res.oriErr * L < best.posErr + best.oriErr * L) best = res;
    if (res.success) break;
  }
  best.iterations = totalIters;
  return best;
}

// ---------------------------------------------------------------- 역기구학 (해석해)
// 구면 손목을 가진 6R 로봇의 기하학적 분리 해법 (Pieper):
//   ① 손목 중심 p_w = p − d₆ z₆ 는 q₁~q₃ 에만 의존 → 위치 문제
//   ② R₃⁶ = (R₀³)ᵀ R₀⁶ 에서 q₄~q₆ 를 ZYZ 오일러각처럼 추출 → 자세 문제
// Standard DH 기준 적용 조건:
//   α₁=±90°, α₂=0, α₃=±90°, α₄=±90°, α₅=−α₄, α₆=0, a₄=a₅=a₆=0, d₅=0
//   (a₁, a₂, a₃, d₁~d₄, d₆ 는 임의 → TX200, PUMA560 모두 해당)
// 해는 최대 8개 = 어깨(2) × 팔꿈치(2) × 손목(2)
export function analyticIK(robotIn, Ttarget) {
  let robot = robotIn;
  let convWarning = null;
  if (robotIn.convention !== 'standard') {
    const c = convertConvention(robotIn, 'standard');
    robot = c.robot;
    convWarning = c.warning;
  }
  const r = robot.rows;
  const near = (x, v) => abs(x - v) < 1e-6;
  const is90 = (x) => near(abs(x), 90);
  const n6 = r.length === 6;
  const checks = [
    { label: '관절 6개', ok: n6 },
    { label: 'α₁ = ±90°', ok: n6 && is90(r[0].alpha) },
    { label: 'α₂ = 0°  (관절 2·3 축 평행 → 평면 2링크)', ok: n6 && near(r[1].alpha, 0) },
    { label: 'α₃ = ±90°', ok: n6 && is90(r[2].alpha) },
    { label: 'α₄ = ±90°, α₅ = −α₄', ok: n6 && is90(r[3].alpha) && near(r[4].alpha, -r[3].alpha) },
    { label: 'α₆ = 0, a₆ = 0', ok: n6 && near(r[5].alpha, 0) && near(r[5].a, 0) },
    { label: 'a₄ = a₅ = 0, d₅ = 0  (구면 손목: 관절 4·5·6 축이 한 점에서 만남)', ok: n6 && near(r[3].a, 0) && near(r[4].a, 0) && near(r[4].d, 0) },
  ];
  const applicable = checks.every((c) => c.ok);
  const out = { applicable, checks, robot, convWarning, solutions: [] };
  if (!applicable) return out;

  // ① 툴 제거 → T₀⁶
  const Ttool = toolMatrix(robot.tool);
  const T06 = mul(Ttarget, invT(Ttool));
  const R06 = rot(T06);
  const p = pos(T06);
  const z6 = [R06[0][2], R06[1][2], R06[2][2]];
  const d6 = r[5].d;
  const pw = [p[0] - d6 * z6[0], p[1] - d6 * z6[1], p[2] - d6 * z6[2]];

  // ② θ₁ : 손목중심의 수평 투영. 관절 2·3 이 움직이는 평면은 z₁ 방향으로 (d₂+d₃) 만큼 떨어져 있다
  const sa1 = Math.sign(r[0].alpha);
  const D = r[1].d + r[2].d;
  const Lat = -D * sa1;
  const rho2 = pw[0] ** 2 + pw[1] ** 2;
  const sa3 = Math.sign(r[2].alpha);
  const a2 = r[1].a, a3 = r[2].a, d4 = r[3].d;
  const L3 = Math.hypot(a3, d4);
  const phi = atan2(-d4 * sa3, a3);
  const s = r[3].alpha < 0 ? 1 : -1; // R₃⁶ = Rz(θ₄)·Ry(s·θ₅)·Rz(θ₆)
  Object.assign(out, { Ttool, T06, R06, p, z6, d6, pw, sa1, D, Lat, rho2, sa3, a2, a3, d4, L3, phi, s });

  if (rho2 < Lat * Lat - 1e-9) {
    out.unreachable = '손목 중심이 어깨 옆 오프셋(d₂+d₃) 보다 z₀ 축에 가까워 θ₁ 해가 없습니다.';
    return out;
  }
  const rAbs = sqrt(Math.max(rho2 - Lat * Lat, 0));
  out.shoulders = [];

  for (const sigma of [1, -1]) {
    const rr = sigma * rAbs;
    const th1 = wrapRad(atan2(pw[1], pw[0]) - atan2(Lat, rr));
    const T01 = dhStandard(r[0].a, r[0].alpha * DEG, r[0].d, th1);
    const p1 = transformPoint(invT(T01), pw);
    const [X, Y, Z] = p1;
    const cpsi = (X * X + Y * Y - a2 * a2 - L3 * L3) / (2 * a2 * L3);
    const sh = { sigma, r: rr, th1, T01, p1, cpsi, reachable: abs(cpsi) <= 1 + 1e-9 };
    out.shoulders.push(sh);
    if (!sh.reachable) continue;

    for (const eps of [1, -1]) {
      const psi = eps * acos(Math.min(1, Math.max(-1, cpsi)));
      const th2 = wrapRad(atan2(Y, X) - atan2(L3 * sin(psi), a2 + L3 * cos(psi)));
      const th3 = wrapRad(psi - phi);
      const T02 = mul(T01, dhStandard(a2, r[1].alpha * DEG, r[1].d, th2));
      const T03 = mul(T02, dhStandard(a3, r[2].alpha * DEG, r[2].d, th3));
      const R03 = rot(T03);
      const R36 = mul3(transpose3(R03), R06);
      const sb = sqrt(R36[0][2] ** 2 + R36[1][2] ** 2);
      const singular = sb < 1e-9;

      for (const w of [1, -1]) {
        let th4, b, th6;
        if (singular) {
          // 손목 특이점: θ₄ + θ₆ (또는 θ₄ − θ₆) 만 결정됨 → θ₄ = 0 으로 고정
          th4 = 0;
          if (R36[2][2] > 0) { b = 0; th6 = atan2(R36[1][0], R36[0][0]); }
          else { b = PI; th6 = -atan2(-R36[1][0], -R36[0][0]); }
          if (w === -1) continue; // 특이점에서는 손목 해가 무한히 많으므로 하나만
        } else {
          b = atan2(w * sb, R36[2][2]);
          th4 = atan2(w * R36[1][2], w * R36[0][2]);
          th6 = atan2(w * R36[2][1], -w * R36[2][0]);
        }
        const th5 = s * b;
        const theta = [th1, th2, th3, th4, th5, th6];
        const q = theta.map((t, i) => {
          // ±360° 등가각 중 한계 안에 드는 값이 있으면 그것을, 없으면 원래 값 유지 (한계 초과로 표시)
          const v = wrapDeg(t * RAD - r[i].offset);
          const f = fitToLimits(v, r[i].min, r[i].max);
          return abs(((f - v) % 360 + 360) % 360) < 1e-9 || abs(((f - v) % 360 + 360) % 360 - 360) < 1e-9 ? f : v;
        });
        const inLimits = q.every((v, i) => v >= r[i].min - 1e-9 && v <= r[i].max + 1e-9);
        const err = poseError(Ttarget, forward(robotIn, q).Tend);
        out.solutions.push({
          shoulder: sigma, elbow: eps, wrist: w, singular,
          theta, q, inLimits, posErr: err.posErr, oriErr: err.oriErr,
          detail: { sh, psi, th2, th3, T02, T03, R03, R36, sb, b },
        });
      }
    }
  }
  return out;
}

// ---------------------------------------------------------------- 규약 변환
// Standard → Modified :  A_i(std) 의 Tx(a_i)Rx(α_i) 를 다음 행으로 넘긴다.
//   마지막 행의 (a_n, α_n) 은 툴 변환 앞에 붙인다. → 말단 자세가 정확히 동일
// Modified → Standard :  반대 방향. 첫 행의 (a_0, α_0) 는 표현할 곳이 없으므로 버린다(경고).
export function convertConvention(robot, target) {
  if (robot.convention === target) return { robot, warning: null };
  const rows = robot.rows.map((r) => ({ ...r }));
  let tool = { ...robot.tool };
  let warning = null;
  const n = rows.length;
  if (target === 'modified') {
    const last = { a: rows[n - 1].a, alpha: rows[n - 1].alpha };
    for (let i = n - 1; i > 0; i--) {
      rows[i].a = rows[i - 1].a;
      rows[i].alpha = rows[i - 1].alpha;
    }
    rows[0].a = 0;
    rows[0].alpha = 0;
    if (abs(last.a) > 1e-12 || abs(last.alpha) > 1e-12) {
      const pre = fromRotPos(Rx(last.alpha * DEG), [last.a, 0, 0]);
      tool = matrixToTool(mul(pre, toolMatrix(tool)));
    }
  } else {
    if (abs(rows[0].a) > 1e-12 || abs(rows[0].alpha) > 1e-12)
      warning = `첫 행의 a₀=${rows[0].a}, α₀=${rows[0].alpha} 는 Standard DH 로 옮길 수 없어 제거되었습니다 (베이스 변환에 해당).`;
    for (let i = 0; i < n - 1; i++) {
      rows[i].a = rows[i + 1].a;
      rows[i].alpha = rows[i + 1].alpha;
    }
    rows[n - 1].a = 0;
    rows[n - 1].alpha = 0;
  }
  return { robot: { ...robot, convention: target, rows, tool }, warning };
}

export function matrixToTool(T) {
  const [rx, ry, rz] = rotToEuler('ZYX', rot(T));
  const round = (v) => (abs(v) < 1e-9 ? 0 : +v.toFixed(9));
  return { x: round(T[0][3]), y: round(T[1][3]), z: round(T[2][3]), rx: round(rx * RAD), ry: round(ry * RAD), rz: round(rz * RAD) };
}

// ---------------------------------------------------------------- 프리셋 (Standard DH 기준)
// 모든 수치는 공개 자료 기반 근사값 (학습용)
const zeroTool = () => ({ x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0 });
const row = (a, alpha, d, offset, min, max) => ({ a, alpha, d, offset, min, max });

export const PRESETS = {
  TX200: {
    name: 'Stäubli TX200 (근사)',
    robot: {
      convention: 'standard',
      rows: [
        row(250, -90, 642, 0, -180, 180),
        row(950, 0, 0, -90, -120, 127.5),
        row(0, 90, 0, 90, -145, 145),
        row(0, -90, 800, 0, -270, 270),
        row(0, 90, 0, 0, -120, 122.5),
        row(0, 0, 194, 0, -270, 270),
      ],
      tool: zeroTool(),
    },
    home: [0, -20, 110, 0, 0, 0],
  },
  PUMA560: {
    name: 'PUMA 560 (Corke)',
    robot: {
      convention: 'standard',
      rows: [
        row(0, 90, 0, 0, -160, 160),
        row(431.8, 0, 0, 0, -225, 45),
        row(20.32, -90, 150.05, 0, -45, 225),
        row(0, 90, 431.8, 0, -110, 170),
        row(0, -90, 0, 0, -100, 100),
        row(0, 0, 0, 0, -266, 266),
      ],
      tool: zeroTool(),
    },
    home: [0, 45, 180, 0, 45, 0],
  },
  UR5: {
    name: 'Universal Robots UR5',
    robot: {
      convention: 'standard',
      rows: [
        row(0, 90, 89.159, 0, -360, 360),
        row(-425, 0, 0, 0, -360, 360),
        row(-392.25, 0, 0, 0, -360, 360),
        row(0, 90, 109.15, 0, -360, 360),
        row(0, -90, 94.65, 0, -360, 360),
        row(0, 0, 82.3, 0, -360, 360),
      ],
      tool: zeroTool(),
    },
    home: [0, -90, 90, -90, -90, 0],
  },
};

export function clonePreset(key) {
  return JSON.parse(JSON.stringify(PRESETS[key]));
}
