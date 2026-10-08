// 왼쪽 "풀이 과정" 패널: FK / IK(해석해) / IK(수치해) 의 단계별 계산을 수식으로 보여준다
import * as K from './kinematics.js';

// KaTeX 는 동적 로드 (CDN 실패 시에도 앱은 동작하도록)
let katex = null;
const KATEX_URL = 'https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.mjs';

const SUB = ['₀', '₁', '₂', '₃', '₄', '₅', '₆', '₇', '₈', '₉'];
const sub = (i) => String(i).split('').map((c) => SUB[+c]).join('');

function fmt(v, d = 3) {
  const s = v.toFixed(d);
  return /^-0\.?0*$/.test(s) ? s.slice(1) : s;
}
const deg = (rad, d = 3) => `${fmt(rad * K.RAD, d)}^\\circ`;

// ---------------------------------------------------------------- TeX 헬퍼
function tex(s, display = true) {
  if (!katex) return `<code class="tex-fallback">${s.replace(/</g, '&lt;')}</code>`;
  return katex.renderToString(s, { throwOnError: false, displayMode: display });
}
const it = (s) => tex(s, false);

function m4(T) {
  return '\\begin{bmatrix}' + T.map((r, i) => r.map((v, j) => {
    if (i === 3) return fmt(v, 0);
    return j === 3 ? `\\color{#f2b33d}{${fmt(v, 2)}}` : fmt(v, 4);
  }).join(' & ')).join(' \\\\ ') + '\\end{bmatrix}';
}
const m3 = (R, d = 4) => '\\begin{bmatrix}' + R.map((r) => r.map((v) => fmt(v, d)).join(' & ')).join(' \\\\ ') + '\\end{bmatrix}';
const vec = (v, d = 2) => '\\begin{bmatrix}' + v.map((x) => fmt(x, d)).join(' \\\\ ') + '\\end{bmatrix}';
const mGeneric = (M, d = 3) => '\\begin{bmatrix}' + M.map((r) => r.map((v) => fmt(v, d)).join(' & ')).join(' \\\\ ') + '\\end{bmatrix}';

function section(title, body) {
  return `<section class="sol-sec"><h4>${title}</h4>${body}</section>`;
}

// <details> 열림 상태를 다시 그려도 유지
const openIds = new Set(['fk-1']);
function details(id, summary, body) {
  return `<details data-id="${id}" ${openIds.has(id) ? 'open' : ''}><summary>${summary}</summary>${body}</details>`;
}

const ang = (a, b) => Math.abs((((a - b) % 360) + 540) % 360 - 180);

// ================================================================ FK
function renderFK(st) {
  const { robot, q, fk } = st;
  const std = robot.convention === 'standard';
  const rows = robot.rows;
  let h = '';

  // 1. DH 표
  const head = std
    ? '<th>i</th><th>a<sub>i</sub></th><th>α<sub>i</sub></th><th>d<sub>i</sub></th><th>θ<sub>i</sub> = q<sub>i</sub> + off</th>'
    : '<th>i</th><th>a<sub>i-1</sub></th><th>α<sub>i-1</sub></th><th>d<sub>i</sub></th><th>θ<sub>i</sub> = q<sub>i</sub> + off</th>';
  const body = rows.map((r, i) => `<tr><td>${i + 1}</td><td>${r.a}</td><td>${r.alpha}°</td><td>${r.d}</td>` +
    `<td>${fmt(q[i], 2)} + ${r.offset} = <b>${fmt(q[i] + r.offset, 2)}°</b></td></tr>`).join('');
  h += section('1. DH 파라미터에 현재 관절각 대입', `<table class="sol-table"><thead><tr>${head}</tr></thead><tbody>${body}</tbody></table>`);

  // 2. 일반식
  const general = std
    ? 'A_i = R_z(\\theta_i)\\,T_z(d_i)\\,T_x(a_i)\\,R_x(\\alpha_i) = \\begin{bmatrix} c_{\\theta_i} & -s_{\\theta_i}c_{\\alpha_i} & s_{\\theta_i}s_{\\alpha_i} & a_i c_{\\theta_i} \\\\ s_{\\theta_i} & c_{\\theta_i}c_{\\alpha_i} & -c_{\\theta_i}s_{\\alpha_i} & a_i s_{\\theta_i} \\\\ 0 & s_{\\alpha_i} & c_{\\alpha_i} & d_i \\\\ 0&0&0&1 \\end{bmatrix}'
    : 'A_i = R_x(\\alpha_{i-1})\\,T_x(a_{i-1})\\,R_z(\\theta_i)\\,T_z(d_i) = \\begin{bmatrix} c_{\\theta_i} & -s_{\\theta_i} & 0 & a_{i-1} \\\\ s_{\\theta_i}c_{\\alpha_{i-1}} & c_{\\theta_i}c_{\\alpha_{i-1}} & -s_{\\alpha_{i-1}} & -s_{\\alpha_{i-1}}d_i \\\\ s_{\\theta_i}s_{\\alpha_{i-1}} & c_{\\theta_i}s_{\\alpha_{i-1}} & c_{\\alpha_{i-1}} & c_{\\alpha_{i-1}}d_i \\\\ 0&0&0&1 \\end{bmatrix}';
  h += section(`2. 링크 변환 일반식 (${std ? 'Standard DH' : 'Modified DH'})`,
    tex(general) + `<p class="sol-note">${it('c_x = \\cos x,\\; s_x = \\sin x')}. 전체 변환은 ${it('T_0^n = A_1 A_2 \\cdots A_n')}</p>`);

  // 3. 링크별
  let steps = '';
  rows.forEach((r, i) => {
    const k = i + 1;
    const th = (q[i] + r.offset) * K.DEG;
    const al = r.alpha * K.DEG;
    const aSym = std ? `a_${k}` : `a_${k - 1}`;
    const alSym = std ? `\\alpha_${k}` : `\\alpha_${k - 1}`;
    let b = tex(`\\theta_${k} = ${fmt(q[i], 2)}^\\circ + ${r.offset}^\\circ = ${deg(th, 2)},\\quad ${alSym} = ${r.alpha}^\\circ,\\quad ${aSym} = ${r.a},\\quad d_${k} = ${r.d}`);
    b += tex(`c_{\\theta} = ${fmt(Math.cos(th), 4)},\\; s_{\\theta} = ${fmt(Math.sin(th), 4)},\\; c_{\\alpha} = ${fmt(Math.cos(al), 4)},\\; s_{\\alpha} = ${fmt(Math.sin(al), 4)}`);
    b += tex(`A_${k} = T_${k - 1}^{${k}} = ${m4(fk.A[i])}`);
    b += k === 1
      ? tex(`T_0^{1} = A_1`)
      : tex(`T_0^{${k}} = T_0^{${k - 1}}\\,A_${k} = ${m4(fk.T[k])}`);
    steps += details(`fk-${k}`, `A${sub(k)} 와 T₀${sub(k)} &nbsp;<span class="muted">θ${sub(k)} = ${fmt(q[i] + r.offset, 2)}°</span>`, b);
  });
  h += section('3. 링크별 대입과 누적 곱', steps);

  // 4. 말단
  const n = rows.length;
  let end = tex(`T_0^{${n}} = A_1 A_2 \\cdots A_${n} = ${m4(fk.T[n])}`);
  if (!K.isIdentityTool(robot.tool)) {
    end += tex(`T_0^{tool} = T_0^{${n}}\\,T_${n}^{tool} = ${m4(fk.T[n])}${m4(fk.Ttool)} = ${m4(fk.Tend)}`);
  }
  const p = K.pos(fk.Tend);
  end += tex(`p = (${p.map((v) => fmt(v, 2)).join(',\\;')})\\ \\text{mm}`);
  h += section('4. 말단 변환행렬', end);

  // 5. 오일러각
  h += section(`5. 회전행렬 → 자세 (${K.EULER_TYPES[st.euler].label})`, renderEulerExtraction(st.euler, K.rot(fk.Tend)));
  return h;
}

function renderEulerExtraction(type, R) {
  const r = (i, j) => R[i - 1][j - 1];
  const e = K.rotToEuler(type, R);
  const f = (v) => fmt(v, 4);
  let h = tex(`R = ${m3(R)}`);
  const lines = [];
  if (type === 'ZYX') {
    h += tex('R = R_z(R_z)\\,R_y(R_y)\\,R_x(R_x) = \\begin{bmatrix} c_z c_y & \\cdots & \\cdots \\\\ s_z c_y & \\cdots & \\cdots \\\\ -s_y & c_y s_x & c_y c_x \\end{bmatrix}');
    const cy = Math.hypot(r(1, 1), r(2, 1));
    lines.push(`R_y = \\operatorname{atan2}\\!\\left(-r_{31},\\, \\sqrt{r_{11}^2 + r_{21}^2}\\right) = \\operatorname{atan2}(${f(-r(3, 1))},\\, ${f(cy)}) = ${deg(e[1])}`);
    lines.push(`R_x = \\operatorname{atan2}(r_{32},\\, r_{33}) = \\operatorname{atan2}(${f(r(3, 2))},\\, ${f(r(3, 3))}) = ${deg(e[0])}`);
    lines.push(`R_z = \\operatorname{atan2}(r_{21},\\, r_{11}) = \\operatorname{atan2}(${f(r(2, 1))},\\, ${f(r(1, 1))}) = ${deg(e[2])}`);
    if (cy < 1e-6) lines.push('\\text{짐벌락}\\ (c_y \\approx 0):\\ R_z = 0 \\text{ 으로 두고 } R_x \\text{ 만 결정}');
  } else if (type === 'XYZ') {
    h += tex('R = R_x(R_x)\\,R_y(R_y)\\,R_z(R_z) = \\begin{bmatrix} c_y c_z & -c_y s_z & s_y \\\\ \\cdots & \\cdots & -s_x c_y \\\\ \\cdots & \\cdots & c_x c_y \\end{bmatrix}');
    const cy = Math.hypot(r(2, 3), r(3, 3));
    lines.push(`R_y = \\operatorname{atan2}\\!\\left(r_{13},\\, \\sqrt{r_{23}^2 + r_{33}^2}\\right) = \\operatorname{atan2}(${f(r(1, 3))},\\, ${f(cy)}) = ${deg(e[1])}`);
    lines.push(`R_x = \\operatorname{atan2}(-r_{23},\\, r_{33}) = \\operatorname{atan2}(${f(-r(2, 3))},\\, ${f(r(3, 3))}) = ${deg(e[0])}`);
    lines.push(`R_z = \\operatorname{atan2}(-r_{12},\\, r_{11}) = \\operatorname{atan2}(${f(-r(1, 2))},\\, ${f(r(1, 1))}) = ${deg(e[2])}`);
    if (cy < 1e-6) lines.push('\\text{짐벌락}\\ (c_y \\approx 0):\\ R_z = 0 \\text{ 으로 두고 } R_x \\text{ 만 결정}');
  } else {
    h += tex('R = R_z(\\phi)\\,R_y(\\theta)\\,R_z(\\psi) = \\begin{bmatrix} \\cdots & \\cdots & c_\\phi s_\\theta \\\\ \\cdots & \\cdots & s_\\phi s_\\theta \\\\ -s_\\theta c_\\psi & s_\\theta s_\\psi & c_\\theta \\end{bmatrix}');
    const sb = Math.hypot(r(1, 3), r(2, 3));
    lines.push(`\\theta = \\operatorname{atan2}\\!\\left(\\sqrt{r_{13}^2 + r_{23}^2},\\, r_{33}\\right) = \\operatorname{atan2}(${f(sb)},\\, ${f(r(3, 3))}) = ${deg(e[1])}`);
    lines.push(`\\phi = \\operatorname{atan2}(r_{23},\\, r_{13}) = \\operatorname{atan2}(${f(r(2, 3))},\\, ${f(r(1, 3))}) = ${deg(e[0])}`);
    lines.push(`\\psi = \\operatorname{atan2}(r_{32},\\, -r_{31}) = \\operatorname{atan2}(${f(r(3, 2))},\\, ${f(-r(3, 1))}) = ${deg(e[2])}`);
    if (sb < 1e-6) lines.push('\\text{특이점}\\ (s_\\theta \\approx 0):\\ \\phi \\pm \\psi \\text{ 만 결정됨 → } \\psi = 0');
  }
  return h + lines.map((l) => tex(l)).join('');
}

// ================================================================ IK (해석해)
let selectedBranch = null; // "sigma,eps,w"
const branchKey = (s) => `${s.shoulder},${s.elbow},${s.wrist}`;
const branchLabel = (s) =>
  `어깨 ${s.shoulder > 0 ? 'r>0' : 'r<0'} · 팔꿈치 ${s.elbow > 0 ? 'ψ>0' : 'ψ<0'} · 손목 ${s.singular ? '특이' : s.wrist > 0 ? '+' : '−'}`;

function renderIKAnalytic(st) {
  if (!st.targetT) return '<p class="sol-note">목표 자세가 없습니다.</p>';
  const res = K.analyticIK(st.robot, st.targetT);
  let h = '';

  // 0. 조건
  const checks = res.checks.map((c) => `<li class="${c.ok ? 'ok' : 'bad'}">${c.ok ? '✔' : '✘'} ${c.label}</li>`).join('');
  let pre = `<ul class="sol-checks">${checks}</ul>`;
  if (st.robot.convention !== 'standard') pre += '<p class="sol-note">Modified DH 로봇은 동일한 Standard DH 로 변환해서 풉니다 (관절각 q 는 동일).</p>';
  if (res.convWarning) pre += `<p class="sol-note warn">${res.convWarning}</p>`;
  h += section('0. 해석해 적용 조건 (구면 손목 + 평행한 관절 2·3)', pre);
  if (!res.applicable) {
    return h + '<p class="sol-note warn">이 로봇 구조는 위 조건을 만족하지 않아 이 방식의 해석해를 쓸 수 없습니다. "IK 수치해" 탭에서 반복 해법의 과정을 확인하세요.</p>';
  }
  const R = res.robot.rows;

  // 1. 툴 제거
  let b1 = tex(`T_{target} = ${m4(st.targetT)}`);
  b1 += K.isIdentityTool(res.robot.tool)
    ? `<p class="sol-note">툴 변환이 단위행렬이므로 ${it('T_0^6 = T_{target}')}</p>`
    : tex(`T_0^6 = T_{target}\\,(T_6^{tool})^{-1} = ${m4(res.T06)}`);
  h += section('1. 목표에서 툴 변환을 제거 → T₀⁶', b1);

  // 2. 손목 중심
  h += section('2. 손목 중심 (관절 4·5·6 축의 교점)',
    `<p class="sol-note">구면 손목에서는 손목 중심이 ${it('q_1, q_2, q_3')} 에만 의존합니다. ${it('o_6 = p_w + d_6 z_6')} 이므로</p>` +
    tex(`p_w = p - d_6\\,z_6 = ${vec(res.p)} - ${fmt(res.d6, 2)}${vec(res.z6, 4)} = ${vec(res.pw)}`));

  if (res.unreachable) return h + `<p class="sol-note bad">${res.unreachable}</p>`;

  // 해 선택
  const sols = res.solutions;
  if (!sols.length) {
    h += section('3. θ₁', renderTheta1(res));
    return h + '<p class="sol-note bad">팔꿈치 방정식의 |cos ψ| > 1 → 목표가 작업영역 밖입니다.</p>';
  }
  if (!selectedBranch || !sols.some((s) => branchKey(s) === selectedBranch)) {
    let best = sols[0], bd = Infinity;
    for (const s of sols) {
      const d = s.q.reduce((acc, v, i) => acc + ang(v, st.q[i]), 0);
      if (d < bd) { bd = d; best = s; }
    }
    selectedBranch = branchKey(best);
  }
  const sel = sols.find((s) => branchKey(s) === selectedBranch);
  const det = sel.detail;
  const sh = det.sh;

  h += `<div class="sol-branch"><label>풀이할 해 선택 <select id="sol-branch">${sols.map((s) =>
    `<option value="${branchKey(s)}" ${branchKey(s) === selectedBranch ? 'selected' : ''}>${branchLabel(s)}${s.inLimits ? '' : ' (한계 초과)'}</option>`).join('')}</select></label></div>`;

  // 3. θ1
  h += section('3. θ₁ — 손목 중심을 위에서 내려다보기', renderTheta1(res, sh.sigma));

  // 4. θ2, θ3
  const sa3 = res.sa3;
  let b4 = `<p class="sol-note">손목 중심을 프레임 {1} 에서 보면 관절 2·3 이 만드는 평면 2링크 문제가 됩니다 (${it('z')} 성분은 ${it('d_2 + d_3')} 로 일정).</p>`;
  b4 += tex(`p_w^{(1)} = (T_0^1)^{-1} p_w = ${vec(sh.p1)} \\quad (Z = ${fmt(sh.p1[2], 2)} \\approx d_2 + d_3 = ${fmt(res.D, 2)})`);
  b4 += `<p class="sol-note">링크 3 의 ${it('a_3')} 와 손목까지의 ${it('d_4')} 를 하나의 등가 링크로 묶습니다.</p>`;
  b4 += tex(`L_3 = \\sqrt{a_3^2 + d_4^2} = \\sqrt{${fmt(res.a3, 2)}^2 + ${fmt(res.d4, 2)}^2} = ${fmt(res.L3, 3)}`);
  b4 += tex(`\\varphi = \\operatorname{atan2}(-d_4 \\sin\\alpha_3,\\; a_3) = \\operatorname{atan2}(${fmt(-res.d4 * sa3, 2)},\\, ${fmt(res.a3, 2)}) = ${deg(res.phi)}`);
  b4 += `<p class="sol-note">코사인 법칙 (${it('a_2 = ' + fmt(res.a2, 2))}):</p>`;
  b4 += tex(`\\cos\\psi = \\frac{X^2 + Y^2 - a_2^2 - L_3^2}{2 a_2 L_3} = \\frac{${fmt(sh.p1[0], 2)}^2 + ${fmt(sh.p1[1], 2)}^2 - ${fmt(res.a2, 2)}^2 - ${fmt(res.L3, 2)}^2}{2 \\cdot ${fmt(res.a2, 2)} \\cdot ${fmt(res.L3, 2)}} = ${fmt(sh.cpsi, 5)}`);
  b4 += tex(`\\psi = ${sel.elbow > 0 ? '+' : '-'}\\arccos(${fmt(sh.cpsi, 5)}) = ${deg(det.psi)} \\quad (\\psi = \\theta_3 + \\varphi)`);
  b4 += tex(`\\theta_2 = \\operatorname{atan2}(Y, X) - \\operatorname{atan2}(L_3 \\sin\\psi,\\; a_2 + L_3\\cos\\psi) = ${deg(Math.atan2(sh.p1[1], sh.p1[0]))} - ${deg(Math.atan2(res.L3 * Math.sin(det.psi), res.a2 + res.L3 * Math.cos(det.psi)))} = ${deg(det.th2)}`);
  b4 += tex(`\\theta_3 = \\psi - \\varphi = ${deg(det.psi)} - ${deg(res.phi)} = ${deg(det.th3)}`);
  h += section('4. θ₂, θ₃ — 평면 2링크 (코사인 법칙)', b4);

  // 5. 손목
  const sgn = res.s > 0 ? '' : '-';
  const r36 = det.R36;
  let b5 = tex(`R_0^3 = ${m3(det.R03)}`);
  b5 += tex(`R_3^6 = (R_0^3)^T R_0^6 = ${m3(r36)}`);
  b5 += `<p class="sol-note">${it(`\\alpha_4 = ${R[3].alpha}^\\circ,\\ \\alpha_5 = ${R[4].alpha}^\\circ`)} 이므로 ${it(`R_x(\\alpha_4) R_z(\\theta_5) R_x(\\alpha_5) = R_y(${sgn}\\theta_5)`)}. 따라서 ZYZ 오일러각과 같은 꼴입니다 (${it(`\\beta = ${sgn}\\theta_5`)}).</p>`;
  b5 += tex('R_3^6 = R_z(\\theta_4)\\,R_y(\\beta)\\,R_z(\\theta_6) = \\begin{bmatrix} c_4 c_\\beta c_6 - s_4 s_6 & -c_4 c_\\beta s_6 - s_4 c_6 & c_4 s_\\beta \\\\ s_4 c_\\beta c_6 + c_4 s_6 & -s_4 c_\\beta s_6 + c_4 c_6 & s_4 s_\\beta \\\\ -s_\\beta c_6 & s_\\beta s_6 & c_\\beta \\end{bmatrix}');
  const w = sel.wrist > 0 ? '+' : '-';
  const f = (v) => fmt(v, 4);
  if (sel.singular) {
    b5 += `<p class="sol-note warn">${it('s_\\beta \\approx 0')} → 손목 특이점. ${it('\\theta_4')} 와 ${it('\\theta_6')} 가 같은 축으로 정렬되어 합(또는 차)만 결정됩니다. ${it('\\theta_4 = 0')} 으로 두고 ${it('\\theta_6')} 를 구합니다.</p>`;
    b5 += tex(`\\theta_5 = ${deg(sel.theta[4])},\\quad \\theta_4 = 0^\\circ,\\quad \\theta_6 = ${deg(sel.theta[5])}`);
  } else {
    b5 += tex(`\\beta = \\operatorname{atan2}\\!\\left(${w}\\sqrt{r_{13}^2 + r_{23}^2},\\; r_{33}\\right) = \\operatorname{atan2}(${w}${f(det.sb)},\\, ${f(r36[2][2])}) = ${deg(det.b)} \\;\\Rightarrow\\; \\theta_5 = ${sgn}\\beta = ${deg(sel.theta[4])}`);
    b5 += tex(`\\theta_4 = \\operatorname{atan2}(${w} r_{23},\\; ${w} r_{13}) = \\operatorname{atan2}(${f(sel.wrist * r36[1][2])},\\, ${f(sel.wrist * r36[0][2])}) = ${deg(sel.theta[3])}`);
    b5 += tex(`\\theta_6 = \\operatorname{atan2}(${w} r_{32},\\; ${sel.wrist > 0 ? '-' : '+'} r_{31}) = \\operatorname{atan2}(${f(sel.wrist * r36[2][1])},\\, ${f(-sel.wrist * r36[2][0])}) = ${deg(sel.theta[5])}`);
    b5 += '<p class="sol-note">±√ 의 부호 선택이 손목 flip (θ₅ 부호 반전, θ₄·θ₆ 가 180° 이동) 에 해당합니다.</p>';
  }
  h += section('5. θ₄, θ₅, θ₆ — 손목 자세 (오일러각 추출)', b5);

  // 6. q
  const qRows = R.map((r, i) => {
    const ok = sel.q[i] >= r.min - 1e-9 && sel.q[i] <= r.max + 1e-9;
    return `<tr><td>${i + 1}</td><td>${fmt(sel.theta[i] * K.RAD, 3)}°</td><td>${r.offset}°</td><td><b>${fmt(sel.q[i], 3)}°</b></td><td class="${ok ? 'ok' : 'bad'}">${ok ? '✔' : '✘'} [${r.min}, ${r.max}]</td></tr>`;
  }).join('');
  let b6 = `<table class="sol-table"><thead><tr><th>i</th><th>θ<sub>i</sub></th><th>offset</th><th>q<sub>i</sub> = θ<sub>i</sub> − off</th><th>한계</th></tr></thead><tbody>${qRows}</tbody></table>`;
  b6 += `<p class="sol-note ${sel.posErr < 1e-3 ? 'ok' : 'bad'}">검산: FK(q) 와 목표의 차이 → 위치 ${sel.posErr.toExponential(2)} mm, 자세 ${(sel.oriErr * K.RAD).toExponential(2)}°</p>`;
  b6 += `<button data-apply="${branchKey(sel)}">이 해를 로봇에 적용</button>`;
  h += section('6. 관절각 q = θ − offset', b6);

  // 7. 전체 해
  const all = sols.map((s, idx) => `<tr class="${branchKey(s) === selectedBranch ? 'sel' : ''}" data-branch="${branchKey(s)}">
      <td>${idx + 1}</td><td>${branchLabel(s)}</td>${s.q.map((v) => `<td>${fmt(v, 1)}</td>`).join('')}
      <td class="${s.inLimits ? 'ok' : 'bad'}">${s.inLimits ? '✔' : '✘'}</td>
      <td><button data-apply="${branchKey(s)}">적용</button></td></tr>`).join('');
  h += section(`7. 모든 해 (${sols.length}개)`,
    `<div class="sol-scroll"><table class="sol-table small"><thead><tr><th>#</th><th>구성</th>${R.map((_, i) => `<th>q${sub(i + 1)}</th>`).join('')}<th>한계</th><th></th></tr></thead><tbody>${all}</tbody></table></div>` +
    '<p class="sol-note">행을 클릭하면 그 해의 풀이를 위에서 볼 수 있습니다.</p>');
  return h;
}

function renderTheta1(res, selSigma) {
  let b = `<p class="sol-note">관절 2·3 이 움직이는 평면은 ${it('z_1')} 방향으로 ${it('d_2 + d_3')} 만큼 떨어져 있습니다. 이 옆 오프셋을 ${it('L')} 이라 하면, 손목 중심의 수평 위치는 ${it('r\\,(c_1, s_1) + L\\,(-s_1, c_1)')} 입니다.</p>`;
  b += tex(`L = -(d_2 + d_3)\\sin\\alpha_1 = -(${fmt(res.D, 2)})(${res.sa1}) = ${fmt(res.Lat, 2)}`);
  b += tex(`\\rho^2 = p_{wx}^2 + p_{wy}^2 = ${fmt(res.pw[0], 2)}^2 + ${fmt(res.pw[1], 2)}^2 = ${fmt(res.rho2, 1)}`);
  b += tex(`r = \\pm\\sqrt{\\rho^2 - L^2} = \\pm ${fmt(Math.sqrt(Math.max(res.rho2 - res.Lat ** 2, 0)), 3)}`);
  b += tex('\\theta_1 = \\operatorname{atan2}(p_{wy}, p_{wx}) - \\operatorname{atan2}(L, r)');
  for (const sh of res.shoulders || []) {
    const mark = sh.sigma === selSigma ? ' \\;\\leftarrow \\text{선택}' : '';
    b += tex(`r = ${fmt(sh.r, 2)}:\\quad \\theta_1 = ${deg(Math.atan2(res.pw[1], res.pw[0]))} - ${deg(Math.atan2(res.Lat, sh.r))} = ${deg(sh.th1)}${sh.reachable ? '' : '\\ (\\text{팔이 닿지 않음})'}${mark}`);
  }
  b += '<p class="sol-note">r > 0 은 손목이 어깨 앞쪽, r < 0 은 몸을 180° 돌려 뒤로 뻗는 자세(어깨 뒤집기) 입니다.</p>';
  return b;
}

// ================================================================ IK (수치해)
let numeric = null; // { key, q0, trace, res }
let numericStart = 'home'; // 'home' | 'zero' | 'current'
const tKey = (T) => T.flat().map((v) => v.toFixed(6)).join(',');
const numericKey = (st) => `${tKey(st.targetT)}|${JSON.stringify(st.robot)}|${numericStart}`;

function startQ(st) {
  if (numericStart === 'zero') return st.robot.rows.map(() => 0);
  if (numericStart === 'home' && st.home && st.home.length === st.robot.rows.length) return st.home.slice();
  return st.q.slice();
}

function computeNumeric(st) {
  const trace = [];
  const q0 = startQ(st);
  const res = K.inverse(st.robot, st.targetT, q0, { trace, restarts: 5, respectLimits: true });
  numeric = { key: numericKey(st), q0, trace, res };
}

function renderIKNumeric(st) {
  if (!st.targetT) return '<p class="sol-note">목표 자세가 없습니다.</p>';
  // '현재 자세' 출발은 버튼을 누를 때만 다시 계산 (적용 후 q 가 바뀌어도 기록 유지)
  if (!numeric || numeric.key !== numericKey(st)) computeNumeric(st);
  const { q0, trace, res } = numeric;
  const first = trace[0];
  const opt = (v, l) => `<option value="${v}" ${numericStart === v ? 'selected' : ''}>${l}</option>`;
  let h = `<div class="sol-branch"><label>출발 자세 q₀ <select id="sol-numeric-start">${opt('home', '홈 자세')}${opt('zero', '영점 (q = 0)')}${opt('current', '현재 자세 (누른 시점)')}</select></label></div>`;

  let b1 = '<p class="sol-note">현재 관절각에서 출발해 오차를 줄이는 방향으로 조금씩 관절을 움직입니다.</p>';
  b1 += tex('e = \\begin{bmatrix} (p_{target} - p(q)) / L \\\\ \\log\\!\\big(R_{target} R(q)^T\\big)^\\vee \\end{bmatrix} \\in \\mathbb{R}^6');
  b1 += tex('\\Delta q = J^T \\left(J J^T + \\lambda^2 I\\right)^{-1} e, \\qquad q \\leftarrow q + \\Delta q');
  b1 += `<p class="sol-note">${it('J')}: 기하학적 자코비안 (${st.robot.convention === 'standard' ? it('J_i = [\\,z_{i-1} \\times (p_e - o_{i-1});\\ z_{i-1}\\,]') : it('J_i = [\\,z_{i} \\times (p_e - o_{i});\\ z_{i}\\,]')}),
    ${it('\\lambda = ' + (first.lambda ?? 0.05))} (감쇠 계수, 특이점 근처 발산 방지), ${it('L = 1000')} (mm→m, 위치·회전 오차 단위 맞춤), 한 번에 최대 0.3 rad 이동.</p>`;
  h += section('1. Damped Least Squares (Levenberg–Marquardt)', b1);

  if (first.J) {
    let b2 = tex(`q_0 = ${vec(q0, 2)}^\\circ`);
    b2 += tex(`e_0 = ${vec(first.e, 4)} \\quad (\\lVert e_p \\rVert = ${fmt(first.posErr, 2)}\\ \\text{mm},\\ \\lVert e_o \\rVert = ${deg(first.oriErr, 2)})`);
    b2 += tex(`J(q_0) = ${mGeneric(first.J, 3)}`);
    const dqDeg = first.dq.map((v) => v * K.RAD);
    b2 += tex(`\\Delta q_0 = ${vec(dqDeg, 2)}^\\circ${first.scale < 1 ? `\\times ${fmt(first.scale, 3)}\\ (\\text{스텝 제한})` : ''}`);
    if (trace[1] && trace[1].attempt === 0) b2 += tex(`q_1 = q_0 + \\Delta q_0 = ${vec(trace[1].q, 2)}^\\circ`);
    h += section('2. 첫 번째 반복 상세', b2);
  } else {
    h += section('2. 첫 번째 반복', '<p class="sol-note">시작 자세가 이미 목표와 일치합니다.</p>');
  }

  const rows = trace.map((t) => `<tr class="${t.it === 0 && t.attempt > 0 ? 'restart' : ''}"><td>${t.attempt}</td><td>${t.it}</td><td>${t.posErr.toExponential(2)}</td><td>${(t.oriErr * K.RAD).toExponential(2)}</td><td class="qcol">${t.q.map((v) => fmt(v, 1)).join(', ')}</td></tr>`).join('');
  h += section(`3. 반복 기록 (${trace.length}회)`,
    `<div class="sol-scroll tall"><table class="sol-table small"><thead><tr><th>시도</th><th>k</th><th>|e<sub>p</sub>| mm</th><th>|e<sub>o</sub>| °</th><th>q (°)</th></tr></thead><tbody>${rows}</tbody></table></div>` +
    '<p class="sol-note">시도 번호가 바뀌는 행은 수렴에 실패해서 랜덤한 관절각으로 다시 시작한 경우입니다 (지역 최소값 탈출).</p>');

  let b4 = `<p class="sol-note ${res.success ? 'ok' : 'bad'}">${res.success ? '✔ 수렴' : '✘ 수렴 실패 (가장 가까운 해)'} — 위치오차 ${res.posErr.toExponential(2)} mm, 자세오차 ${(res.oriErr * K.RAD).toExponential(2)}°</p>`;
  b4 += tex(`q^* = ${vec(res.q, 3)}^\\circ`);
  b4 += '<p class="sol-note">수치해는 출발점에서 가까운 해 <b>하나</b>만 찾습니다. 다른 해를 보려면 출발 자세를 바꾸거나 해석해 탭을 보세요.</p>';
  b4 += `<div class="row buttons"><button data-apply-numeric>이 해를 로봇에 적용</button><button data-recompute>다시 풀기</button></div>`;
  h += section('4. 결과', b4);
  return h;
}

// ================================================================ 패널
export function createSolutionPanel(root, ctx) {
  let mode = 'fk';
  let scheduled = false;

  const tabs = root.querySelector('.sol-tabs');
  const content = root.querySelector('.sol-content');

  tabs.querySelectorAll('button').forEach((b) => b.addEventListener('click', () => {
    mode = b.dataset.mode;
    tabs.querySelectorAll('button').forEach((x) => x.classList.toggle('active', x === b));
    render();
  }));

  content.addEventListener('toggle', (e) => {
    const id = e.target.dataset && e.target.dataset.id;
    if (!id) return;
    if (e.target.open) openIds.add(id); else openIds.delete(id);
  }, true);

  content.addEventListener('change', (e) => {
    if (e.target.id === 'sol-branch') { selectedBranch = e.target.value; render(); }
    if (e.target.id === 'sol-numeric-start') { numericStart = e.target.value; computeNumeric(ctx.getState()); render(); }
  });

  content.addEventListener('click', (e) => {
    const st = ctx.getState();
    const applyBtn = e.target.closest('[data-apply]');
    if (applyBtn) {
      const res = K.analyticIK(st.robot, st.targetT);
      const s = res.solutions.find((x) => branchKey(x) === applyBtn.dataset.apply);
      if (s) { selectedBranch = branchKey(s); ctx.applyQ(s.q); }
      return;
    }
    if (e.target.closest('[data-apply-numeric]') && numeric) { ctx.applyQ(numeric.res.q); return; }
    if (e.target.closest('[data-recompute]')) { computeNumeric(st); render(); return; }
    if (e.target.closest('[data-reveal-fk]')) { ctx.revealFk(); return; }
    if (e.target.closest('[data-reveal-ik]')) { ctx.revealIk(); return; }
    const row = e.target.closest('tr[data-branch]');
    if (row) { selectedBranch = row.dataset.branch; render(); }
  });

  function render() {
    scheduled = false;
    if (root.classList.contains('collapsed')) return;
    const st = ctx.getState();
    if (!st.fk) return;
    const scrollTop = content.scrollTop;
    let html;
    if (mode === 'fk' && st.quizFk) {
      html = '<p class="sol-note warn">정기구학 연습문제를 푸는 중이라 풀이를 가렸습니다.</p><button data-reveal-fk>풀이 보기 (문제 종료)</button>';
    } else if (mode !== 'fk' && st.quizIk && !st.quizIk.revealed) {
      html = '<p class="sol-note warn">역기구학 연습문제를 푸는 중이라 풀이를 가렸습니다.</p><button data-reveal-ik>풀이 보기</button>';
    } else if (mode === 'fk') html = renderFK(st);
    else if (mode === 'ika') html = renderIKAnalytic(st);
    else html = renderIKNumeric(st);
    content.innerHTML = html;
    content.scrollTop = scrollTop;
  }

  function update() {
    if (scheduled) return;
    scheduled = true;
    requestAnimationFrame(render);
  }

  import(KATEX_URL).then((m) => { katex = m.default; update(); }).catch(() => { /* 텍스트로 표시 */ });

  return { update, render };
}
