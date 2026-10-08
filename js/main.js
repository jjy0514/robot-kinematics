import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { CSS2DRenderer, CSS2DObject } from 'three/addons/renderers/CSS2DRenderer.js';
import * as K from './kinematics.js';
import { createSolutionPanel } from './solution.js';

// 계산은 mm, 화면은 m 단위
const S = 0.001;
const STORAGE_KEY = 'robot-kinematics-state-v1';

// ============================================================ 상태
const state = {
  presetKey: 'TX200',
  robot: null,
  home: null,
  q: null,
  euler: 'ZYX',
  targetT: null,
  fk: null,
  view: {
    showFrames: [],      // 프레임별 표시 여부
    showTool: true,
    showLabels: true,
    showJointAxes: true,
    dhColors: false,
    showTarget: true,
    showGrid: true,
    axisScale: 1,
    linkOpacity: 0.75,
  },
  quizFk: null,          // { q }
  quizIk: null,          // { q, T, revealed }
};

function loadState() {
  try {
    const s = JSON.parse(localStorage.getItem(STORAGE_KEY));
    if (s && s.robot && Array.isArray(s.q) && s.q.length === s.robot.rows.length) {
      Object.assign(state, { presetKey: s.presetKey, robot: s.robot, q: s.q, euler: s.euler || 'ZYX' });
      state.home = (K.PRESETS[s.presetKey] || {}).home || s.q.map(() => 0);
      return;
    }
  } catch (_) { /* 저장값 없음/손상 → 기본값 */ }
  const p = K.clonePreset(state.presetKey);
  state.robot = p.robot;
  state.home = p.home;
  state.q = p.home.slice();
}

function saveState() {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify({
      presetKey: state.presetKey, robot: state.robot, q: state.q, euler: state.euler,
    }));
  } catch (_) { /* 저장 불가 환경은 무시 */ }
}

// ============================================================ 유틸
const $ = (sel) => document.querySelector(sel);
const n = () => state.robot.rows.length;
const isStd = () => state.robot.convention === 'standard';

function fmt(v, d = 2) {
  const s = v.toFixed(d);
  return /^-0\.?0*$/.test(s) ? s.slice(1) : s;
}

function matHTML(T, rd = 4, pd = 2) {
  const rows = T.map((r, i) => {
    const cells = r.map((v, j) => {
      const isP = j === 3 && i < 3;
      return `<td class="${isP ? 'p' : ''}">${fmt(v, i === 3 ? 0 : isP ? pd : rd)}</td>`;
    }).join('');
    return `<tr class="${i === 3 ? 'last' : ''}">${cells}</tr>`;
  }).join('');
  return `<table class="matrix">${rows}</table>`;
}

const vecP = (T) => new THREE.Vector3(T[0][3] * S, T[1][3] * S, T[2][3] * S);
const vecX = (T) => new THREE.Vector3(T[0][0], T[1][0], T[2][0]);
const vecZ = (T) => new THREE.Vector3(T[0][2], T[1][2], T[2][2]);

function toMatrix4(T) {
  const m = new THREE.Matrix4();
  m.set(
    T[0][0], T[0][1], T[0][2], T[0][3] * S,
    T[1][0], T[1][1], T[1][2], T[1][3] * S,
    T[2][0], T[2][1], T[2][2], T[2][3] * S,
    0, 0, 0, 1,
  );
  return m;
}

function fromMatrix4(m) {
  const e = m.elements; // column-major
  return [
    [e[0], e[4], e[8], e[12] / S],
    [e[1], e[5], e[9], e[13] / S],
    [e[2], e[6], e[10], e[14] / S],
    [0, 0, 0, 1],
  ];
}

function robotSize() {
  const r = state.robot;
  let L = r.rows.reduce((s, row) => s + Math.abs(row.a) + Math.abs(row.d), 0);
  L += Math.hypot(r.tool.x, r.tool.y, r.tool.z);
  return Math.max(L * S, 0.3); // m
}

function poseText(T) {
  const p = K.pos(T);
  const e = K.rotToEuler(state.euler, K.rot(T)).map((v) => v * K.RAD);
  return { p, e };
}

// ============================================================ Three.js 장면
const viewport = $('#viewport');
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x14171c);
viewport.appendChild(renderer.domElement);

const labelRenderer = new CSS2DRenderer();
labelRenderer.domElement.style.position = 'absolute';
labelRenderer.domElement.style.top = '0';
labelRenderer.domElement.style.pointerEvents = 'none';
viewport.appendChild(labelRenderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 100);
camera.up.set(0, 0, 1);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(3, -4, 6);
scene.add(sun);

let grid = null;

const unitCyl = new THREE.CylinderGeometry(1, 1, 1, 28);
const unitCone = new THREE.ConeGeometry(1, 1, 20);
const Y_UP = new THREE.Vector3(0, 1, 0);

const mats = {
  link: new THREE.MeshStandardMaterial({ color: 0xf0b23a, roughness: 0.55, metalness: 0.1 }),
  linkD: new THREE.MeshStandardMaterial({ color: 0x3f7cf0, roughness: 0.55 }),
  linkA: new THREE.MeshStandardMaterial({ color: 0xf04a4a, roughness: 0.55 }),
  joint: new THREE.MeshStandardMaterial({ color: 0x47505f, roughness: 0.4, metalness: 0.3 }),
  jointHi: new THREE.MeshStandardMaterial({ color: 0x2fd3c2, roughness: 0.4, emissive: 0x0b4a44 }),
  base: new THREE.MeshStandardMaterial({ color: 0x353b46, roughness: 0.7 }),
  gripper: new THREE.MeshStandardMaterial({ color: 0x9aa4b4, roughness: 0.4, metalness: 0.5 }),
  jointLine: new THREE.LineDashedMaterial({ color: 0x8a98ad, dashSize: 0.04, gapSize: 0.03, transparent: true, opacity: 0.7 }),
};
const axisColors = { x: 0xff5a5a, y: 0x44d17a, z: 0x4f8dff };

function applyOpacity() {
  const o = state.view.linkOpacity;
  for (const m of [mats.link, mats.linkD, mats.linkA, mats.joint, mats.jointHi, mats.base, mats.gripper]) {
    m.opacity = o;
    m.transparent = o < 0.999;
    m.depthWrite = o >= 0.999;
    m.needsUpdate = true;
  }
}

// p1 → p2 를 잇는 원기둥 배치 (단위 원기둥 스케일링)
function placeSegment(mesh, p1, p2, r) {
  const dir = new THREE.Vector3().subVectors(p2, p1);
  const len = dir.length();
  if (len < 1e-6) { mesh.visible = false; return; }
  mesh.visible = true;
  mesh.position.copy(p1).addScaledVector(dir, 0.5);
  mesh.quaternion.setFromUnitVectors(Y_UP, dir.normalize());
  mesh.scale.set(r, len, r);
}

function makeLabel(html, cls) {
  const div = document.createElement('div');
  div.className = cls;
  div.innerHTML = html;
  return new CSS2DObject(div);
}

// 좌표축 (x 빨강, y 초록, z 파랑) + 이름표
function makeAxes(len, sub, { target = false } = {}) {
  const g = new THREE.Group();
  g.userData.labels = [];
  const shaftR = len * 0.022;
  const headLen = len * 0.2;
  const headR = len * 0.065;
  const dirs = { x: new THREE.Vector3(1, 0, 0), y: new THREE.Vector3(0, 1, 0), z: new THREE.Vector3(0, 0, 1) };
  for (const ax of ['x', 'y', 'z']) {
    const mat = new THREE.MeshBasicMaterial({
      color: axisColors[ax], depthTest: false, transparent: true, opacity: target ? 0.65 : 1,
    });
    const d = dirs[ax];
    const shaft = new THREE.Mesh(unitCyl, mat);
    placeSegment(shaft, new THREE.Vector3(), d.clone().multiplyScalar(len - headLen), shaftR);
    const head = new THREE.Mesh(unitCone, mat);
    head.scale.set(headR, headLen, headR);
    head.position.copy(d).multiplyScalar(len - headLen / 2);
    head.quaternion.setFromUnitVectors(Y_UP, d);
    shaft.renderOrder = head.renderOrder = 10;
    g.add(shaft, head);
    const lbl = makeLabel(`${ax}<sub>${sub}</sub>`, `axis-label ${ax}`);
    lbl.position.copy(d).multiplyScalar(len * 1.12);
    g.add(lbl);
    g.userData.labels.push(lbl);
  }
  const fl = makeLabel(`{${sub}}`, `frame-label${target ? ' target' : ''}`);
  fl.position.set(-len * 0.12, -len * 0.12, -len * 0.12);
  g.add(fl);
  g.userData.labels.push(fl);
  return g;
}

function setGroupVisible(g, vis) {
  g.visible = vis;
  // CSS2DRenderer 는 부모 visible 을 보지 않으므로 이름표를 직접 끈다
  for (const l of g.userData.labels || []) l.visible = vis && state.view.showLabels;
}

function disposeGroup(g) {
  g.traverse((o) => {
    if (o.isCSS2DObject && o.element.parentNode) o.element.parentNode.removeChild(o.element);
    if (o.material && !Object.values(mats).includes(o.material)) o.material.dispose();
    if (o.geometry && o.geometry !== unitCyl && o.geometry !== unitCone) o.geometry.dispose();
  });
  g.removeFromParent();
}

// ------------------------------------------------------------ 로봇 비주얼
const vis = {
  root: null, links: [], joints: [], jointLines: [], frames: [], toolFrame: null, toolSeg: null, gripper: null,
  rLink: 0.05, rJoint: 0.07, hJoint: 0.15, L: 1,
};
let highlightedJoint = -1;

function buildRobotVisual() {
  if (vis.root) disposeGroup(vis.root);
  const root = new THREE.Group();
  const L = robotSize();
  vis.L = L;
  vis.rLink = L * 0.018;
  vis.rJoint = vis.rLink * 1.45;
  vis.hJoint = vis.rLink * 3.4;
  const axisLen = L * 0.11 * state.view.axisScale;

  // 베이스
  const base = new THREE.Mesh(unitCyl, mats.base);
  placeSegment(base, new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, vis.rLink * 0.8), vis.rLink * 3);
  root.add(base);

  vis.links = [];
  vis.joints = [];
  vis.jointLines = [];
  for (let i = 0; i < n(); i++) {
    const segs = [new THREE.Mesh(unitCyl, mats.link), new THREE.Mesh(unitCyl, mats.link)];
    root.add(...segs);
    vis.links.push(segs);
    const j = new THREE.Mesh(unitCyl, mats.joint);
    root.add(j);
    vis.joints.push(j);
    const lg = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, 0, 1)]);
    const line = new THREE.Line(lg, mats.jointLine);
    root.add(line);
    vis.jointLines.push(line);
  }
  vis.toolSeg = new THREE.Mesh(unitCyl, mats.link);
  root.add(vis.toolSeg);

  // 간이 그리퍼 (TCP = 마지막 프레임/툴 프레임 원점에서 끝남)
  const gr = new THREE.Group();
  gr.matrixAutoUpdate = false;
  const r = vis.rLink;
  const flange = new THREE.Mesh(unitCyl, mats.gripper);
  placeSegment(flange, new THREE.Vector3(0, 0, -r * 2.6), new THREE.Vector3(0, 0, -r * 2.2), r * 1.4);
  const palm = new THREE.Mesh(new THREE.BoxGeometry(r * 0.8, r * 3, r * 0.5), mats.gripper);
  palm.position.set(0, 0, -r * 1.95);
  const f1 = new THREE.Mesh(new THREE.BoxGeometry(r * 0.6, r * 0.35, r * 1.7), mats.gripper);
  const f2 = f1.clone();
  f1.position.set(0, r * 1.2, -r * 0.85);
  f2.position.set(0, -r * 1.2, -r * 0.85);
  gr.add(flange, palm, f1, f2);
  root.add(gr);
  vis.gripper = gr;

  // 좌표계 {0}..{n}, {tool}
  vis.frames = [];
  for (let k = 0; k <= n(); k++) {
    const f = makeAxes(axisLen, String(k));
    f.matrixAutoUpdate = false;
    root.add(f);
    vis.frames.push(f);
  }
  vis.toolFrame = makeAxes(axisLen, 'tool');
  vis.toolFrame.matrixAutoUpdate = false;
  root.add(vis.toolFrame);

  scene.add(root);
  vis.root = root;

  // 그리드
  if (grid) { grid.geometry.dispose(); grid.material.dispose(); grid.removeFromParent(); }
  const gsize = Math.ceil(L * 1.8) || 2;
  grid = new THREE.GridHelper(gsize, gsize * 5, 0x3a4252, 0x262c37);
  grid.rotation.x = Math.PI / 2;
  grid.visible = state.view.showGrid;
  scene.add(grid);

  rebuildTargetVisual();
  applyOpacity();
  applyViewToggles();
}

function updatePose() {
  const fk = K.forward(state.robot, state.q);
  state.fk = fk;
  const T = fk.T;
  const std = isStd();
  for (let i = 0; i < n(); i++) {
    const row = state.robot.rows[i];
    const o0 = vecP(T[i]);
    const oi = vecP(T[i + 1]);
    // standard: o_{i-1} --d·z_{i-1}--> mid --a·x_i--> o_i
    // modified: o_{i-1} --a·x_{i-1}--> mid --d·z_i--> o_i
    const mid = std
      ? o0.clone().addScaledVector(vecZ(T[i]), row.d * S)
      : o0.clone().addScaledVector(vecX(T[i]), row.a * S);
    const [s0, s1] = vis.links[i];
    placeSegment(s0, o0, mid, vis.rLink);
    placeSegment(s1, mid, oi, vis.rLink);
    if (state.view.dhColors) {
      s0.material = std ? mats.linkD : mats.linkA;
      s1.material = std ? mats.linkA : mats.linkD;
    } else {
      s0.material = s1.material = mats.link;
    }

    const JF = K.jointFrame(state.robot, fk, i);
    const oj = vecP(JF);
    const z = vecZ(JF);
    placeSegment(vis.joints[i], oj.clone().addScaledVector(z, -vis.hJoint / 2), oj.clone().addScaledVector(z, vis.hJoint / 2), vis.rJoint);
    vis.joints[i].material = i === highlightedJoint ? mats.jointHi : mats.joint;

    const ext = vis.L * 0.22;
    const line = vis.jointLines[i];
    const pa = line.geometry.attributes.position;
    pa.setXYZ(0, ...oj.clone().addScaledVector(z, -ext).toArray());
    pa.setXYZ(1, ...oj.clone().addScaledVector(z, ext).toArray());
    pa.needsUpdate = true;
    line.geometry.computeBoundingSphere();
    line.computeLineDistances();
  }
  placeSegment(vis.toolSeg, vecP(T[n()]), vecP(fk.Tend), vis.rLink * 0.7);

  for (let k = 0; k <= n(); k++) vis.frames[k].matrix.copy(toMatrix4(T[k]));
  vis.toolFrame.matrix.copy(toMatrix4(fk.Tend));
  vis.gripper.matrix.copy(toMatrix4(fk.Tend));

  updateFkPanel();
  updateHud();
  syncSliders();
  saveState();
  solution.update();
}

// ------------------------------------------------------------ 목표 프레임 + 기즈모
const targetObj = new THREE.Group();
scene.add(targetObj);
let targetAxes = null;

function rebuildTargetVisual() {
  if (targetAxes) disposeGroup(targetAxes);
  targetAxes = makeAxes(vis.L * 0.11 * state.view.axisScale * 1.25, 't', { target: true });
  const ball = new THREE.Mesh(
    new THREE.SphereGeometry(vis.rLink * 0.6, 16, 12),
    new THREE.MeshBasicMaterial({ color: 0xf2b33d, depthTest: false, transparent: true, opacity: 0.8 }),
  );
  ball.renderOrder = 11;
  targetAxes.add(ball);
  targetObj.add(targetAxes);
  setGroupVisible(targetAxes, state.view.showTarget);
}

const gizmo = new TransformControls(camera, renderer.domElement);
gizmo.attach(targetObj);
gizmo.setSize(0.8);
scene.add(gizmo);
gizmo.addEventListener('dragging-changed', (e) => { orbit.enabled = !e.value; });
gizmo.addEventListener('objectChange', () => {
  targetObj.updateMatrix();
  state.targetT = fromMatrix4(targetObj.matrix);
  writeTargetInputs();
  solution.update();
  if ($('#ik-realtime').checked) {
    const res = K.inverse(state.robot, state.targetT, state.q, {
      restarts: 0, maxIter: 60, respectLimits: $('#ik-limits').checked,
    });
    state.q = res.q;
    updatePose();
    showIkResult(res, true);
  }
});

function setTargetT(T) {
  state.targetT = T;
  const m = toMatrix4(T);
  const p = new THREE.Vector3(), qt = new THREE.Quaternion(), sc = new THREE.Vector3();
  m.decompose(p, qt, sc);
  targetObj.position.copy(p);
  targetObj.quaternion.copy(qt);
  targetObj.updateMatrix();
  solution.update();
}

// ------------------------------------------------------------ 카메라
function setView(kind) {
  const L = vis.L;
  const c = new THREE.Vector3(L * 0.15, 0, L * 0.3);
  const D = L * 1.35;
  const dirs = {
    iso: new THREE.Vector3(1.0, -1.25, 0.75),
    front: new THREE.Vector3(0, -1, 0.0001),
    side: new THREE.Vector3(1, 0, 0.0001),
    top: new THREE.Vector3(0.0001, -0.0001, 1),
  };
  camera.position.copy(c).addScaledVector(dirs[kind].normalize(), D);
  orbit.target.copy(c);
  orbit.update();
}

function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h);
  labelRenderer.setSize(w, h);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(viewport);

function loop() {
  orbit.update();
  renderer.render(scene, camera);
  labelRenderer.render(scene, camera);
  requestAnimationFrame(loop);
}

// ------------------------------------------------------------ 애니메이션
let anim = null;
function animateTo(qTarget, ms = 700) {
  if (anim) cancelAnimationFrame(anim.id);
  const q0 = state.q.slice();
  const t0 = performance.now();
  const step = (t) => {
    const s = Math.min(1, (t - t0) / ms);
    const e = s < 0.5 ? 2 * s * s : 1 - (-2 * s + 2) ** 2 / 2;
    state.q = q0.map((v, i) => v + (qTarget[i] - v) * e);
    updatePose();
    if (s < 1) anim.id = requestAnimationFrame(step);
    else anim = null;
  };
  anim = { id: requestAnimationFrame(step) };
}

function setQ(q, animate = false) {
  if (animate) animateTo(q);
  else { state.q = q.slice(); updatePose(); }
}

// ============================================================ UI: 상단
function initHeader() {
  const ps = $('#preset');
  ps.innerHTML = Object.entries(K.PRESETS).map(([k, p]) => `<option value="${k}">${p.name}</option>`).join('')
    + '<option value="custom">사용자 정의</option>';
  ps.value = K.PRESETS[state.presetKey] ? state.presetKey : 'custom';
  ps.addEventListener('change', () => {
    if (ps.value === 'custom') return;
    loadPreset(ps.value);
  });

  $('#convention').value = state.robot.convention;
  $('#convention').addEventListener('change', (e) => {
    const { robot, warning } = K.convertConvention(state.robot, e.target.value);
    state.robot = robot;
    onRobotChanged({ rebuildTable: true });
    showMsg('#dh-msg', warning || `${e.target.value === 'modified' ? 'Modified (Craig)' : 'Standard'} DH 로 변환했습니다. 말단 자세는 동일하고, 중간 좌표계 위치가 달라집니다.`, warning ? 'warn' : 'ok');
  });

  const es = $('#euler');
  es.innerHTML = Object.entries(K.EULER_TYPES).map(([k, t]) => `<option value="${k}">${t.label}</option>`).join('');
  es.value = state.euler;
  es.addEventListener('change', () => {
    state.euler = es.value;
    buildTargetInputs();
    updatePose();
    if (state.quizIk) renderQuizIkTarget();
  });

  document.querySelectorAll('.tabs button').forEach((b) => b.addEventListener('click', () => {
    document.querySelectorAll('.tabs button').forEach((x) => x.classList.toggle('active', x === b));
    document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('active', t.id === 'tab-' + b.dataset.tab));
  }));

  document.querySelectorAll('#view-buttons [data-view]').forEach((b) => b.addEventListener('click', () => setView(b.dataset.view)));
}

function loadPreset(key) {
  const p = K.clonePreset(key);
  state.presetKey = key;
  state.home = p.home;
  let robot = p.robot;
  const conv = $('#convention').value;
  if (conv !== robot.convention) robot = K.convertConvention(robot, conv).robot;
  state.robot = robot;
  state.q = p.home.slice();
  $('#preset').value = key;
  onRobotChanged({ rebuildTable: true });
  setTargetT(state.fk.Tend);
  writeTargetInputs();
  setView('iso');
  showMsg('#dh-msg', '', '');
}

function onRobotChanged({ rebuildTable = false } = {}) {
  if (rebuildTable) { buildDhTable(); buildToolInputs(); }
  updateDhHeader();
  buildSliders();
  buildStepSelect();
  buildFrameToggles();
  buildQuizIkInputs();
  buildRobotVisual();
  updatePose();
}

function markCustom() {
  $('#preset').value = 'custom';
}

// ============================================================ UI: DH 테이블
function updateDhHeader() {
  const std = isStd();
  $('#legend-conv').textContent = std ? 'Standard DH' : 'Modified DH (Craig)';
  $('#dh-formula').innerHTML = std
    ? 'A<sub>i</sub> = Rot<sub>z</sub>(θ<sub>i</sub>) · Trans<sub>z</sub>(d<sub>i</sub>) · Trans<sub>x</sub>(a<sub>i</sub>) · Rot<sub>x</sub>(α<sub>i</sub>)'
    : 'A<sub>i</sub> = Rot<sub>x</sub>(α<sub>i-1</sub>) · Trans<sub>x</sub>(a<sub>i-1</sub>) · Rot<sub>z</sub>(θ<sub>i</sub>) · Trans<sub>z</sub>(d<sub>i</sub>)';
  const a = std ? 'a<sub>i</sub>' : 'a<sub>i-1</sub>';
  const al = std ? 'α<sub>i</sub>' : 'α<sub>i-1</sub>';
  $('#dh-head').innerHTML = `<tr><th>i</th><th>${a}</th><th>${al}</th><th>d<sub>i</sub></th><th>θ off</th><th class="lim">min</th><th class="lim">max</th></tr>`;
}

const DH_FIELDS = ['a', 'alpha', 'd', 'offset', 'min', 'max'];

function buildDhTable() {
  const body = $('#dh-body');
  body.innerHTML = '';
  state.robot.rows.forEach((row, i) => {
    const tr = document.createElement('tr');
    tr.innerHTML = `<td class="idx">${i + 1}</td>` + DH_FIELDS.map((f) =>
      `<td class="${f === 'min' || f === 'max' ? 'lim' : ''}"><input type="number" step="any" data-i="${i}" data-f="${f}" value="${row[f]}"></td>`,
    ).join('');
    body.appendChild(tr);
  });
  body.querySelectorAll('input').forEach((inp) => inp.addEventListener('input', () => {
    const v = parseFloat(inp.value);
    if (!Number.isFinite(v)) return;
    const i = +inp.dataset.i, f = inp.dataset.f;
    state.robot.rows[i][f] = v;
    markCustom();
    if (f === 'min' || f === 'max') buildSliders();
    buildRobotVisual();
    updatePose();
  }));
}

function buildToolInputs() {
  const wrap = $('#tool-inputs');
  const labels = { x: 'x (mm)', y: 'y (mm)', z: 'z (mm)', rx: 'Rx (°)', ry: 'Ry (°)', rz: 'Rz (°)' };
  wrap.innerHTML = Object.entries(labels).map(([k, l]) =>
    `<label>${l}<input type="number" step="any" data-k="${k}" value="${+state.robot.tool[k].toFixed(6)}"></label>`).join('');
  wrap.querySelectorAll('input').forEach((inp) => inp.addEventListener('input', () => {
    const v = parseFloat(inp.value);
    if (!Number.isFinite(v)) return;
    state.robot.tool[inp.dataset.k] = v;
    markCustom();
    buildFrameToggles();
    buildRobotVisual();
    updatePose();
  }));
}

function initDhButtons() {
  $('#btn-reset-preset').addEventListener('click', () => {
    loadPreset(K.PRESETS[state.presetKey] ? state.presetKey : 'TX200');
    showMsg('#dh-msg', '프리셋 값으로 되돌렸습니다.', 'ok');
  });
  $('#btn-export').addEventListener('click', async () => {
    const json = JSON.stringify(state.robot, null, 2);
    try {
      await navigator.clipboard.writeText(json);
      showMsg('#dh-msg', 'DH 설정 JSON 을 클립보드에 복사했습니다.', 'ok');
    } catch (_) {
      window.prompt('아래 JSON 을 복사하세요', json);
    }
  });
  $('#btn-import').addEventListener('click', () => {
    const text = window.prompt('DH 설정 JSON 을 붙여넣으세요');
    if (!text) return;
    try {
      const r = JSON.parse(text);
      if (!Array.isArray(r.rows) || !r.rows.length) throw new Error('rows 배열이 없습니다');
      r.rows = r.rows.map((row) => {
        const o = {};
        for (const f of DH_FIELDS) o[f] = Number(row[f] ?? (f === 'min' ? -180 : f === 'max' ? 180 : 0));
        return o;
      });
      r.convention = r.convention === 'modified' ? 'modified' : 'standard';
      r.tool = { x: 0, y: 0, z: 0, rx: 0, ry: 0, rz: 0, ...(r.tool || {}) };
      state.robot = r;
      state.q = r.rows.map(() => 0);
      state.home = state.q.slice();
      $('#convention').value = r.convention;
      markCustom();
      onRobotChanged({ rebuildTable: true });
      setView('iso');
      showMsg('#dh-msg', '불러왔습니다.', 'ok');
    } catch (e) {
      showMsg('#dh-msg', 'JSON 오류: ' + e.message, 'bad');
    }
  });
}

// ============================================================ UI: 정기구학
function buildSliders() {
  const wrap = $('#joint-sliders');
  wrap.innerHTML = '';
  state.robot.rows.forEach((row, i) => {
    const div = document.createElement('div');
    div.className = 'joint';
    div.innerHTML = `<span class="name">q${i + 1}</span>
      <input type="range" min="${row.min}" max="${row.max}" step="0.1" data-i="${i}">
      <input type="number" step="0.1" data-i="${i}">`;
    const [range, num] = div.querySelectorAll('input');
    const onInput = (e) => {
      const v = parseFloat(e.target.value);
      if (!Number.isFinite(v)) return;
      state.q[i] = v;
      updatePose();
    };
    range.addEventListener('input', onInput);
    num.addEventListener('input', onInput);
    div.addEventListener('mouseenter', () => { highlightedJoint = i; updatePose(); });
    div.addEventListener('mouseleave', () => { highlightedJoint = -1; updatePose(); });
    wrap.appendChild(div);
  });
  syncSliders();
}

function syncSliders() {
  document.querySelectorAll('#joint-sliders .joint').forEach((div, i) => {
    const [range, num] = div.querySelectorAll('input');
    range.value = state.q[i];
    if (document.activeElement !== num) num.value = fmt(state.q[i], 1);
  });
}

function buildStepSelect() {
  const sel = $('#step-select');
  const prev = sel.value;
  sel.innerHTML = state.robot.rows.map((_, i) => `<option value="${i}">${i + 1}</option>`).join('');
  sel.value = prev && +prev < n() ? prev : '0';
  sel.onchange = updateFkPanel;
}

function updateFkPanel() {
  const fk = state.fk;
  const { p, e } = poseText(fk.Tend);
  const names = K.EULER_TYPES[state.euler].names;
  $('#fk-pose').innerHTML =
    ['x', 'y', 'z'].map((k, i) => `<span><b>${k}</b> ${fmt(p[i], 2)}</span>`).join('') +
    names.map((k, i) => `<span><b>${k}</b> ${fmt(e[i], 3)}°</span>`).join('');
  $('#fk-matrix').innerHTML = matHTML(fk.Tend);

  const i = +$('#step-select').value || 0;
  const std = isStd();
  const row = state.robot.rows[i];
  $('#step-A-title').innerHTML = `A<sub>${i + 1}</sub> = T<sub>${i}</sub><sup>${i + 1}</sup>`;
  $('#step-T-title').innerHTML = `T<sub>0</sub><sup>${i + 1}</sup> = T<sub>0</sub><sup>${i}</sup> · A<sub>${i + 1}</sub>`;
  $('#step-A').innerHTML = matHTML(fk.A[i]);
  $('#step-T').innerHTML = matHTML(fk.T[i + 1]);
  const ai = std ? `a<sub>${i + 1}</sub>` : `a<sub>${i}</sub>`;
  const ali = std ? `α<sub>${i + 1}</sub>` : `α<sub>${i}</sub>`;
  $('#step-params').innerHTML = `${ai} = ${row.a}, ${ali} = ${row.alpha}°, d<sub>${i + 1}</sub> = ${row.d}, ` +
    `θ<sub>${i + 1}</sub> = q<sub>${i + 1}</sub> + offset = ${fmt(state.q[i], 2)} + ${row.offset} = ${fmt(state.q[i] + row.offset, 2)}°`;

  const hidden = !!state.quizFk;
  $('#fk-result').classList.toggle('hidden', hidden);
  $('#step-wrap').classList.toggle('hidden', hidden);
}

function updateHud() {
  const qs = state.q.map((v) => fmt(v, 1).padStart(6)).join(',');
  let text = `q = [${qs} ]°`;
  if (!state.quizFk) {
    const { p, e } = poseText(state.fk.Tend);
    const names = K.EULER_TYPES[state.euler].names;
    text += `\np = (${p.map((v) => fmt(v, 1)).join(', ')}) mm`;
    text += `\n${names.join('/')} = (${e.map((v) => fmt(v, 2)).join(', ')})°`;
  } else {
    text += '\n(연습문제 진행 중 — 말단 자세 숨김)';
  }
  $('#hud').textContent = text;
}

function initFkButtons() {
  $('#btn-zero').addEventListener('click', () => setQ(state.q.map(() => 0), true));
  $('#btn-home').addEventListener('click', () => setQ(state.home.length === n() ? state.home : state.q.map(() => 0), true));
  $('#btn-random').addEventListener('click', () => setQ(randomQ(), true));
  $('#btn-reveal-fk').addEventListener('click', () => endFkQuiz());
}

function randomQ(frac = 0.8, round = false) {
  return state.robot.rows.map((r) => {
    const mid = (r.min + r.max) / 2, half = ((r.max - r.min) / 2) * frac;
    const v = mid + (Math.random() * 2 - 1) * half;
    return round ? Math.round(v) : v;
  });
}

// ============================================================ UI: 역기구학
function buildTargetInputs() {
  const names = K.EULER_TYPES[state.euler].names;
  const labels = [['x', 'x (mm)'], ['y', 'y (mm)'], ['z', 'z (mm)'], ['e0', names[0] + ' (°)'], ['e1', names[1] + ' (°)'], ['e2', names[2] + ' (°)']];
  const wrap = $('#target-inputs');
  wrap.innerHTML = labels.map(([k, l]) => `<label>${l}<input type="number" step="any" data-k="${k}"></label>`).join('');
  wrap.querySelectorAll('input').forEach((inp) => inp.addEventListener('change', readTargetInputs));
  writeTargetInputs();
}

function writeTargetInputs() {
  if (!state.targetT) return;
  const { p, e } = poseText(state.targetT);
  const vals = { x: p[0], y: p[1], z: p[2], e0: e[0], e1: e[1], e2: e[2] };
  document.querySelectorAll('#target-inputs input').forEach((inp) => {
    if (document.activeElement !== inp) inp.value = fmt(vals[inp.dataset.k], inp.dataset.k[0] === 'e' ? 3 : 2);
  });
}

function readTargetInputs() {
  const v = {};
  document.querySelectorAll('#target-inputs input').forEach((inp) => { v[inp.dataset.k] = parseFloat(inp.value) || 0; });
  const R = K.eulerToRot(state.euler, [v.e0 * K.DEG, v.e1 * K.DEG, v.e2 * K.DEG]);
  setTargetT(K.fromRotPos(R, [v.x, v.y, v.z]));
}

function showIkResult(res, quick = false) {
  const el = $('#ik-result');
  const head = res.success ? '✔ 수렴' : (quick ? '… 추종 중' : '✘ 수렴 실패 (가장 가까운 해 적용)');
  el.className = 'msg ' + (res.success ? 'ok' : quick ? 'warn' : 'bad');
  el.textContent = `${head}\n반복 ${res.iterations}회 · 위치오차 ${res.posErr.toExponential(2)} mm · 자세오차 ${(res.oriErr * K.RAD).toExponential(2)}°\n` +
    `q = [${res.q.map((v) => fmt(v, 2)).join(', ')}]°`;
}

function solveIk() {
  readTargetInputs();
  const res = K.inverse(state.robot, state.targetT, state.q, { restarts: 20, respectLimits: $('#ik-limits').checked });
  showIkResult(res);
  setQ(res.q, $('#ik-animate').checked);
}

function initIk() {
  buildTargetInputs();
  $('#btn-target-from-current').addEventListener('click', () => { setTargetT(state.fk.Tend); writeTargetInputs(); });
  $('#btn-solve').addEventListener('click', solveIk);
  $('#gizmo-mode').addEventListener('change', (e) => {
    const m = e.target.value;
    gizmo.enabled = gizmo.visible = m !== 'off';
    if (m !== 'off') gizmo.setMode(m);
  });
  $('#gizmo-space').addEventListener('change', (e) => gizmo.setSpace(e.target.value));
}

// ============================================================ UI: 연습문제
function showMsg(sel, text, cls) {
  const el = $(sel);
  el.className = 'msg ' + (cls || '');
  el.textContent = text;
}

function initQuiz() {
  $('#quiz-fk-inputs').innerHTML = ['x', 'y', 'z'].map((k) => `<label>${k} (mm)<input type="number" step="any" data-k="${k}"></label>`).join('');

  $('#quiz-fk-new').addEventListener('click', () => {
    const q = randomQ(0.7, true);
    state.quizFk = { q };
    setQ(q, false);
    $('#quiz-fk-q').textContent = `q = [${q.join(', ')}]°\n→ 말단 위치 p = (x, y, z) 를 구하세요.`;
    document.querySelectorAll('#quiz-fk-inputs input').forEach((i) => { i.value = ''; });
    showMsg('#quiz-fk-msg', '', '');
  });

  $('#quiz-fk-check').addEventListener('click', () => {
    if (!state.quizFk) return showMsg('#quiz-fk-msg', '먼저 "새 문제"를 누르세요.', 'warn');
    const ans = K.pos(K.forward(state.robot, state.quizFk.q).Tend);
    const inp = [...document.querySelectorAll('#quiz-fk-inputs input')].map((i) => parseFloat(i.value));
    if (inp.some((v) => !Number.isFinite(v))) return showMsg('#quiz-fk-msg', 'x, y, z 를 모두 입력하세요.', 'warn');
    const err = Math.hypot(...inp.map((v, i) => v - ans[i]));
    const lines = ['x', 'y', 'z'].map((k, i) => `${k}: 입력 ${fmt(inp[i], 2)} / 차이 ${fmt(inp[i] - ans[i], 2)}`).join('\n');
    showMsg('#quiz-fk-msg', `${err < 1 ? '✔ 정답!' : '✘ 오차 ' + fmt(err, 2) + ' mm'}\n${lines}`, err < 1 ? 'ok' : 'bad');
  });

  $('#quiz-fk-end').addEventListener('click', endFkQuiz);

  $('#quiz-ik-new').addEventListener('click', () => {
    const q = randomQ(0.6, true);
    const T = K.forward(state.robot, q).Tend;
    state.quizIk = { q, T, revealed: false };
    setTargetT(T);
    writeTargetInputs();
    renderQuizIkTarget();
    showMsg('#quiz-ik-msg', '', '');
  });

  $('#quiz-ik-check').addEventListener('click', () => {
    if (!state.quizIk) return showMsg('#quiz-ik-msg', '먼저 "새 문제"를 누르세요.', 'warn');
    const q = [...document.querySelectorAll('#quiz-ik-inputs input')].map((i) => parseFloat(i.value));
    if (q.some((v) => !Number.isFinite(v))) return showMsg('#quiz-ik-msg', '모든 관절각을 입력하세요.', 'warn');
    const err = K.poseError(state.quizIk.T, K.forward(state.robot, q).Tend);
    setQ(q, true);
    const pass = err.posErr < 1 && err.oriErr * K.RAD < 0.1;
    showMsg('#quiz-ik-msg', `${pass ? '✔ 정답! 목표에 도달했습니다.' : '✘ 목표와 다릅니다.'}\n위치오차 ${fmt(err.posErr, 3)} mm · 자세오차 ${fmt(err.oriErr * K.RAD, 3)}°`, pass ? 'ok' : 'bad');
  });

  $('#quiz-ik-show').addEventListener('click', () => {
    if (!state.quizIk) return;
    state.quizIk.revealed = true;
    solution.update();
    const q = state.quizIk.q;
    const el = $('#quiz-ik-msg');
    el.className = 'msg warn';
    el.innerHTML = '';
    el.append(`문제 생성에 쓰인 해: q = [${q.join(', ')}]°\n(다른 해도 존재할 수 있습니다: 어깨 좌/우, 팔꿈치 위/아래, 손목 flip)\n`);
    const b = document.createElement('button');
    b.textContent = '이 해 적용';
    b.onclick = () => {
      document.querySelectorAll('#quiz-ik-inputs input').forEach((inp, i) => { inp.value = q[i]; });
      setQ(q, true);
    };
    el.append(b);
  });
}

function buildQuizIkInputs() {
  $('#quiz-ik-inputs').innerHTML = state.robot.rows.map((_, i) =>
    `<label>q${i + 1} (°)<input type="number" step="any"></label>`).join('');
}

function renderQuizIkTarget() {
  if (!state.quizIk) return;
  const { p, e } = poseText(state.quizIk.T);
  const names = K.EULER_TYPES[state.euler].names;
  $('#quiz-ik-target').innerHTML =
    `목표: p = (${p.map((v) => fmt(v, 2)).join(', ')}) mm\n` +
    `      ${names.join('/')} = (${e.map((v) => fmt(v, 3)).join(', ')})°  [${K.EULER_TYPES[state.euler].label}]` +
    matHTML(state.quizIk.T);
}

function endFkQuiz() {
  state.quizFk = null;
  updateFkPanel();
  updateHud();
  solution.update();
}

// ============================================================ UI: 표시
function buildFrameToggles() {
  const wrap = $('#frame-toggles');
  const count = n() + 1;
  if (state.view.showFrames.length !== count) state.view.showFrames = Array(count).fill(true);
  const items = state.view.showFrames.map((on, k) =>
    `<label><input type="checkbox" data-k="${k}" ${on ? 'checked' : ''}> {${k}}</label>`);
  if (!K.isIdentityTool(state.robot.tool)) items.push(`<label><input type="checkbox" data-k="tool" ${state.view.showTool ? 'checked' : ''}> {tool}</label>`);
  wrap.innerHTML = items.join('') + '<button id="frames-all">전체</button><button id="frames-none">없음</button>';
  wrap.querySelectorAll('input').forEach((inp) => inp.addEventListener('change', () => {
    if (inp.dataset.k === 'tool') state.view.showTool = inp.checked;
    else state.view.showFrames[+inp.dataset.k] = inp.checked;
    applyViewToggles();
  }));
  const setAll = (v) => {
    state.view.showFrames.fill(v);
    state.view.showTool = v;
    buildFrameToggles();
    applyViewToggles();
  };
  $('#frames-all').onclick = () => setAll(true);
  $('#frames-none').onclick = () => setAll(false);
}

function applyViewToggles() {
  if (!vis.root) return;
  vis.frames.forEach((f, k) => setGroupVisible(f, state.view.showFrames[k] !== false));
  setGroupVisible(vis.toolFrame, state.view.showTool && !K.isIdentityTool(state.robot.tool));
  vis.jointLines.forEach((l) => { l.visible = state.view.showJointAxes; });
  if (targetAxes) setGroupVisible(targetAxes, state.view.showTarget);
  if (grid) grid.visible = state.view.showGrid;
}

function initView() {
  const bind = (id, key, after) => {
    const el = $(id);
    el.checked = state.view[key];
    el.addEventListener('change', () => { state.view[key] = el.checked; applyViewToggles(); if (after) after(); });
  };
  bind('#show-labels', 'showLabels');
  bind('#show-joint-axes', 'showJointAxes');
  bind('#show-dh-segments', 'dhColors', updatePose);
  bind('#show-target', 'showTarget', () => {
    gizmo.visible = gizmo.enabled = state.view.showTarget && $('#gizmo-mode').value !== 'off';
  });
  bind('#show-grid', 'showGrid');
  $('#axis-length').addEventListener('input', (e) => {
    state.view.axisScale = parseFloat(e.target.value);
    buildRobotVisual();
    updatePose();
  });
  $('#link-opacity').value = state.view.linkOpacity;
  $('#link-opacity').addEventListener('input', (e) => {
    state.view.linkOpacity = parseFloat(e.target.value);
    applyOpacity();
  });
}

// ============================================================ 풀이 과정 패널
const solutionRoot = $('#solution');
const solution = createSolutionPanel(solutionRoot, {
  getState: () => state,
  applyQ: (q) => setQ(q, true),
  revealFk: () => endFkQuiz(),
  revealIk: () => { if (state.quizIk) state.quizIk.revealed = true; solution.update(); },
});

function initSolutionToggle() {
  const btn = $('#toggle-solution');
  const apply = (open) => {
    solutionRoot.classList.toggle('collapsed', !open);
    btn.textContent = open ? '◀ 풀이' : '▶ 풀이';
    try { localStorage.setItem('robot-kinematics-solution-open', open ? '1' : '0'); } catch (_) { /* 무시 */ }
    solution.update();
  };
  let open = true;
  try { open = localStorage.getItem('robot-kinematics-solution-open') !== '0'; } catch (_) { /* 무시 */ }
  apply(open);
  btn.addEventListener('click', () => apply(solutionRoot.classList.contains('collapsed')));
}

// ============================================================ 시작
loadState();
initSolutionToggle();
initHeader();
initDhButtons();
initFkButtons();
initView();
initQuiz();
onRobotChanged({ rebuildTable: true });
initIk();
setTargetT(state.fk.Tend);
writeTargetInputs();
resize();
setView('iso');
loop();
