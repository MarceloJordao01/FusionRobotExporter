import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';

// Robotics convention: Z up.
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const viewport = document.getElementById('viewport');
const statusEl = document.getElementById('status');
const currentEl = document.getElementById('current');
const showJointsEl = document.getElementById('showJoints');
const showNamesEl = document.getElementById('showNames');
const fontSizeEl = document.getElementById('fontSize');
const axisSizeEl = document.getElementById('axisSize');
const fontValEl = document.getElementById('fontVal');
const axisValEl = document.getElementById('axisVal');
const openBtn = document.getElementById('openBtn');
const modal = document.getElementById('modal');
const dlgPath = document.getElementById('dlgPath');
const dlgList = document.getElementById('dlgList');
const dlgClose = document.getElementById('dlgClose');

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1e1e22);

const camera = new THREE.PerspectiveCamera(50, 1, 0.001, 1000);
camera.position.set(0.6, -0.6, 0.4);

const renderer = new THREE.WebGLRenderer({ antialias: true });
viewport.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xffffff, 0x444455, 1.1));
const dir = new THREE.DirectionalLight(0xffffff, 1.2);
dir.position.set(1, -1, 2);
scene.add(dir);

const grid = new THREE.GridHelper(2, 20, 0x555555, 0x333333);
grid.rotation.x = Math.PI / 2; // grid in XY plane (Z up)
scene.add(grid);
scene.add(new THREE.AxesHelper(0.15));

let robotGroup = new THREE.Group();
let jointGroup = new THREE.Group();
scene.add(robotGroup);
scene.add(jointGroup);

const stlLoader = new STLLoader();
const objLoader = new OBJLoader();
const daeLoader = new ColladaLoader();

function setStatus(msg, cls) {
  statusEl.innerHTML = `<span class="${cls || ''}">${msg}</span>`;
}

function resize() {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();

// --------------------------------------------------------------------------- //
// Mesh loading
// --------------------------------------------------------------------------- //
function meshUrl(rel) { return '/mesh?path=' + encodeURIComponent(rel); }

function loadOne(entry) {
  const url = meshUrl(entry.mesh);
  const material = new THREE.MeshStandardMaterial({
    color: new THREE.Color(entry.color[0], entry.color[1], entry.color[2]),
    opacity: entry.color[3] ?? 1.0,
    transparent: (entry.color[3] ?? 1.0) < 1.0,
    metalness: 0.1, roughness: 0.7, side: THREE.DoubleSide,
  });

  const place = (obj) => {
    obj.position.fromArray(entry.position);
    obj.quaternion.set(entry.quaternion[0], entry.quaternion[1], entry.quaternion[2], entry.quaternion[3]);
    obj.scale.fromArray(entry.scale);
    robotGroup.add(obj);
  };

  return new Promise((resolve) => {
    const fail = (e) => { console.warn('load failed', entry.mesh, e); resolve(false); };
    if (entry.format === 'stl') {
      stlLoader.load(url, (geo) => { place(new THREE.Mesh(geo, material)); resolve(true); }, undefined, fail);
    } else if (entry.format === 'obj') {
      objLoader.load(url, (grp) => {
        grp.traverse((c) => { if (c.isMesh) c.material = material; });
        place(grp); resolve(true);
      }, undefined, fail);
    } else if (entry.format === 'dae') {
      daeLoader.load(url, (res) => { place(res.scene); resolve(true); }, undefined, fail);
    } else {
      fail('unsupported format: ' + entry.format);
    }
  });
}

// --------------------------------------------------------------------------- //
// Joint visualization
// --------------------------------------------------------------------------- //
function makeLabel(text) {
  const pad = 12, fontPx = 48;
  const canvas = document.createElement('canvas');
  const ctx = canvas.getContext('2d');
  ctx.font = `${fontPx}px sans-serif`;
  const tw = ctx.measureText(text).width;
  canvas.width = Math.ceil(tw + pad * 2);
  canvas.height = fontPx + pad * 2;
  ctx.font = `${fontPx}px sans-serif`;
  ctx.fillStyle = 'rgba(20,20,24,0.78)';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#ffe08a';
  ctx.textBaseline = 'middle';
  ctx.fillText(text, pad, canvas.height / 2);

  const tex = new THREE.CanvasTexture(canvas);
  tex.minFilter = THREE.LinearFilter;
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  sprite.scale.set(canvas.width / canvas.height, 1, 1); // unit height, caller rescales
  sprite.userData.aspect = canvas.width / canvas.height;
  return sprite;
}

// RViz-style RGB axis triad (X=red, Y=green, Z=blue) at the joint frame.
function addTriad(group, position, quaternion, length) {
  const q = new THREE.Quaternion().fromArray(quaternion);
  const o = new THREE.Vector3().fromArray(position);
  const head = length * 0.22, headW = length * 0.11;
  const axes = [
    [new THREE.Vector3(1, 0, 0), 0xff3b54],  // X red
    [new THREE.Vector3(0, 1, 0), 0x4cd964],  // Y green
    [new THREE.Vector3(0, 0, 1), 0x3b9dff],  // Z blue
  ];
  for (const [v, color] of axes) {
    const dir = v.clone().applyQuaternion(q).normalize();
    group.add(new THREE.ArrowHelper(dir, o, length, color, head, headW));
  }
}

let lastJoints = [];
let lastRadius = 0.2;

function buildJoints(joints, radius) {
  jointGroup.clear();
  lastJoints = joints || [];
  lastRadius = radius > 0 ? radius : 0.2;
  if (!lastJoints.length) return;

  const fontMul = parseFloat(fontSizeEl.value);
  const axisMul = parseFloat(axisSizeEl.value);
  const r = lastRadius;
  const triadLen = r * 0.4 * axisMul;
  const labelH = r * 0.13 * fontMul;

  for (const j of lastJoints) {
    const quat = j.quaternion || [0, 0, 0, 1];
    addTriad(jointGroup, j.position, quat, triadLen);

    if (showNamesEl.checked) {
      const label = makeLabel(j.name);
      label.position.fromArray(j.position);
      label.position.z += triadLen * 0.15 + labelH * 0.6;
      label.scale.set(label.userData.aspect * labelH, labelH, 1);
      label.renderOrder = 1000;
      jointGroup.add(label);
    }
  }
  jointGroup.visible = showJointsEl.checked;
}

function rebuildJoints() { buildJoints(lastJoints, lastRadius); }

showJointsEl.addEventListener('change', () => { jointGroup.visible = showJointsEl.checked; });
showNamesEl.addEventListener('change', rebuildJoints);
fontSizeEl.addEventListener('input', () => {
  fontValEl.textContent = parseFloat(fontSizeEl.value).toFixed(1) + '×';
  rebuildJoints();
});
axisSizeEl.addEventListener('input', () => {
  axisValEl.textContent = parseFloat(axisSizeEl.value).toFixed(1) + '×';
  rebuildJoints();
});

// --------------------------------------------------------------------------- //
// Camera fit
// --------------------------------------------------------------------------- //
function fitCamera() {
  const box = new THREE.Box3().setFromObject(robotGroup);
  if (box.isEmpty()) return 0.2;
  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());
  const radius = Math.max(size.x, size.y, size.z) * 0.5 || 0.2;
  controls.target.copy(center);
  const dist = radius / Math.tan((camera.fov * Math.PI) / 360) * 1.6;
  camera.position.copy(center).add(new THREE.Vector3(1, -1, 0.8).normalize().multiplyScalar(dist));
  camera.near = Math.max(dist / 1000, 0.0001);
  camera.far = dist * 1000;
  camera.updateProjectionMatrix();
  grid.scale.setScalar(Math.max(radius * 4, 0.2) / 2);
  return radius;
}

// --------------------------------------------------------------------------- //
// Render a scene
// --------------------------------------------------------------------------- //
async function renderScene(rel) {
  currentEl.textContent = rel;
  setStatus('Loading ' + rel + ' …');
  scene.remove(robotGroup);
  robotGroup = new THREE.Group();
  scene.add(robotGroup);
  jointGroup.clear();

  let data;
  try {
    data = await (await fetch('/api/scene?file=' + encodeURIComponent(rel))).json();
  } catch (e) {
    setStatus('Failed to fetch scene: ' + e, 'err');
    return;
  }
  if (data.error) { setStatus('Parse error:\n' + data.error, 'err'); return; }
  if (!data.entries.length) { setStatus('No visual meshes found in this file.', 'err'); return; }

  const results = await Promise.all(data.entries.map(loadOne));
  const radius = fitCamera();
  buildJoints(data.joints, radius);

  const ok = results.filter(Boolean).length;
  const fail = results.length - ok;
  const njoints = (data.joints || []).length;
  setStatus(`${data.name} (${data.format.toUpperCase()})\n`
    + `loaded ${ok}/${results.length} meshes, ${njoints} joint(s)`
    + (fail ? `\n${fail} mesh(es) failed (see console)` : ''), fail ? 'err' : 'ok');
}

// --------------------------------------------------------------------------- //
// Modal file browser
// --------------------------------------------------------------------------- //
function addRow(label, cls, onClick) {
  const row = document.createElement('div');
  row.className = 'row ' + cls;
  row.textContent = label;
  row.addEventListener('click', onClick);
  dlgList.appendChild(row);
}

async function browse(dir) {
  let data;
  try {
    data = await (await fetch('/api/browse?dir=' + encodeURIComponent(dir))).json();
  } catch (e) {
    dlgList.innerHTML = '<div class="empty">Failed to browse: ' + e + '</div>';
    return;
  }
  dlgPath.textContent = '/' + (data.dir || '');
  dlgList.innerHTML = '';

  if (data.dir) addRow('📁 ..', 'dir', () => browse(data.parent || ''));
  for (const d of data.dirs) addRow('📁 ' + d.name, 'dir', () => browse(d.rel));
  for (const f of data.files) {
    addRow('📄 ' + f.name, 'file', () => { closeModal(); renderScene(f.rel); });
  }
  if (!data.dirs.length && !data.files.length) {
    dlgList.innerHTML = '<div class="empty">(no folders or .urdf/.xacro/.sdf here)</div>';
  }
}

function openModal() { modal.classList.remove('hidden'); browse(''); }
function closeModal() { modal.classList.add('hidden'); }

openBtn.addEventListener('click', openModal);
dlgClose.addEventListener('click', closeModal);
modal.addEventListener('click', (e) => { if (e.target === modal) closeModal(); });
window.addEventListener('keydown', (e) => { if (e.key === 'Escape') closeModal(); });

// Open the picker on first load.
openModal();
