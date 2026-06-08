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
const simplifyPanel = document.getElementById('simplifyPanel');
const linkList = document.getElementById('linkList');
const simplifyMsg = document.getElementById('simplifyMsg');
const revertBtn = document.getElementById('revertBtn');
const clearPreviewBtn = document.getElementById('clearPreviewBtn');
const applyAllBtn = document.getElementById('applyAllBtn');
const exportBtn = document.getElementById('exportBtn');
const importBtn = document.getElementById('importBtn');
const importFile = document.getElementById('importFile');
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
    // Track per-link objects + placement so the simplify panel can preview a
    // decimated mesh in place (hiding the originals).
    (linkObjects[entry.link] ||= []).push(obj);
    if (!linkPlacements[entry.link]) {
      linkPlacements[entry.link] = {
        position: entry.position, quaternion: entry.quaternion, scale: entry.scale,
      };
    }
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
let currentFile = null;
let linkObjects = {};      // link name -> [Object3D] currently shown
let linkPlacements = {};   // link name -> {position, quaternion, scale}
let previewObj = null;     // the decimated-preview mesh, if any
let previewState = null;   // {link, target} currently previewed

async function renderScene(rel) {
  currentFile = rel;
  currentEl.textContent = rel;
  setStatus('Loading ' + rel + ' …');
  scene.remove(robotGroup);
  robotGroup = new THREE.Group();
  scene.add(robotGroup);
  jointGroup.clear();
  // A fresh scene invalidates any preview / link tracking.
  linkObjects = {};
  linkPlacements = {};
  previewObj = null;
  previewState = null;
  activePreview = null;
  clearTimeout(previewTimer);
  previewToken++;             // invalidate any in-flight preview load
  if (clearPreviewBtn) clearPreviewBtn.style.display = 'none';

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

  loadLinks(rel);
}

// --------------------------------------------------------------------------- //
// Mesh simplification panel
// --------------------------------------------------------------------------- //
function setSimplifyMsg(msg, cls) {
  simplifyMsg.innerHTML = msg ? `<span class="${cls || ''}">${msg}</span>` : '';
}

function fmtFaces(n) {
  return n == null ? '—' : n.toLocaleString() + ' △';
}

// --------------------------------------------------------------------------- //
// Algorithm registry + per-row algorithm controls (driven by /api/algorithms)
// --------------------------------------------------------------------------- //
let algorithms = [];      // [{id,label,desc,available,requires,honors_target,params}]
let algosLoaded = null;   // cached fetch promise

function ensureAlgorithms() {
  if (!algosLoaded) {
    algosLoaded = fetch('/api/algorithms').then((r) => r.json())
      .then((d) => { algorithms = d.algorithms || []; })
      .catch(() => {
        algorithms = [{ id: 'quadric', label: 'Quadric (QEM)', desc: '',
                        available: true, honors_target: true, params: [] }];
      });
  }
  return algosLoaded;
}

// A self-contained algorithm <select> + dynamic param sliders. `onChange` fires
// when the algorithm or any parameter changes (used to drive the live preview).
function makeAlgoControl(onChange) {
  const wrap = document.createElement('div');
  wrap.className = 'algo-ctl';
  const sel = document.createElement('select');
  sel.className = 'algo-sel';
  for (const a of algorithms) {
    const opt = document.createElement('option');
    opt.value = a.id;
    opt.textContent = a.available ? a.label : `${a.label} — needs ${a.requires}`;
    opt.disabled = !a.available;
    sel.appendChild(opt);
  }
  const firstOk = algorithms.find((a) => a.available);
  if (firstOk) sel.value = firstOk.id;

  const paramsBox = document.createElement('div');
  paramsBox.className = 'algo-params';
  const store = {};   // algoId -> {paramName: value}
  const meta = () => algorithms.find((a) => a.id === sel.value) || algorithms[0];

  function renderParams() {
    paramsBox.innerHTML = '';
    const a = meta();
    if (!a) return;
    const s = (store[a.id] ||= {});
    for (const p of a.params) {
      if (s[p.name] == null) s[p.name] = p.default;
      const pr = document.createElement('div'); pr.className = 'p';

      if (p.type === 'bool') {
        const lab = document.createElement('label'); lab.className = 'bool';
        const cb = document.createElement('input'); cb.type = 'checkbox';
        cb.checked = !!s[p.name];
        const t = document.createElement('span'); t.textContent = p.label;
        cb.addEventListener('change', () => {
          s[p.name] = cb.checked;
          if (onChange) onChange();
        });
        lab.append(cb, t);
        pr.appendChild(lab);
        paramsBox.appendChild(pr);
        continue;
      }

      if (p.type === 'select') {
        const lab = document.createElement('label');
        lab.textContent = p.label;
        const selEl = document.createElement('select'); selEl.className = 'algo-sel';
        for (const o of (p.options || [])) {
          const opt = document.createElement('option');
          opt.value = o.value; opt.textContent = o.label;
          selEl.appendChild(opt);
        }
        selEl.value = s[p.name];
        selEl.addEventListener('change', () => {
          s[p.name] = selEl.value;
          if (onChange) onChange();
        });
        pr.append(lab, selEl);
        paramsBox.appendChild(pr);
        continue;
      }

      const lab = document.createElement('label');
      const t = document.createElement('span'); t.textContent = p.label;
      const v = document.createElement('span'); v.className = 'pv'; v.textContent = s[p.name];
      lab.append(t, v);
      const sl = document.createElement('input'); sl.type = 'range';
      sl.min = String(p.min); sl.max = String(p.max); sl.step = String(p.step);
      sl.value = String(s[p.name]);
      sl.addEventListener('input', () => {
        s[p.name] = parseFloat(sl.value); v.textContent = sl.value;
        if (onChange) onChange();
      });
      pr.append(lab, sl);
      paramsBox.appendChild(pr);
    }
  }
  sel.addEventListener('change', () => { renderParams(); if (onChange) onChange(); });
  renderParams();
  wrap.append(sel, paramsBox);

  return {
    el: wrap,
    getAlgo: () => sel.value,
    getParams: () => store[sel.value] || {},
    honorsTarget: () => { const a = meta(); return !!(a && a.honors_target); },
    setAlgo: (id) => {
      const opt = Array.from(sel.options).find((o) => o.value === id);
      if (!opt || opt.disabled) return false;     // unavailable here -> keep current
      sel.value = id; renderParams(); return true;
    },
    setParams: (obj) => {
      const a = meta(); if (!a) return;
      Object.assign((store[a.id] ||= {}), obj || {});
      renderParams();
    },
  };
}

// Registry of the simplify-panel rows, so presets can read/write every row's state.
let rowRegistry = [];   // [{link, target, getState, setState}]

const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

// One target row (visual or collision): its OWN algorithm + params, a % slider
// AND a target-faces field (synced), plus Preview (live) and Apply.
function targetRow(link, target, info) {
  const row = document.createElement('div');
  row.className = 'tgt' + (info ? '' : ' disabled');
  row.style.display = 'block';
  const cls = target === 'visual' ? 'vis' : 'col';

  const head = document.createElement('div');
  head.style.cssText = 'display:flex;justify-content:space-between;align-items:baseline;';
  const lbl = document.createElement('span');
  lbl.className = 'lbl ' + cls;
  lbl.textContent = target;
  head.appendChild(lbl);

  if (!info) {
    const none = document.createElement('span');
    none.className = 'faces';
    none.textContent = '(no mesh)';
    head.appendChild(none);
    row.appendChild(head);
    return row;
  }

  const orig = info.orig_faces ?? info.faces ?? 0;
  const facesEl = document.createElement('span');
  facesEl.className = 'faces';
  facesEl.textContent = 'now ' + fmtFaces(info.faces)
    + (orig !== info.faces ? ' · orig ' + fmtFaces(orig) : '')
    + (info.missing ? ' ⚠' : '');
  head.appendChild(facesEl);
  row.appendChild(head);

  // per-row algorithm + params; changing them re-runs the live preview and
  // toggles whether the face-target controls are relevant.
  const algo = makeAlgoControl(onAlgoChange);
  row.appendChild(algo.el);

  // Face-target controls (% slider + exact count). Only shown for algorithms
  // that honor a face target — hidden for "Fit box" and the clustering methods,
  // which decide the face count their own way.
  const targetWrap = document.createElement('div');

  const sl = document.createElement('div');
  sl.style.cssText = 'display:flex;align-items:center;gap:6px;margin:4px 0;';
  const slider = document.createElement('input');
  slider.type = 'range'; slider.min = '1'; slider.max = '100'; slider.step = '1'; slider.value = '30';
  const pct = document.createElement('span'); pct.className = 'pct'; pct.textContent = '30%';
  sl.append(slider, pct);
  targetWrap.appendChild(sl);

  const facesRow = document.createElement('div');
  facesRow.style.cssText = 'display:flex;align-items:center;gap:5px;margin-bottom:4px;';
  const num = document.createElement('input');
  num.type = 'number'; num.min = '4'; num.max = String(orig); num.step = '1';
  num.title = 'Target face count'; num.style.width = '70px';
  const unit = document.createElement('span'); unit.className = 'pct'; unit.textContent = '△';
  facesRow.append(num, unit);
  targetWrap.appendChild(facesRow);
  row.appendChild(targetWrap);

  // Actions — always visible.
  const actions = document.createElement('div');
  actions.style.cssText = 'display:flex;align-items:center;gap:5px;';
  const previewBtn = document.createElement('button'); previewBtn.className = 'previewBtn';
  previewBtn.textContent = '👁';
  previewBtn.title = 'Live-preview in 3D — updates as you change the slider, faces or algorithm';
  const applyBtn = document.createElement('button'); applyBtn.textContent = 'Apply';
  actions.append(previewBtn, applyBtn);
  row.appendChild(actions);

  function updateTargetVisibility() {
    targetWrap.style.display = algo.honorsTarget() ? '' : 'none';
  }
  function onAlgoChange() { updateTargetVisibility(); scheduleLive(ctx); }

  const lo = Math.min(4, orig);
  const syncFromSlider = () => {
    pct.textContent = slider.value + '%';
    num.value = String(clamp(Math.round(orig * slider.value / 100), lo, orig));
  };
  const syncFromNum = () => {
    const f = clamp(parseInt(num.value || '0', 10), lo, orig);
    num.value = String(f);
    const p = orig > 0 ? clamp(Math.round(f / orig * 100), 1, 100) : 100;
    slider.value = String(p); pct.textContent = p + '%';
  };
  const faces = () => clamp(parseInt(num.value || '0', 10), lo, orig);

  // Context handed to the preview/apply engine (everything it needs from this row).
  const ctx = { link, target, faces, getAlgo: algo.getAlgo, getParams: algo.getParams, btn: previewBtn };

  slider.addEventListener('input', () => { syncFromSlider(); scheduleLive(ctx); });
  num.addEventListener('change', () => { syncFromNum(); scheduleLive(ctx); });
  syncFromSlider();
  updateTargetVisibility();   // hide the target controls if the default algo ignores them

  if (info.missing || info.faces == null || orig <= 0) {
    previewBtn.disabled = applyBtn.disabled = true;
  }
  previewBtn.addEventListener('click', () => startPreview(ctx));
  applyBtn.addEventListener('click', () => applySimplify(ctx, applyBtn));

  // Expose this row's options so presets can save/restore them.
  rowRegistry.push({
    link, target,
    getState: () => ({ algorithm: algo.getAlgo(), params: { ...algo.getParams() }, target_faces: faces() }),
    setState: (s) => {
      if (s.algorithm) algo.setAlgo(s.algorithm);
      if (s.params) algo.setParams(s.params);
      if (s.target_faces != null) {
        num.value = String(clamp(parseInt(s.target_faces, 10) || lo, lo, orig));
        syncFromNum();
      }
      updateTargetVisibility();   // chosen algo may not use the face target
      scheduleLive(ctx);          // refresh live preview if this row is the armed one
    },
  });
  return row;
}

async function loadLinks(rel) {
  linkList.innerHTML = '';
  rowRegistry = [];
  setSimplifyMsg('');
  await ensureAlgorithms();   // rows need the algorithm list to build their selects
  let data;
  try {
    data = await (await fetch('/api/links?file=' + encodeURIComponent(rel))).json();
  } catch (e) {
    simplifyPanel.style.display = '';
    setSimplifyMsg('Failed to load links: ' + e, 'err');
    return;
  }
  simplifyPanel.style.display = '';
  if (data.error) { setSimplifyMsg(data.error, 'err'); return; }

  const withMesh = (data.links || []).filter((l) => l.visual || l.collision);
  if (!withMesh.length) {
    linkList.innerHTML = '<div class="empty">No mesh links to simplify.</div>';
    return;
  }
  for (const l of withMesh) {
    const block = document.createElement('div');
    block.className = 'lk';
    const name = document.createElement('div');
    name.className = 'lk-name';
    name.textContent = l.link;
    block.appendChild(name);
    block.appendChild(targetRow(l.link, 'visual', l.visual));
    block.appendChild(targetRow(l.link, 'collision', l.collision));
    linkList.appendChild(block);
  }
}

async function applySimplify(ctx, btn) {
  const { link, target } = ctx;
  const targetFaces = ctx.faces();
  const old = btn.textContent;
  btn.disabled = true; btn.textContent = '…';
  setSimplifyMsg(`Simplifying ${link} ${target} → ${targetFaces.toLocaleString()} △ …`);
  let res;
  try {
    res = await (await fetch('/api/simplify', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ file: currentFile, link, target, target_faces: targetFaces,
                             algorithm: ctx.getAlgo(), params: ctx.getParams() }),
    })).json();
  } catch (e) {
    setSimplifyMsg('Request failed: ' + e, 'err');
    btn.disabled = false; btn.textContent = old;
    return;
  }
  if (res.error) {
    setSimplifyMsg(res.error, 'err');
    btn.disabled = false; btn.textContent = old;
    return;
  }
  // renderScene() resets preview/link tracking, so any active preview is cleared.
  await renderScene(currentFile);
  setSimplifyMsg(`${link} ${target}: ${res.before.toLocaleString()} → `
    + `${res.after.toLocaleString()} △ (applied)`, 'ok');
}

// --------------------------------------------------------------------------- //
// Live 3D preview: one click on 👁 arms a row; then tweaking its slider, faces
// or algorithm/params refreshes the preview automatically (debounced). The
// decimated mesh stands in for the link's mesh with a wireframe overlay. Nothing
// is written to the description until you press Apply.
// --------------------------------------------------------------------------- //
let activePreview = null;   // the ctx currently armed for preview (or null)
let previewTimer = null;    // debounce handle
let previewToken = 0;       // guards against out-of-order async results

function startPreview(ctx) {
  if (activePreview && activePreview.btn) activePreview.btn.classList.remove('live');
  activePreview = ctx;
  ctx.btn.classList.add('live');
  if (clearPreviewBtn) clearPreviewBtn.style.display = '';
  runPreview(ctx);
}

function scheduleLive(ctx) {
  if (activePreview !== ctx) return;        // only the armed row auto-updates
  clearTimeout(previewTimer);
  previewTimer = setTimeout(() => runPreview(ctx), 250);
}

async function runPreview(ctx) {
  const { link, target } = ctx;
  const faces = ctx.faces();
  const token = ++previewToken;
  setSimplifyMsg(`Previewing ${link} ${target} → ${faces.toLocaleString()} △ …`);
  let res;
  try {
    res = await (await fetch('/api/preview', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ file: currentFile, link, target, target_faces: faces,
                             algorithm: ctx.getAlgo(), params: ctx.getParams() }),
    })).json();
  } catch (e) {
    if (token === previewToken) setSimplifyMsg('Preview failed: ' + e, 'err');
    return;
  }
  if (token !== previewToken) return;       // superseded by a newer tweak
  if (res.error) { setSimplifyMsg(res.error, 'err'); return; }
  showPreviewMesh(ctx, res, token);
}

function showPreviewMesh(ctx, res, token) {
  const { link, target } = ctx;
  const place = linkPlacements[link];
  const url = '/mesh?path=' + encodeURIComponent(res.mesh) + '&_=' + token;  // cache-bust
  const mat = new THREE.MeshStandardMaterial({
    color: 0xffa033, metalness: 0.0, roughness: 0.85, flatShading: true, side: THREE.DoubleSide,
  });
  const onObj = (obj) => {
    if (token !== previewToken) return;     // a newer preview already won the race
    removePreviewMesh();
    if (place) {
      obj.position.fromArray(place.position);
      obj.quaternion.set(place.quaternion[0], place.quaternion[1], place.quaternion[2], place.quaternion[3]);
      obj.scale.fromArray(place.scale);
    }
    obj.traverse((c) => {
      if (c.isMesh) {
        c.material = mat;
        c.add(new THREE.LineSegments(
          new THREE.WireframeGeometry(c.geometry),
          new THREE.LineBasicMaterial({ color: 0x222222 })));
      }
    });
    robotGroup.add(obj);
    previewObj = obj;
    previewState = { link, target };
    (linkObjects[link] || []).forEach((o) => { o.visible = false; });
    setSimplifyMsg(`Preview: ${link} ${target} ${res.before.toLocaleString()} → `
      + `${res.after.toLocaleString()} △ — not applied`, 'ok');
  };
  const fail = (e) => { if (token === previewToken) setSimplifyMsg('Preview mesh load failed: ' + e, 'err'); };
  if (res.format === 'stl') stlLoader.load(url, (geo) => onObj(new THREE.Mesh(geo, mat)), undefined, fail);
  else if (res.format === 'obj') objLoader.load(url, (grp) => onObj(grp), undefined, fail);
  else if (res.format === 'dae') daeLoader.load(url, (r) => onObj(r.scene), undefined, fail);
  else fail('unsupported format: ' + res.format);
}

function removePreviewMesh() {
  if (previewObj) { robotGroup.remove(previewObj); previewObj = null; }
  if (previewState) {
    (linkObjects[previewState.link] || []).forEach((o) => { o.visible = true; });
    previewState = null;
  }
}

function stopPreview() {
  clearTimeout(previewTimer);
  previewToken++;                            // invalidate any in-flight load
  removePreviewMesh();
  if (activePreview && activePreview.btn) activePreview.btn.classList.remove('live');
  activePreview = null;
  if (clearPreviewBtn) clearPreviewBtn.style.display = 'none';
}

if (clearPreviewBtn) {
  clearPreviewBtn.addEventListener('click', () => {
    stopPreview();
    setSimplifyMsg('Preview cleared.', 'ok');
  });
}

revertBtn.addEventListener('click', async () => {
  if (!currentFile) return;
  setSimplifyMsg('Reverting …');
  let res;
  try {
    res = await (await fetch('/api/revert', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ file: currentFile }),
    })).json();
  } catch (e) { setSimplifyMsg('Revert failed: ' + e, 'err'); return; }
  if (res.error) { setSimplifyMsg(res.error, 'err'); return; }
  await renderScene(currentFile);
  setSimplifyMsg('Reverted to original.', 'ok');
});

// --------------------------------------------------------------------------- //
// Presets: export the panel's options to a JSON file and load them back later.
// A preset stores, per link and target, the chosen algorithm + params + target
// face count. On import, rows are matched by link/target name (missing ones are
// skipped), so a preset is reusable on the same or a similar model.
// --------------------------------------------------------------------------- //
function exportPreset() {
  if (!rowRegistry.length) { setSimplifyMsg('Nothing to export — open a file first.', 'err'); return; }
  const links = {};
  for (const r of rowRegistry) (links[r.link] ||= {})[r.target] = r.getState();
  const preset = { type: 'fusion-simplify-preset', version: 1, file: currentFile, links };

  const base = (currentFile ? currentFile.split('/').pop().replace(/\.[^.]+$/, '') : 'robot');
  const blob = new Blob([JSON.stringify(preset, null, 2)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = base + '.simplify-preset.json';
  a.click();
  URL.revokeObjectURL(a.href);
  setSimplifyMsg(`Exported preset for ${Object.keys(links).length} link(s).`, 'ok');
}

function applyPreset(preset) {
  if (!preset || preset.type !== 'fusion-simplify-preset' || !preset.links) {
    setSimplifyMsg('Not a valid simplify preset file.', 'err'); return;
  }
  let n = 0;
  for (const r of rowRegistry) {
    const cfg = preset.links[r.link] && preset.links[r.link][r.target];
    if (cfg) { r.setState(cfg); n++; }
  }
  setSimplifyMsg(n ? `Preset loaded into ${n} row(s). Review, then “Apply all”.`
                   : 'Preset has no rows matching this model.', n ? 'ok' : 'err');
}

function importPreset(file) {
  const reader = new FileReader();
  reader.onload = () => {
    let preset;
    try { preset = JSON.parse(reader.result); }
    catch (e) { setSimplifyMsg('Invalid preset file: ' + e, 'err'); return; }
    applyPreset(preset);
  };
  reader.readAsText(file);
}

// Apply every row's current settings in one go (so a loaded preset is one click
// to use), then reload the scene once.
async function applyAll() {
  const jobs = rowRegistry.map((r) => ({ link: r.link, target: r.target, ...r.getState() }));
  if (!jobs.length) { setSimplifyMsg('Nothing to apply.', 'err'); return; }
  const oldLabel = applyAllBtn.textContent;
  applyAllBtn.disabled = true; applyAllBtn.textContent = '…';
  let ok = 0, fail = 0; const errs = [];
  try {
    for (let i = 0; i < jobs.length; i++) {
      const j = jobs[i];
      setSimplifyMsg(`Applying ${i + 1}/${jobs.length}: ${j.link} ${j.target} …`);
      try {
        const res = await (await fetch('/api/simplify', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ file: currentFile, link: j.link, target: j.target,
                                 target_faces: j.target_faces, algorithm: j.algorithm, params: j.params }),
        })).json();
        if (res.error) { fail++; errs.push(`${j.link}/${j.target}: ${res.error}`); } else ok++;
      } catch (e) { fail++; errs.push(`${j.link}/${j.target}: ${e}`); }
    }
    await renderScene(currentFile);   // rebuilds rows with the new meshes/counts
    setSimplifyMsg(`Applied ${ok} ok` + (fail ? `, ${fail} failed:\n` + errs.slice(0, 5).join('\n') : '.'),
                   fail ? 'err' : 'ok');
  } finally {
    applyAllBtn.disabled = false; applyAllBtn.textContent = oldLabel;
  }
}

if (exportBtn) exportBtn.addEventListener('click', exportPreset);
if (importBtn) importBtn.addEventListener('click', () => importFile.click());
if (importFile) importFile.addEventListener('change', () => {
  if (importFile.files[0]) importPreset(importFile.files[0]);
  importFile.value = '';   // allow re-importing the same file
});
if (applyAllBtn) applyAllBtn.addEventListener('click', applyAll);

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
