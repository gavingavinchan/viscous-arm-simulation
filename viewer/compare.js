import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
import {STLLoader} from './vendor/STLLoader.js';

const $ = id => document.getElementById(id);
const names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_yaw', 'wrist_roll'];
const labels = ['J1 · Pan', 'J2 · Shoulder', 'J3 · Elbow', 'J4 · Wrist flex', 'J5 · Wrist yaw', 'J6 · Wrist roll'];
const state = {leader: [0, 50, -80, 30, 10, 0], aligned: true, ready: false};
const views = [];
const child = (el, tag) => [...el.children].find(n => n.tagName === tag);
const children = (el, tag) => [...el.children].filter(n => n.tagName === tag);
const nums = (s, fallback = [0, 0, 0]) => s ? s.trim().split(/\s+/).map(Number) : fallback;
function origin(group, el) {
  const o = child(el, 'origin');
  group.position.fromArray(nums(o?.getAttribute('xyz')));
  group.quaternion.setFromEuler(new THREE.Euler(...nums(o?.getAttribute('rpy')), 'ZYX'));
}

class Model {
  constructor(id, path, color, jointNames) {
    this.id = id; this.path = path; this.jointNames = jointNames;
    this.links = new Map(); this.joints = new Map(); this.meshes = []; this.loaded = 0;
    this.scene = new THREE.Scene(); this.scene.background = new THREE.Color('#0b1119');
    this.robot = new THREE.Group(); this.scene.add(this.robot);
    this.camera = new THREE.PerspectiveCamera(38, 1, .001, 20);
    this.camera.up.set(0, 0, 1);
    this.renderer = new THREE.WebGLRenderer({antialias: true});
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.3;
    this.viewport = $(id); this.viewport.prepend(this.renderer.domElement);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 0, .18); this.controls.minDistance = .1; this.controls.maxDistance = 3;
    this.scene.add(new THREE.HemisphereLight(0xddefff, 0x354455, 2.8));
    for (const [x, y, z] of [[2, -3, 4], [-2, 1, 2]]) {
      const light = new THREE.DirectionalLight(0xffffff, 2); light.position.set(x, y, z); this.scene.add(light);
    }
    const grid = new THREE.GridHelper(1.2, 24, 0x3a5262, 0x192b3b);
    grid.rotation.x = Math.PI / 2; grid.position.z = -.001; this.scene.add(grid);
    this.bones = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color, depthTest: false}));
    this.bones.renderOrder = 5; this.scene.add(this.bones);
    this.wristAxes = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color: 0xffc375, depthTest: false}));
    this.wristAxes.renderOrder = 6; this.scene.add(this.wristAxes);
    new ResizeObserver(() => {
      this.renderer.setSize(this.viewport.clientWidth, this.viewport.clientHeight);
      this.camera.aspect = this.viewport.clientWidth / this.viewport.clientHeight; this.camera.updateProjectionMatrix();
    }).observe(this.viewport);
    this.controls.addEventListener('change', () => syncCameras(this));
    views.push(this);
  }
  async load() {
    const response = await fetch(this.path); if (!response.ok) throw new Error(`URDF: HTTP ${response.status}`);
    const url = response.url, xml = new DOMParser().parseFromString(await response.text(), 'application/xml');
    if (xml.querySelector('parsererror')) throw new Error('Invalid URDF XML');
    const root = xml.querySelector('robot'), pending = [], geometryCache = new Map(), loader = new STLLoader();
    const materials = new Map(children(root, 'material').map(m => [m.getAttribute('name'), m]));
    for (const el of children(root, 'link')) {
      const link = new THREE.Group(); link.name = el.getAttribute('name'); this.links.set(link.name, link);
      for (const visual of children(el, 'visual')) {
        pending.push((async () => {
          const geometry = child(visual, 'geometry'), meshEl = child(geometry, 'mesh');
          if (!meshEl) throw new Error('Expected mesh geometry');
          const meshURL = new URL(meshEl.getAttribute('filename'), url).href;
          if (!geometryCache.has(meshURL)) geometryCache.set(meshURL, loader.loadAsync(meshURL));
          const geo = await geometryCache.get(meshURL);
          const mat = child(visual, 'material'), colorEl = mat && child(mat, 'color') || materials.get(mat?.getAttribute('name'))?.querySelector('color');
          const rgba = nums(colorEl?.getAttribute('rgba'), [.64, .71, .77, 1]);
          const mesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({color: new THREE.Color(...rgba.slice(0, 3)), roughness: .47, metalness: .27}));
          mesh.scale.fromArray(nums(meshEl.getAttribute('scale'), [1, 1, 1]));
          const group = new THREE.Group(); origin(group, visual); group.add(mesh); link.add(group); this.meshes.push(mesh);
          mesh.visible = $('cad').checked;
          $(this.id + '-loading').textContent = `Assembling CAD · ${++this.loaded} pieces`;
        })());
      }
    }
    const childLinks = new Set();
    for (const el of children(root, 'joint')) {
      const frame = new THREE.Group(), motion = new THREE.Group(); origin(frame, el); frame.add(motion);
      const parent = child(el, 'parent').getAttribute('link'), desc = child(el, 'child').getAttribute('link');
      motion.add(this.links.get(desc)); this.links.get(parent).add(frame); childLinks.add(desc);
      this.joints.set(el.getAttribute('name'), {frame, motion, type: el.getAttribute('type'), axis: new THREE.Vector3().fromArray(nums(child(el, 'axis')?.getAttribute('xyz'), [1, 0, 0])).normalize()});
    }
    for (const [name, link] of this.links) if (!childLinks.has(name)) this.robot.add(link);
    this.pose([0, 0, 0, 0, 0, 0]); // Render continues as individual meshes arrive.
    await Promise.all(pending);
    $(this.id + '-loading').textContent = `${this.loaded} CAD pieces · no hardware connection`;
  }
  pose(degrees) {
    this.jointNames.forEach((name, i) => this.joints.get(name)?.motion.quaternion.setFromAxisAngle(this.joints.get(name).axis, THREE.MathUtils.degToRad(degrees[i])));
    this.robot.updateMatrixWorld(true);
    if (!this.joints.size) return;
    this.points = this.jointNames.map(n => this.joints.get(n).frame.getWorldPosition(new THREE.Vector3()));
    this.axes = this.jointNames.map(n => {
      const j = this.joints.get(n);
      return j.axis.clone().applyQuaternion(j.frame.getWorldQuaternion(new THREE.Quaternion()));
    });
    this.bones.geometry.dispose(); this.bones.geometry = new THREE.BufferGeometry().setFromPoints(this.points.slice(1, 4));
    const axes = [4, 5].flatMap(i => [this.points[i].clone().addScaledVector(this.axes[i], -.04), this.points[i].clone().addScaledVector(this.axes[i], .04)]);
    this.wristAxes.geometry.dispose(); this.wristAxes.geometry = new THREE.BufferGeometry().setFromPoints(axes);
  }
}

let syncing = false, report, live;
function syncCameras(source) {
  if (syncing) return; syncing = true;
  for (const other of views) if (other !== source) {
    other.camera.position.copy(source.camera.position); other.camera.quaternion.copy(source.camera.quaternion);
    other.controls.target.copy(source.controls.target); other.controls.update();
  }
  syncing = false;
}
function setView(name) {
  const direction = {iso: [1, -1.5, .8], front: [0, -1, .01], side: [1, 0, .01], top: [0, -.001, 1]}[name];
  for (const model of views) {
    model.controls.target.set(0, 0, .18);
    model.camera.position.copy(model.controls.target).add(new THREE.Vector3(...direction).normalize().multiplyScalar(.98));
    model.controls.update();
  }
}
const sliders = names.map((name, i) => {
  const row = document.createElement('div'); row.className = 'joint';
  const label = document.createElement('label'), output = document.createElement('output'), input = document.createElement('input');
  input.type = 'range'; input.min = -150; input.max = 150; input.step = .1; input.id = 'pose-' + i;
  label.htmlFor = input.id; label.append(labels[i], output); row.append(label, input); $('sliders').append(row);
  input.addEventListener('input', () => {state.leader[i] = Number(input.value); update();});
  return {input, output};
});
const leader = new Model('leader', '../leader/model/star_arm_102_hd.urdf', 0xc4a0f9, names.map((_, i) => 'joint' + (i + 1)));
const follower = new Model('follower', '../model/viscous_arm.urdf', 0x81dfcb, names);
function angle(a, b) {return THREE.MathUtils.radToDeg(a.clone().normalize().angleTo(b.clone().normalize()));}
function segment(model, a, b) {return model.points[b].clone().sub(model.points[a]);}
function update() {
  if (!state.ready) return;
  const q = state.leader;
  const target = names.map((n, i) => report.rules[n].direction * q[i] + (state.aligned ? report.rules[n].offset_deg : 0));
  leader.pose(q); follower.pose(target);
  sliders.forEach(({input, output}, i) => {input.value = q[i]; output.textContent = `${q[i].toFixed(1)}°`;});
  $('angle-rows').replaceChildren(...names.map((n, i) => {
    const rule = report.rules[n], row = document.createElement('tr');
    for (const value of [labels[i], `${q[i].toFixed(2)}°`, `${target[i].toFixed(2)}°`, `${rule.direction < 0 ? '−' : '+'}q ${state.aligned ? (rule.offset_deg < 0 ? '−' : '+') + ' ' + Math.abs(rule.offset_deg).toFixed(2) + '°' : ''}`]) {
      const cell = document.createElement('td'); cell.textContent = value; row.append(cell);
    }
    return row;
  }));
  const upper = angle(segment(leader, 1, 2), segment(follower, 1, 2));
  const forearm = angle(segment(leader, 2, 3), segment(follower, 2, 3));
  const axes = names.map((n, i) => angle(leader.axes[i], follower.axes[i].clone().multiplyScalar(report.rules[n].direction)));
  $('metrics').textContent = `Direction difference: upper arm ${upper.toFixed(3)}° · forearm ${forearm.toFixed(3)}° · largest joint-axis difference ${Math.max(...axes).toFixed(3)}°`;
  window.postureDiagnostics = () => ({leaderCAD: [...q], followerCAD: target, aligned: state.aligned,
    upperDirectionErrorDeg: upper, forearmDirectionErrorDeg: forearm, maxAxisDirectionErrorDeg: Math.max(...axes),
    leaderPieces: leader.loaded, followerPieces: follower.loaded, physicalEncoderConversion: false, hardwareConnected: false});
}
$('aligned').onchange = () => {state.aligned = $('aligned').checked; update();};
$('cad').onchange = () => {for (const view of views) for (const mesh of view.meshes) mesh.visible = $('cad').checked;};
$('sample').onclick = () => {state.leader = [0, 50, -80, 30, 10, 0]; update();};
$('zero').onclick = () => {state.leader = [0, 0, 0, 0, 0, 0]; update();};
document.querySelectorAll('[data-view]').forEach(button => {button.onclick = () => setView(button.dataset.view);});
setView('iso');
function animate() {for (const view of views) view.renderer.render(view.scene, view.camera); requestAnimationFrame(animate);}
animate();
async function start() {
  const responses = await Promise.all([fetch('../leader/inspection/posture_geometry_validation.json'), fetch('../leader/mapping/mac_waterproof_20261018.json')]);
  if (responses.some(r => !r.ok)) throw new Error('Comparison data unavailable');
  [report, live] = await Promise.all(responses.map(r => r.json()));
  $('gain-rows').replaceChildren(...names.map((n, i) => {
    const row = document.createElement('tr');
    for (const value of [labels[i], `${(live.profile.leader.joint_directions[n] * 10).toFixed(2)}°`, '±10.00°']) {
      const cell = document.createElement('td'); cell.textContent = value; row.append(cell);
    }
    return row;
  }));
  await Promise.all([leader.load(), follower.load()]); state.ready = true; update();
}
start().catch(e => {$('error').textContent = e.message; console.error(e);});
