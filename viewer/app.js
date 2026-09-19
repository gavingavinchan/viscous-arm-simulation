import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
import {STLLoader} from './vendor/STLLoader.js';

const $=id=>document.getElementById(id), viewport=$('viewport');
const scene=new THREE.Scene(); scene.background=new THREE.Color('#0b1119');
const camera=new THREE.PerspectiveCamera(38,1,.001,100);camera.up.set(0,0,1);camera.position.set(.8,-1.1,.7);
const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;renderer.toneMapping=THREE.ACESFilmicToneMapping;renderer.toneMappingExposure=1.35;viewport.prepend(renderer.domElement);
const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.dampingFactor=.08;controls.minDistance=.05;controls.maxDistance=10;
scene.add(new THREE.HemisphereLight(0xc8e8ff,0x314357,2.6));
for(const [pos,intensity] of [[[2,-3,4],3], [[-3,1,2],2]]){const light=new THREE.DirectionalLight(0xf0f6ff,intensity);light.position.fromArray(pos);scene.add(light);}
const robot=new THREE.Group();scene.add(robot);
const grid=new THREE.GridHelper(2,40,0x3b5b69,0x1b2d3b);grid.rotation.x=Math.PI/2;grid.position.z=-.001;scene.add(grid);
const worldAxes=new THREE.AxesHelper(.12);worldAxes.position.set(-.3,-.3,.002);scene.add(worldAxes);
const links=new Map(),joints=new Map(),collisionMeshes=[],visualMeshes=[],jointFrames=[],sliders=[];
const loader=new STLLoader(),geometryCache=new Map();let loadCount=0,modelURL,sourceFallback=false;
const nums=(str,fallback=[0,0,0])=>str?str.trim().split(/\s+/).map(Number):fallback;
const children=(node,tag)=>Array.from(node.children).filter(n=>n.tagName===tag);
const child=(node,tag)=>children(node,tag)[0];
function origin(group,node){const o=child(node,'origin');group.position.fromArray(nums(o?.getAttribute('xyz')));const [r,p,y]=nums(o?.getAttribute('rpy'));group.quaternion.setFromEuler(new THREE.Euler(r,p,y,'ZYX'));}
function meshURL(filename){if(filename.startsWith('package://')){const rest=filename.slice(10),slash=rest.indexOf('/');return new URL(rest.slice(slash+1),modelURL).href;}return new URL(filename,modelURL).href;}
async function geometry(g){
 const m=child(g,'mesh');if(m){const url=meshURL(m.getAttribute('filename'));if(!geometryCache.has(url))geometryCache.set(url,loader.loadAsync(url));return {geo:await geometryCache.get(url),scale:nums(m.getAttribute('scale'),[1,1,1])};}
 const b=child(g,'box');if(b)return {geo:new THREE.BoxGeometry(...nums(b.getAttribute('size')))};
 const s=child(g,'sphere');if(s)return {geo:new THREE.SphereGeometry(Number(s.getAttribute('radius')),32,20)};
 const c=child(g,'cylinder');if(c){const geo=new THREE.CylinderGeometry(Number(c.getAttribute('radius')),Number(c.getAttribute('radius')),Number(c.getAttribute('length')),32);geo.rotateX(Math.PI/2);return {geo};}
 throw new Error('Unsupported URDF geometry');
}
const palette=[0x708792,0xcbd6df,0x4caaaa,0xc4d0d8,0x627786,0x76b4b5,0xc6d4dc,0x83a7b7,0xb3c4ce];
async function addShape(element,link,index,isCollision,materials){
 const {geo,scale}=await geometry(child(element,'geometry'));const group=new THREE.Group();origin(group,element);
 const mat=child(element,'material');const named=materials.get(mat?.getAttribute('name'));const colorNode=mat&&child(mat,'color')||named&&child(named,'color');const rgba=colorNode?nums(colorNode.getAttribute('rgba'),[.65,.72,.78,1]):null;
 const material=isCollision?new THREE.MeshBasicMaterial({color:0xffba63,wireframe:true,transparent:true,opacity:.42,depthWrite:false}):new THREE.MeshStandardMaterial({color:rgba?new THREE.Color(rgba[0],rgba[1],rgba[2]):palette[index%palette.length],roughness:.43,metalness:.32,transparent:rgba?.[3]<1,opacity:rgba?.[3]??1});
 const mesh=new THREE.Mesh(geo,material);mesh.userData.originalOpacity=material.opacity;if(scale)mesh.scale.fromArray(scale);mesh.name=link.name+(isCollision?' collision':' visual');group.add(mesh);link.add(group);if(isCollision){group.visible=false;collisionMeshes.push(group);}else visualMeshes.push(mesh);
 loadCount++;$('status').textContent=`Loading local geometry · ${loadCount} shapes ready`;
}
function jointValue(j,seen=new Set()){
 if(!j.mimic)return j.value;if(seen.has(j.name))throw new Error('Mimic joint cycle');seen.add(j.name);const parent=joints.get(j.mimic.name);if(!parent)throw new Error(`Missing mimic source: ${j.mimic.name}`);return jointValue(parent,seen)*j.mimic.multiplier+j.mimic.offset;
}
function applyPose(){
 for(const j of joints.values()){
  const q=jointValue(j);j.motion.position.set(0,0,0);j.motion.quaternion.identity();
  if(j.type==='revolute'||j.type==='continuous')j.motion.quaternion.setFromAxisAngle(j.axis,q);
  else if(j.type==='prismatic')j.motion.position.copy(j.axis).multiplyScalar(q);
 }
 robot.updateMatrixWorld(true);for(const s of sliders){s.input.value=s.j.value*s.factor;s.output.textContent=`${(s.j.value*s.factor).toFixed(1)}${s.unit}`;}
}
function fit(){
 robot.updateMatrixWorld(true);const box=new THREE.Box3();for(const mesh of visualMeshes)box.expandByObject(mesh);if(box.isEmpty())return;
 const center=box.getCenter(new THREE.Vector3()),size=box.getSize(new THREE.Vector3()),radius=size.length()/2;const distance=radius/Math.sin(THREE.MathUtils.degToRad(camera.fov/2))/Math.min(camera.aspect,1)*1.17;
 controls.target.copy(center);camera.position.copy(center).add(new THREE.Vector3(1,-1.4,.8).normalize().multiplyScalar(distance));camera.near=Math.max(radius/1000,.0001);camera.far=Math.max(radius*100,10);camera.updateProjectionMatrix();controls.update();
}
function makeSliders(){
 let n=0;for(const j of joints.values()){if(j.type==='fixed'||j.mimic)continue;if(!['revolute','continuous','prismatic'].includes(j.type))throw new Error(`Unsupported joint type: ${j.type}`);
 const factor=j.type==='prismatic'?1000:180/Math.PI,unit=j.type==='prismatic'?' mm':'°';const row=document.createElement('div');row.className='joint';
 const label=document.createElement('label');label.textContent=j.name.replaceAll('_',' ');label.htmlFor='joint-'+n;
 const output=document.createElement('output');output.className='value';output.htmlFor='joint-'+n;const top=document.createElement('div');top.className='joint-label';top.append(label,output);
 const input=document.createElement('input');input.type='range';input.id='joint-'+n++;input.min=j.min*factor;input.max=j.max*factor;input.step='any';input.value=j.value*factor;input.addEventListener('input',()=>{j.value=Number(input.value)/factor;applyPose();});
 const limits=document.createElement('div');limits.className='limits';for(const value of [j.min,j.max]){const span=document.createElement('span');span.textContent=(value*factor).toFixed(0)+unit;limits.append(span);}row.append(top,input,limits);$('joints').append(row);sliders.push({j,input,output,factor,unit});
 }
}
async function load(){
 let response=await fetch('../model/viscous_arm.urdf',{cache:'no-store'});if(!response.ok){if(response.status!==404)throw new Error(`URDF HTTP ${response.status}`);sourceFallback=true;response=await fetch('../reference/maker-urdf/robot.urdf',{cache:'no-store'});}if(!response.ok)throw new Error(`URDF HTTP ${response.status}`);modelURL=response.url;
 const doc=new DOMParser().parseFromString(await response.text(),'application/xml');if(doc.querySelector('parsererror'))throw new Error('Invalid URDF XML');const root=doc.querySelector('robot');if(!root)throw new Error('URDF has no robot');
 const materials=new Map(children(root,'material').map(m=>[m.getAttribute('name'),m])),pending=[];let index=0;
 for(const el of children(root,'link')){const link=new THREE.Group();link.name=el.getAttribute('name');links.set(link.name,link);for(const visual of children(el,'visual'))pending.push(addShape(visual,link,index,false,materials));for(const collision of children(el,'collision'))pending.push(addShape(collision,link,index,true,materials));index++;}
 const childNames=new Set();for(const el of children(root,'joint')){
  const name=el.getAttribute('name'),type=el.getAttribute('type'),parentName=child(el,'parent').getAttribute('link'),childName=child(el,'child').getAttribute('link');const parent=links.get(parentName),descendant=links.get(childName);if(!parent||!descendant)throw new Error(`Missing link in joint ${name}`);
  const frame=new THREE.Group(),motion=new THREE.Group();frame.name=name;origin(frame,el);frame.add(motion);motion.add(descendant);parent.add(frame);childNames.add(childName);
  const axis=new THREE.Vector3().fromArray(nums(child(el,'axis')?.getAttribute('xyz'),[1,0,0])).normalize(),limit=child(el,'limit'),mimicEl=child(el,'mimic');const j={name,type,frame,motion,axis,value:0,min:type==='continuous'?-Math.PI:Number(limit?.getAttribute('lower')??-Math.PI),max:type==='continuous'?Math.PI:Number(limit?.getAttribute('upper')??Math.PI)};
  if(mimicEl)j.mimic={name:mimicEl.getAttribute('joint'),multiplier:Number(mimicEl.getAttribute('multiplier')??1),offset:Number(mimicEl.getAttribute('offset')??0)};joints.set(name,j);
  const axes=new THREE.AxesHelper(.035);axes.visible=false;motion.add(axes);jointFrames.push(axes);
 }
 for(const [name,link] of links)if(!childNames.has(name))robot.add(link);
 await Promise.all(pending);makeSliders();applyPose();fit();
 const active=sliders.length,mimics=[...joints.values()].filter(j=>j.mimic).length;
 $('status').textContent=sourceFallback?'Reference model loaded · regenerated model is not yet available. Reload after export.':`${links.size} links · ${active} independent joints · ${mimics} mimic joint${mimics===1?'':'s'} · CAD zero`;
 $('meta').textContent=`${root.getAttribute('name')} · ${sourceFallback?'reference/maker-urdf/robot.urdf':'model/viscous_arm.urdf'}\nThree.js r160 · local assets`;
 for(const id of ['zero','sample','frame'])$(id).disabled=false;
 // Deliberately expose read-only numeric diagnostics for browser QA; no hardware interfaces.
 window.modelDiagnostics=()=>({source:modelURL,links:links.size,joints:[...joints.values()].map(j=>({name:j.name,type:j.type,value:jointValue(j),mimic:j.mimic??null})),visualMeshes:visualMeshes.length,collisionMeshes:collisionMeshes.length,collisionVisible:$('collision').checked,zeroConvention:'CAD zero; encoder offsets unverified'});
}
$('zero').onclick=()=>{for(const j of joints.values())j.value=0;applyPose();$('status').textContent='CAD zero · encoder offsets unverified';};
$('sample').onclick=()=>{let i=0;const degrees=[25,-32,48,22,-28,15];for(const j of joints.values()){if(j.mimic||j.type==='fixed')continue;const value=j.type==='prismatic'?(j.min+j.max)*.5:THREE.MathUtils.degToRad(degrees[i++%degrees.length]);j.value=THREE.MathUtils.clamp(value,j.min,j.max);}applyPose();fit();$('status').textContent='Illustrative sample pose · no collision or reachability validation';};
$('frame').onclick=fit;$('collision').onchange=()=>{for(const obj of collisionMeshes)obj.visible=$('collision').checked;for(const mesh of visualMeshes){mesh.material.opacity=$('collision').checked?.24:mesh.userData.originalOpacity;mesh.material.transparent=mesh.material.opacity<1;}};
$('axes').onchange=()=>{for(const axes of jointFrames)axes.visible=$('axes').checked;};
new ResizeObserver(()=>{const width=viewport.clientWidth,height=viewport.clientHeight;renderer.setSize(width,height);camera.aspect=width/height;camera.updateProjectionMatrix();}).observe(viewport);
renderer.setAnimationLoop(()=>{controls.update();renderer.render(scene,camera);});
load().catch(error=>{$('status').textContent=`Model could not load: ${error.message}`;$('status').style.borderColor='#e77b77';console.error(error);});
