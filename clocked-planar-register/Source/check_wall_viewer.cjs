// Offline event/matrix checks; this is not a browser render or visual approval.
const fs=require('fs'),vm=require('vm'),assert=require('assert'),crypto=require('crypto');
const html=fs.readFileSync('Wall register.html','utf8');
const data=html.match(/<script id="data"[^>]*>([\s\S]*?)<\/script>/)[1];
const source=[...html.matchAll(/<script[^>]*>([\s\S]*?)<\/script>/g)].at(-1)[1];
let drawCount=0,axisLabels=[],animation;
const noop=()=>{};const ctx=new Proxy({fillText(text,x,y){if(/^\+[XYZ]$/.test(text))axisLabels.push({text,x,y});},measureText:t=>({width:t.length*7})},{get:(o,k)=>o[k]||noop,set:(o,k,v)=>(o[k]=v,true)});
const gl=new Proxy({getShaderParameter:()=>true,getProgramParameter:()=>true,getAttribLocation:()=>0,getUniformLocation:()=>0,createShader:()=>({}),createProgram:()=>({}),createBuffer:()=>({}),drawArrays:()=>drawCount++},{get:(o,k)=>o[k]||(k===k.toUpperCase()?0:noop)});
const elements={};
for(const id of ['view','labels','axisCanvas','data','stage','rows','time','partView','labelMode','navMode','readout','front','endView','oblique','play','resetView'])elements[id]={value:({rows:'1',partView:'assembly',labelMode:'all',navMode:'rotate'})[id]||'',textContent:'',clientWidth:1000,clientHeight:580,width:1000,height:580,getContext:kind=>kind==='webgl'?gl:ctx,setPointerCapture:noop};
elements.data.textContent=data;
const sandbox={document:{querySelector:s=>elements[s.slice(1)]},window:{devicePixelRatio:1},devicePixelRatio:1,requestAnimationFrame:f=>animation=f,atob,Blob,DecompressionStream,Response,Float32Array,Uint8Array,Math,console};
vm.createContext(sandbox);vm.runInContext(source,sandbox);
(async()=>{
 for(let n=0;n<200&&!elements.partView.onchange;n++)await new Promise(r=>setTimeout(r,50));
 assert(elements.partView.onchange,'Viewer did not initialise');assert(!elements.readout.textContent.startsWith('Viewer error'),elements.readout.textContent);
 const runs=[];
 for(const width of [1000,320])for(const scene of ['assembly','frames','test'])for(const view of ['front','endView','oblique'])for(const labels of ['all','off']){
  elements.view.clientWidth=elements.labels.clientWidth=width;elements.partView.value=scene;elements.partView.onchange();elements.labelMode.value=labels;axisLabels=[];drawCount=0;elements[view].onclick();
  assert.equal(axisLabels.length,3,[width,scene,view,labels].join(' '));assert.deepEqual(axisLabels.map(x=>x.text).sort(),['+X','+Y','+Z']);
  for(const p of axisLabels)assert(p.x>=0&&p.x<=130&&p.y>=10&&p.y<=100,JSON.stringify(p));
  assert(drawCount>0);runs.push({width,scene,view,labels,drawCount});
 }
 const counts=Object.fromEntries(runs.filter(x=>x.width===1000&&x.view==='front'&&x.labels==='off').map(x=>[x.scene,x.drawCount]));assert(counts.test<counts.assembly&&counts.frames===3);assert(elements.rows.disabled);
 elements.view.onwheel({preventDefault:noop,deltaY:-100});elements.navMode.value='pan';elements.view.onpointerdown({button:0,pointerId:1,clientX:20,clientY:20});elements.view.onpointermove({pointerId:1,clientX:70,clientY:50});elements.view.onpointerup({pointerId:1});
 assert(/#orientation\{position:fixed/.test(html));assert(/X: data\/power axles/.test(html));assert(/Z: mounting rows/.test(html));assert(/\+Y: toward rear\/base/.test(html));
 const result={geometry_sha256:crypto.createHash('sha256').update(fs.readFileSync('Wall register/geometry.npz')).digest('hex'),viewer_sha256:crypto.createHash('sha256').update(html).digest('hex'),scope:'Offline JavaScript initialisation, draw filters, axes bounds and view/overlay/navigation event checks. No browser visual inspection.',pass:true,runs,part_counts:counts};fs.writeFileSync('Wall register/Viewer interaction checks.json',JSON.stringify(result,null,2));console.log('PASS',runs.length,'view/scene/label/width combinations',counts);
})().catch(e=>{console.error(e);process.exitCode=1});
