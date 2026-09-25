let panX=0,panY=0;
const pointers=new Map();
const navMode=document.querySelector('#navMode');
function moveView(dx,dy){panX+=2*dx/canvas.clientWidth;panY-=2*dy/canvas.clientHeight;}
canvas.oncontextmenu=e=>e.preventDefault();
canvas.onpointerdown=e=>{if(e.button>2)return;pointers.set(e.pointerId,{x:e.clientX,y:e.clientY,pan:e.button!==0||e.shiftKey||navMode.value==='pan'});canvas.setPointerCapture(e.pointerId);};
canvas.onpointermove=e=>{
 const old=pointers.get(e.pointerId);if(!old)return;
 const next={...old,x:e.clientX,y:e.clientY};
 if(pointers.size===2){
  const other=[...pointers.entries()].find(([id])=>id!==e.pointerId)[1];
  moveView((next.x-old.x)/2,(next.y-old.y)/2);
  const before=Math.hypot(old.x-other.x,old.y-other.y),after=Math.hypot(next.x-other.x,next.y-other.y);
  if(before>2&&after>2)zoom=Math.max(.05,Math.min(40,zoom*after/before));
 }else if(pointers.size===1){
  if(old.pan||e.shiftKey)moveView(next.x-old.x,next.y-old.y);
  else{az+=(next.x-old.x)*.006;el+=(next.y-old.y)*.006;}
 }
 pointers.set(e.pointerId,next);draw();
};
function endPointer(e){pointers.delete(e.pointerId);}
canvas.onpointerup=endPointer;canvas.onpointercancel=endPointer;canvas.onlostpointercapture=endPointer;
canvas.onwheel=e=>{e.preventDefault();zoom=Math.max(.05,Math.min(40,zoom*Math.exp(-e.deltaY*.001)));draw();};
document.querySelector('#resetView').onclick=()=>{panX=panY=0;zoom=1;az=el=0;draw();};
