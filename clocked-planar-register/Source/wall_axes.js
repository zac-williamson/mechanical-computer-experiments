// Same azimuth/elevation convention as the model vertex shader; independent
// of scene filters, annotations, zoom and pan.
function axisProjection(v,a,b){
 const x=Math.cos(a)*v[0]+Math.sin(a)*v[1];
 const y=-Math.sin(a)*v[0]+Math.cos(a)*v[1];
 return [x,Math.sin(b)*y+Math.cos(b)*v[2],Math.cos(b)*y-Math.sin(b)*v[2]];
}
function drawAxes(){
 const c=document.querySelector('#axisCanvas'),r=window.devicePixelRatio||1;
 c.width=150*r;c.height=100*r;const g=c.getContext('2d');g.scale(r,r);
 g.clearRect(0,0,150,100);g.font='bold 12px system-ui';g.lineWidth=2;
 const origin=[72,54],axes=[[[1,0,0],'+X','#b62b35'],[[0,1,0],'+Y','#268044'],[[0,0,1],'+Z','#2756b0']];
 axes.sort((u,v)=>axisProjection(u[0],az,el)[2]-axisProjection(v[0],az,el)[2]);
 for(const [v,label,color]of axes){
  const [x,z,d]=axisProjection(v,az,el),l=Math.hypot(x,z),px=origin[0]+34*x,py=origin[1]-34*z;g.strokeStyle=g.fillStyle=color;
  g.beginPath();g.moveTo(...origin);g.lineTo(px,py);g.stroke();
  if(l<.12){g.beginPath();g.arc(...origin,4,0,Math.PI*2);g.stroke();if(d<0){g.beginPath();g.arc(...origin,1.5,0,Math.PI*2);g.fill();}g.fillText(label,origin[0]+8,origin[1]+17);}
  else{const a=Math.atan2(-z,x);g.beginPath();g.moveTo(px,py);g.lineTo(px-7*Math.cos(a-.4),py-7*Math.sin(a-.4));g.lineTo(px-7*Math.cos(a+.4),py-7*Math.sin(a+.4));g.closePath();g.fill();g.fillText(label,Math.max(4,Math.min(124,px+7*x/l-9)),Math.max(13,Math.min(95,py-8*z/l+4)));}
 }
}
