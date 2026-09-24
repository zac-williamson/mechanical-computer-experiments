// Same azimuth/elevation projection as the model, fixed-size corner guide.
function drawModelAxes(){
 const r=devicePixelRatio||1,w=190*r,h=170*r,left=labels.width-w-12*r,top=12*r;
 ctx.save();ctx.fillStyle='rgba(255,255,255,.94)';ctx.fillRect(left,top,w,h);
 ctx.font=`bold ${14*r}px system-ui`;ctx.fillStyle='#243b3e';ctx.fillText('Model axes',left+12*r,top+22*r);
 const cx=left+95*r,cy=top+99*r,L=48*r;
 const basis=[['X',[1,0,0],'#d73535'],['Y',[0,1,0],'#168146'],['Z',[0,0,1],'#2469da']];
 for(const [name,v,color] of basis){
  const u=Math.cos(az)*v[0]+Math.sin(az)*v[1],depth=-Math.sin(az)*v[0]+Math.cos(az)*v[1],z=Math.sin(el)*depth+Math.cos(el)*v[2];
  ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=2*r;ctx.font=`bold ${13*r}px system-ui`;
  if(Math.hypot(u,z)<.12){
   const away=Math.cos(el)*depth-Math.sin(el)*v[2]>0;
   ctx.setLineDash([]);ctx.beginPath();ctx.arc(cx,cy,6*r,0,2*Math.PI);ctx.stroke();
   ctx.fillText(`+${name} ${away?'into':'out of'} screen`,left+12*r,top+153*r);
   if(away){ctx.beginPath();ctx.moveTo(cx-3*r,cy-3*r);ctx.lineTo(cx+3*r,cy+3*r);ctx.moveTo(cx-3*r,cy+3*r);ctx.lineTo(cx+3*r,cy-3*r);ctx.stroke();}
   else {ctx.beginPath();ctx.arc(cx,cy,2*r,0,2*Math.PI);ctx.fill();}
   continue;
  }
  for(const sign of [-1,1]){
   const dx=sign*u*L,dy=-sign*z*L,ex=cx+dx,ey=cy+dy;
   ctx.setLineDash(sign<0?[3*r,3*r]:[]);ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(ex,ey);ctx.stroke();ctx.setLineDash([]);
   if(sign>0){const a=Math.atan2(dy,dx);ctx.beginPath();ctx.moveTo(ex,ey);ctx.lineTo(ex-8*r*Math.cos(a-.4),ey-8*r*Math.sin(a-.4));ctx.lineTo(ex-8*r*Math.cos(a+.4),ey-8*r*Math.sin(a+.4));ctx.closePath();ctx.fill();}
   ctx.textAlign=dx>4*r?'left':dx<-4*r?'right':'center';ctx.fillText((sign>0?'+':'−')+name,ex+(dx>4*r?5:dx<-4*r?-5:0)*r,ey+(dy>4*r?14:-5)*r);
  }
 }
 ctx.restore();
}
