function drawAnnotations(f,sx,sy,ratio){
 const mode=document.querySelector('#labelMode').value;if(mode==='off')return;
 const width=labels.width,height=labels.height;
 const groups={left:[],right:[]};
 for(const a of D.annotations){
  if(mode!=='all'&&a.scope!==mode)continue;
  const [shift,axis,origin,angle]=f.joints[a.part],v=a.point.map((x,i)=>x-origin[i]);
  const dot=v.reduce((s,x,i)=>s+x*axis[i],0),cross=[axis[1]*v[2]-axis[2]*v[1],axis[2]*v[0]-axis[0]*v[2],axis[0]*v[1]-axis[1]*v[0]];
  const point=v.map((x,i)=>x*Math.cos(angle)+cross[i]*Math.sin(angle)+axis[i]*dot*(1-Math.cos(angle))+origin[i]+shift[i]);
  const x=point[0]-48,y=point[1]-18,z=point[2]-62-(rows-1)*56;
  const u=Math.cos(az)*x+Math.sin(az)*y,w=-Math.sin(az)*x+Math.cos(az)*y,zz=Math.sin(el)*w+Math.cos(el)*z;
  groups[a.side].push({...a,px:(1+u*sx)*width/2,py:(1-zz*sy)*height/2});
 }
 ctx.font=`600 ${12*ratio}px system-ui`;ctx.lineWidth=ratio;
 for(const [side,items]of Object.entries(groups)){
  items.sort((a,b)=>a.py-b.py);const gap=26*ratio,margin=16*ratio;
  for(let i=0;i<items.length;i++){
   const a=items[i];a.ly=Math.max(margin+i*gap,Math.min(height-margin-(items.length-1-i)*gap,a.py));
   if(i)a.ly=Math.max(a.ly,items[i-1].ly+gap);
   const text=a.tag+'  '+a.label,tw=ctx.measureText(text).width,bw=tw+16*ratio;
   const bx=side==='left'?10*ratio:width-10*ratio-bw,by=a.ly-10*ratio;
   ctx.strokeStyle=a.scope==='control'?'#995019':'#22666c';ctx.fillStyle='#fffdf3';
   ctx.beginPath();ctx.moveTo(a.px,a.py);ctx.lineTo(side==='left'?bx+bw:bx,a.ly);ctx.stroke();
   ctx.fillRect(bx,by,bw,21*ratio);ctx.strokeRect(bx,by,bw,21*ratio);ctx.fillStyle=ctx.strokeStyle;ctx.fillText(text,bx+8*ratio,a.ly+4*ratio);
   ctx.beginPath();ctx.arc(a.px,a.py,3*ratio,0,2*Math.PI);ctx.fill();
  }
 }
}
