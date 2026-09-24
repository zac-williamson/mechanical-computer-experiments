{const placed=[];for(const [name,point] of D.ports){
 const [x,y,z]=[point[0]-48,point[1]-18,point[2]-62];
 const u=Math.cos(az)*x+Math.sin(az)*y,w=-Math.sin(az)*x+Math.cos(az)*y,zz=Math.sin(el)*w+Math.cos(el)*z;
 const px=(1+u*sx)*labels.width/2,py=(1-zz*sy)*labels.height/2,tw=ctx.measureText(name).width,bw=tw+12*ratio,bh=23*ratio;
 const bx=Math.max(4*ratio,Math.min(labels.width-bw-4*ratio,px+6*ratio));let by=py-22*ratio;
 for(const offset of [0,-28,28,-56,56,-84,84,-112,112]){
  const candidate=Math.max(4*ratio,Math.min(labels.height-bh-4*ratio,py+(offset-22)*ratio));
  if(!placed.some(r=>bx<r[0]+r[2]+3*ratio&&bx+bw+3*ratio>r[0]&&candidate<r[1]+r[3]+3*ratio&&candidate+bh+3*ratio>r[1])){by=candidate;break;}
 }
 placed.push([bx,by,bw,bh]);ctx.strokeStyle='#0b6975';ctx.lineWidth=ratio;ctx.beginPath();ctx.moveTo(px,py);ctx.lineTo(Math.max(bx,Math.min(bx+bw,px)),Math.max(by,Math.min(by+bh,py)));ctx.stroke();
 ctx.fillStyle='#0b6975';ctx.beginPath();ctx.arc(px,py,4*ratio,0,Math.PI*2);ctx.fill();ctx.fillStyle='#fff';ctx.fillRect(bx,by,bw,bh);ctx.fillStyle='#164b54';ctx.fillText(name,bx+6*ratio,by+16*ratio);
}}
drawModelAxes();
