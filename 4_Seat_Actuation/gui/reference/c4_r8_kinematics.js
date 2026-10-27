/* C4 R8 geometry. Plain JavaScript: browser file:// and Node, no dependencies.
 * Angles: degrees. Lengths: mm. rx_deg is rotation about GLOBAL +X.
 * A kinematic solution does not establish mechanical clearance or load capacity.
 */
(function(root){
'use strict';
const geometry={revision:'C4_R8',units:'mm',radius_mm:24,
 centers_mm:[[-65,-36,48],[-65,-36,-48]],anchors_mm:[[-55,12,48],[-55,12,-48]],
 signs:[1,-1],yaw_limits_deg:[0,15],pitch_limits_deg:[-8,8]};
const DEG=Math.PI/180,dist=(a,b)=>Math.hypot(...a.map((x,i)=>x-b[i]));
const lengths=geometry.anchors_mm.map((a,i)=>{const c=geometry.centers_mm[i];return dist(a,[c[0],c[1],c[2]+geometry.signs[i]*24]);});
function finite(x,label){if(!Number.isFinite(x))throw Error(label+' must be finite');}
function checkPose(y,p){finite(y,'yaw');finite(p,'pitch');if(y<0-1e-9||y>15+1e-9||p< -8-1e-9||p>8+1e-9)throw Error('판 각도 범위: Yaw 0~15°, Pitch −8~8°');}
function inverse(yaw,pitch,enforce=true){
 finite(yaw,'yaw');finite(pitch,'pitch');if(enforce)checkPose(yaw,pitch);
 const cy=Math.cos(-yaw*DEG),sy=Math.sin(-yaw*DEG),cp=Math.cos(pitch*DEG),sp=Math.sin(pitch*DEG);
 const R=[[cy,-sy*cp,sy*sp],[sy,cy*cp,-cy*sp],[0,sp,cp]];
 const motors=geometry.anchors_mm.map((a,i)=>{
  const c=geometry.centers_mm[i],s=geometry.signs[i],r=geometry.radius_mm;
  const p=R.map(row=>row.reduce((v,x,j)=>v+x*a[j],0)),d=p.map((x,j)=>x-c[j]);
  const rho=Math.hypot(d[1],d[2]);if(rho<1e-10)throw Error('Singular linkage');
  const q=(d.reduce((v,x)=>v+x*x,0)+r*r-lengths[i]**2)/(2*r*rho);
  if(Math.abs(q)>1+1e-9)throw Error('Linkage cannot reach pose');
  const phi=Math.asin(Math.max(-1,Math.min(1,q)))-Math.atan2(s*d[2],d[1]);
  const ball=[c[0],c[1]+r*Math.sin(phi),c[2]+s*r*Math.cos(phi)];
  return {id:'M'+(i+1),phi_deg:phi/DEG,rx_deg:-s*phi/DEG,anchor_mm:p,ball_mm:ball,
   center_mm:c.slice(),link_length_mm:lengths[i],length_error_mm:Math.abs(dist(p,ball)-lengths[i])};
 });
 return {yaw_deg:yaw,pitch_deg:pitch,R,motors};
}
function forward(rx1,rx2,seed=[7.5,0]){
 finite(rx1,'M1 rx');finite(rx2,'M2 rx');const targets=[rx1,rx2];
 for(const start of [seed,[7.5,0],[0,0],[15,0],[7.5,7],[7.5,-7]]){
  let y=start[0],p=start[1];
  for(let n=0;n<40;n++){
   let s;try{s=inverse(y,p,false);}catch(e){break;}
   const f=s.motors.map((m,i)=>m.rx_deg-targets[i]);
   if(Math.hypot(...f)<1e-7){try{checkPose(y,p);return inverse(Math.max(0,Math.min(15,y)),Math.max(-8,Math.min(8,p)));}catch(e){break;}}
   const h=1e-4;let dy,dp;try{dy=inverse(y+h,p,false);dp=inverse(y,p+h,false);}catch(e){break;}
   const a=(dy.motors[0].rx_deg-s.motors[0].rx_deg)/h,b=(dp.motors[0].rx_deg-s.motors[0].rx_deg)/h;
   const c=(dy.motors[1].rx_deg-s.motors[1].rx_deg)/h,d=(dp.motors[1].rx_deg-s.motors[1].rx_deg)/h,det=a*d-b*c;
   if(Math.abs(det)<1e-10)break;
   const uy=(d*f[0]-b*f[1])/det,up=(-c*f[0]+a*f[1])/det;
   let accepted=false;
   for(let t=1;t>1/128;t/=2){
    const ny=y-t*uy,np=p-t*up;if(ny< -5||ny>20||np< -13||np>13)continue;
    try{const e=inverse(ny,np,false).motors.map((m,i)=>m.rx_deg-targets[i]);if(Math.hypot(...e)<Math.hypot(...f)){y=ny;p=np;accepted=true;break;}}catch(e){}
   }
   if(!accepted)break;
  }
 }
 throw Error('이 두 모터 각도로는 현재 판 각도 범위의 자세를 찾을 수 없습니다.');
}
function parseWaypoints(text){
 const rows=text.trim().split(/\r?\n/).filter(x=>x.trim()&&!x.trim().startsWith('#'));
 if(rows[0]&&/[a-z]/i.test(rows[0]))rows.shift();
 const out=rows.map((s,i)=>{const a=s.split(',').map(x=>x.trim());if(a.length!==3||a.some(x=>x===''))throw Error('경로는 t_s,yaw_deg,pitch_deg 세 열입니다.');const [t,y,p]=a.map(Number);finite(t,'time');checkPose(y,p);if(t<0)throw Error('음수 시간');return {t_s:t,yaw_deg:y,pitch_deg:p};});
 if(out.length<2||out.length>500||out[0].t_s!==0)throw Error('시간 0부터 시작하는 경로 점 2~500개가 필요합니다.');
 for(let i=1;i<out.length;i++)if(out[i].t_s<=out[i-1].t_s)throw Error('경로 시간이 증가해야 합니다.');
 return out;
}
function poseAt(w,t){
 const last=w[w.length-1];if(t>=last.t_s)return inverse(last.yaw_deg,last.pitch_deg);
 let i=0;while(i<w.length-2&&t>w[i+1].t_s)i++;
 const a=w[i],b=w[i+1],u=Math.max(0,Math.min(1,(t-a.t_s)/(b.t_s-a.t_s))),k=u*u*u*(10-15*u+6*u*u);
 return inverse(a.yaw_deg+(b.yaw_deg-a.yaw_deg)*k,a.pitch_deg+(b.pitch_deg-a.pitch_deg)*k);
}
function trajectory(w,hz=50){
 finite(hz,'sample rate');if(hz<1||hz>200)throw Error('샘플 주파수는 1~200 Hz');const duration=w[w.length-1].t_s;
 if(duration<=0||duration*hz>100000)throw Error('경로 샘플은 최대 100,000개입니다.');
 const out=[],n=Math.ceil(duration*hz);for(let i=0;i<=n;i++){const t=Math.min(i/hz,duration),s=poseAt(w,t);out.push({t_s:t,...s});}
 let peak=[0,0];for(let i=1;i<out.length;i++){const dt=out[i].t_s-out[i-1].t_s;out[i].motors.forEach((m,j)=>{const vel=(m.rx_deg-out[i-1].motors[j].rx_deg)/dt;m.rx_velocity_deg_s=vel;peak[j]=Math.max(peak[j],Math.abs(vel));});}
 return {samples:out,peak_rx_velocity_deg_s:peak};
}
const api={geometry,lengths,inverse,forward,checkPose,parseWaypoints,poseAt,trajectory};
if(typeof module!=='undefined'&&module.exports)module.exports=api;root.C4K=api;
})(typeof globalThis!=='undefined'?globalThis:this);
