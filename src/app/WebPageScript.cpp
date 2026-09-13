// Browser-side logic of the ground-station page (kept separate from the markup for readability).
extern const char* kPageScript;
const char* kPageScript = R"HTML(<script>
const $=id=>document.getElementById(id);
let tok=localStorage.getItem('tello_token')||'';const qtok=new URLSearchParams(location.search).get('token');if(qtok){tok=qtok;localStorage.setItem('tello_token',tok);history.replaceState(null,'',location.pathname);}$('tok').value=tok;
$('bTok').onclick=()=>{tok=$('tok').value.trim();localStorage.setItem('tello_token',tok);location.reload();};
const q=tok?('?token='+encodeURIComponent(tok)):'';
let ws=null,role='viewer',cam='',mode='mission',flying=false,wasStale=false;
function video(){const img=$('v');img.src='/stream.mjpg'+(q?q+'&':'?')+'r='+Date.now();img.onerror=()=>setTimeout(video,3000);}
let msgT=null;function msg(t,sticky){const e=$('msg');e.textContent=t;e.hidden=!t;clearTimeout(msgT);if(t&&!sticky)msgT=setTimeout(()=>{e.hidden=true;},5000);}
function send(cmd,arg){if(!ws||ws.readyState!==1){msg('not connected: enter the operator token under ACCESS and press apply');return false;}
 if(role!=='operator'){msg('this token only watches: enter the operator token under ACCESS to fly');return false;}
 ws.send(JSON.stringify({cmd:cmd,arg:arg||''}));return true;}
const arm={};function armed(id,label,f){const b=$(id);if(arm[id]){clearTimeout(arm[id]);arm[id]=null;b.textContent=b.dataset.l;b.classList.remove('arm');f();return;}
 if(!ws||ws.readyState!==1||role!=='operator'){send(id);return;}b.dataset.l=b.textContent;b.textContent=label;b.classList.add('arm');
 arm[id]=setTimeout(()=>{arm[id]=null;b.textContent=b.dataset.l;b.classList.remove('arm');},3000);}
function bind(id,f){const e=$(id);if(e)e.onclick=f;}
bind('bStart',()=>armed('bStart','click again to START',()=>send('start')));
bind('bLand',()=>send('land'));bind('bLand2',()=>send('land'));
bind('bEmg',()=>armed('bEmg','click again: STOP MOTORS',()=>send('emergency')));bind('bEmg2',()=>armed('bEmg2','click again: STOP MOTORS',()=>send('emergency')));
bind('bRec',()=>send('record'));bind('bRec2',()=>send('record'));
bind('bCam',()=>{send('camera');setTimeout(video,1500);});bind('bCam2',()=>{send('camera');setTimeout(video,1500);});
bind('bTakeoff',()=>armed('bTakeoff','click again to TAKE OFF',()=>send('takeoff')));
bind('mMission',()=>send('mode','mission'));bind('mManual',()=>{if(send('mode','manual'))msg('manual mode: TAKE OFF, then keys W/S A/D R/F Q/E or the pads');});
$('resume').onchange=e=>send('tag',e.target.value);
$('spd').oninput=e=>{$('spdv').textContent=e.target.value;};
/* manual sticks: keyboard + on-screen pads, sent at 10 Hz while active, one zero when released */
const key={w:0,s:0,a:0,d:0,r:0,f:0,q:0,e:0};let padL={x:0,y:0},padR={x:0,y:0},lastSent='';
function sticks(){const v=+$('spd').value;let lr=(key.d-key.a)+padL.x,fb=(key.w-key.s)-padL.y,ud=(key.r-key.f)-padR.y,yaw=(key.e-key.q)+padR.x;
 const c=x=>Math.round(Math.max(-1,Math.min(1,x))*v);return [c(lr),c(fb),c(ud),c(yaw)];}
setInterval(()=>{if(mode!=='manual'||role!=='operator')return;const s=sticks().join(',');const zero=(s==='0,0,0,0');
 if(!zero||lastSent!==s){send('rc',s);lastSent=s;}},100);
document.addEventListener('keydown',e=>{if(mode!=='manual'||e.target.tagName==='INPUT')return;const k=e.key.toLowerCase();
 if(k in key){key[k]=1;e.preventDefault();}if(e.key===' '){for(const i in key)key[i]=0;padL={x:0,y:0};padR={x:0,y:0};e.preventDefault();}});
document.addEventListener('keyup',e=>{const k=e.key.toLowerCase();if(k in key)key[k]=0;});
function pad(id,st){const el=$(id),kn=el.querySelector('.knob');let act=false;
 const upd=ev=>{const r=el.getBoundingClientRect(),cx=r.left+r.width/2,cy=r.top+r.height/2,R=r.width/2*0.65;
  let x=(ev.clientX-cx)/R,y=(ev.clientY-cy)/R;const m=Math.hypot(x,y);if(m>1){x/=m;y/=m;}st.x=x;st.y=y;
  kn.style.transform='translate(calc(-50% + '+(x*R)+'px), calc(-50% + '+(y*R)+'px))';};
 el.addEventListener('pointerdown',ev=>{act=true;el.setPointerCapture(ev.pointerId);upd(ev);});
 el.addEventListener('pointermove',ev=>{if(act)upd(ev);});
 const rel=()=>{act=false;st.x=0;st.y=0;kn.style.transform='translate(-50%,-50%)';};
 el.addEventListener('pointerup',rel);el.addEventListener('pointercancel',rel);}
pad('padL',padL);pad('padR',padR);
function setMode(m){mode=m;document.body.classList.toggle('manual',m==='manual');document.body.classList.toggle('mission',m!=='manual');
 $('mMission').classList.toggle('sel',m!=='manual');$('mManual').classList.toggle('sel',m==='manual');}
function connect(){const proto=location.protocol==='https:'?'wss://':'ws://';ws=new WebSocket(proto+location.host+'/ws'+q);
 ws.onopen=()=>{$('dot').classList.add('on');$('conn').textContent='connected';};
 ws.onclose=()=>{$('dot').classList.remove('on');$('conn').textContent='reconnecting';setTimeout(connect,2000);};
 ws.onmessage=e=>{let m;try{m=JSON.parse(e.data);}catch(x){return;}
  if(m.hello){role=m.role;$('role').textContent=role;document.body.classList.toggle('viewer',role!=='operator');return;}
  if(m.error){msg(m.error);return;}
  flying=!!m.flying;if(!arm['bStart'])$('bStart').textContent=flying?'START (from hover)':'START (take off)';if(m.mode!==mode)setMode(m.mode);
  $('phase').textContent=m.phase+(m.mode==='manual'?' (manual)':'');$('seg').textContent=m.seg;$('t').textContent=m.t.toFixed(1)+' s';
  $('drone').textContent=m.drone;const b=$('bat');b.textContent=m.telemetry?(m.bat+' %'):'no telemetry';b.classList.toggle('low',m.telemetry&&m.bat<30);
  $('h').textContent=m.telemetry?(m.tof+' cm'):'-';$('temp').textContent=m.telemetry?(m.templ+'-'+m.temph+' C'):'-';
  $('fps').textContent=m.fps.toFixed(1)+' fps';$('tag').textContent=m.tag>0?('ID'+m.tag+' @ '+m.tagX.toFixed(2)+' m'):'-';
  const mf=(m.mode==='manual'&&m.flying),st=!!(m.stale&&mf);const L=$('link');
  L.textContent=(m.wifi?m.wifi+' dBm':'-')+(mf?(st?' / STICKS RELEASED':' / input '+(m.rcAge>=0?m.rcAge.toFixed(1)+'s':'none')):'');L.classList.toggle('low',st);
  if(st&&!wasStale)msg('sticks released: no input reached the ground station for '+(m.rcAge>=0?m.rcAge.toFixed(1):'?')+' s. Weak connection between this device and the Pi?');wasStale=st;
  $('rc').textContent=m.rc.join(' ');$('cam').textContent=(m.camera==='belly'?'belly camera (downward)':'front camera')+(m.recording?'  REC':'')+(m.flying?'  FLYING':'');
  if(cam&&m.camera!==cam)video();cam=m.camera;
  $('ev').textContent=m.events.join('\n');const rt=m.recording?'STOP RECORDING':'RECORD';$('bRec').textContent=rt;$('bRec2').textContent=rt;};}
fetch('/api/status'+q).then(r=>{if(r.status===401){$('conn').textContent='token required';$('cam').textContent='no video: token required';msg('Enter the operator token in the ACCESS box (bottom right) and press apply. Viewers use the viewer token.',true);$('tok').focus();}
 else{video();connect();}}).catch(()=>{$('conn').textContent='server unreachable';setTimeout(()=>location.reload(),5000);});
</script></body></html>
)HTML";
