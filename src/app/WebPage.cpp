// The ground-station page, embedded so the executable needs no files next to it.
#include "app/WebServer.h"

static const char* kPageA = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>tello_race ground station</title>
<style>
:root{--bg:#0f1216;--panel:#171c23;--fg:#e6e9ee;--dim:#8a93a2;--ok:#3ddc84;--warn:#ffb020;--bad:#ff4d4f;--acc:#3b9cff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.4 system-ui,Segoe UI,Roboto,sans-serif}
header{display:flex;flex-wrap:wrap;gap:12px;align-items:center;padding:10px 16px;background:var(--panel);border-bottom:1px solid #263040}
header h1{font-size:16px;margin:0 12px 0 0}.dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:var(--bad);margin-right:6px}
.dot.on{background:var(--ok)}.kv{color:var(--dim)}.kv b{color:var(--fg);font-weight:600}
main{display:grid;grid-template-columns:minmax(0,3fr) minmax(280px,1fr);gap:14px;padding:14px}
@media(max-width:900px){main{grid-template-columns:1fr}}
.video{background:#000;border-radius:8px;overflow:hidden;position:relative;min-height:240px}.video img{width:100%;display:block}
.tag{position:absolute;left:10px;top:10px;background:rgba(0,0,0,.55);padding:4px 8px;border-radius:6px;font-size:12px}
.panel{background:var(--panel);border-radius:8px;padding:12px;margin-bottom:14px}.panel h2{font-size:13px;margin:0 0 8px;color:var(--dim);text-transform:uppercase;letter-spacing:.06em}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:6px 12px}.grid div span{color:var(--dim)}
button{border:0;border-radius:6px;padding:10px 12px;font-weight:600;color:#fff;background:#2b3442;cursor:pointer;width:100%}
button:disabled{opacity:.4;cursor:not-allowed}.btns{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.start{background:var(--ok);color:#062}.land{background:var(--acc)}.emg{background:var(--bad);grid-column:1/-1;font-size:15px}
.phase{font-size:20px;font-weight:700}.ev{font-family:ui-monospace,Consolas,monospace;font-size:12px;color:var(--dim);max-height:180px;overflow:auto;white-space:pre-wrap}
select,input{width:100%;padding:8px;border-radius:6px;border:1px solid #2b3442;background:#0f1216;color:var(--fg)}
.viewer .op{display:none}.bat.low{color:var(--bad)}
</style></head><body>
<header><h1>tello_race</h1><span><span class="dot" id="dot"></span><span id="conn">connecting</span></span>
<span class="kv">drone <b id="drone">-</b></span><span class="kv">battery <b id="bat" class="bat">-</b></span>
<span class="kv">height <b id="h">-</b></span><span class="kv">temp <b id="temp">-</b></span>
<span class="kv">video <b id="fps">-</b></span><span class="kv">role <b id="role">-</b></span></header>
<main>
<section>
 <div class="video"><img id="v" alt="camera"><div class="tag" id="cam">camera</div></div>
</section>
<aside>
 <div class="panel"><h2>Mission</h2><div class="phase" id="phase">IDLE</div>
  <div class="grid"><div><span>segments</span> <b id="seg">0</b></div><div><span>time</span> <b id="t">0.0 s</b></div>
  <div><span>tag</span> <b id="tag">-</b></div><div><span>rc</span> <b id="rc">-</b></div></div></div>
 <div class="panel op"><h2>Controls</h2><div class="btns">
  <button class="start" id="bStart">START (take off)</button><button class="land" id="bLand">LAND</button>
  <button id="bCam">CAMERA front/belly</button><button id="bRec">RECORD</button>
  <button class="emg" id="bEmg">EMERGENCY STOP</button></div>
  <p class="kv">resume from checkpoint: <select id="resume"><option value="0">start line</option><option value="1">tag 1 (forward)</option>
  <option value="2">tag 2 (left)</option><option value="3">tag 3 (right)</option></select></p></div>
 <div class="panel"><h2>Events</h2><div class="ev" id="ev"></div></div>
 <div class="panel"><h2>Access</h2><input id="tok" placeholder="token (operator or viewer)"><p><button id="bTok">apply token</button></p></div>
</aside></main>
)HTML";

static const char* kPageB = R"HTML(<script>
const $=id=>document.getElementById(id);
let tok=localStorage.getItem('tello_token')||'';$('tok').value=tok;
$('bTok').onclick=()=>{tok=$('tok').value.trim();localStorage.setItem('tello_token',tok);location.reload();};
const q=tok?('?token='+encodeURIComponent(tok)):'';
let ws,role='viewer',cam='';
function video(){const img=$('v');img.src='/stream.mjpg'+(q?q+'&':'?')+'r='+Date.now();img.onerror=()=>setTimeout(video,2000);}
video();
function send(cmd,arg){if(!ws||ws.readyState!==1){alert('not connected');return;}ws.send(JSON.stringify({cmd:cmd,arg:arg||''}));}
$('bStart').onclick=()=>{if(confirm('Take off and start the mission?'))send('start');};
$('bLand').onclick=()=>send('land');
$('bEmg').onclick=()=>{if(confirm('EMERGENCY: stop the motors now?'))send('emergency');};
$('bRec').onclick=()=>send('record');
$('bCam').onclick=()=>{send('camera');setTimeout(video,1500);};
$('resume').onchange=e=>send('tag',e.target.value);
function connect(){const proto=location.protocol==='https:'?'wss://':'ws://';ws=new WebSocket(proto+location.host+'/ws'+q);
 ws.onopen=()=>{$('dot').classList.add('on');$('conn').textContent='connected';};
 ws.onclose=()=>{$('dot').classList.remove('on');$('conn').textContent='reconnecting';setTimeout(connect,2000);};
 ws.onmessage=e=>{let m;try{m=JSON.parse(e.data);}catch(x){return;}
  if(m.hello){role=m.role;$('role').textContent=role;document.body.classList.toggle('viewer',role!=='operator');return;}
  if(m.error){alert(m.error);return;}
  $('phase').textContent=m.phase;$('seg').textContent=m.seg;$('t').textContent=m.t.toFixed(1)+' s';
  $('drone').textContent=m.drone;const b=$('bat');b.textContent=m.telemetry?(m.bat+' %'):'no telemetry';b.classList.toggle('low',m.telemetry&&m.bat<30);
  $('h').textContent=m.telemetry?(m.tof+' cm'):'-';$('temp').textContent=m.telemetry?(m.templ+'-'+m.temph+' C'):'-';
  $('fps').textContent=m.fps.toFixed(1)+' fps';$('tag').textContent=m.tag>0?('ID'+m.tag+' @ '+m.tagX.toFixed(2)+' m'):'-';
  $('rc').textContent=m.rc.join(' ');$('cam').textContent=(m.camera==='belly'?'belly camera (downward)':'front camera')+(m.recording?'  REC':'');
  if(cam&&m.camera!==cam)video();cam=m.camera;
  $('ev').textContent=m.events.join('\n');$('bRec').textContent=m.recording?'STOP RECORDING':'RECORD';};}
connect();
</script></body></html>
)HTML";

const char* webPageHtml() {
    static const std::string page = std::string(kPageA) + kPageB;
    return page.c_str();
}
