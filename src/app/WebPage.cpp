// The ground-station page, embedded so the executable needs no files next to it (script in WebPageScript.cpp).
#include "app/WebServer.h"

extern const char* kPageScript;

static const char* kPageA = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>tello_race ground station</title>
<style>
:root{--bg:#0f1216;--panel:#171c23;--fg:#e6e9ee;--dim:#8a93a2;--ok:#3ddc84;--warn:#ffb020;--bad:#ff4d4f;--acc:#3b9cff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px/1.4 system-ui,Segoe UI,Roboto,sans-serif}
header{display:flex;flex-wrap:wrap;gap:12px;align-items:center;padding:10px 16px;background:var(--panel);border-bottom:1px solid #263040}
header h1{font-size:16px;margin:0 12px 0 0}.dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:var(--bad);margin-right:6px}
.dot.on{background:var(--ok)}.kv{color:var(--dim)}.kv b{color:var(--fg);font-weight:600}
main{display:grid;grid-template-columns:minmax(0,3fr) minmax(300px,1fr);gap:14px;padding:14px}
@media(max-width:900px){main{grid-template-columns:1fr}}
.video{background:#000;border-radius:8px;overflow:hidden;position:relative;min-height:240px}.video img{width:100%;display:block;min-height:240px}
.tag{position:absolute;left:10px;top:10px;background:rgba(0,0,0,.55);padding:4px 8px;border-radius:6px;font-size:12px}
.panel{background:var(--panel);border-radius:8px;padding:12px;margin-bottom:14px}.panel h2{font-size:13px;margin:0 0 8px;color:var(--dim);text-transform:uppercase;letter-spacing:.06em}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:6px 12px}.grid div span{color:var(--dim)}
button{border:0;border-radius:6px;padding:10px 12px;font-weight:600;color:#fff;background:#2b3442;cursor:pointer;width:100%}
button:disabled{opacity:.4;cursor:not-allowed}.btns{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.start{background:var(--ok);color:#062}.land{background:var(--acc)}.emg{background:var(--bad);grid-column:1/-1;font-size:15px}
.phase{font-size:20px;font-weight:700}.ev{font-family:ui-monospace,Consolas,monospace;font-size:12px;color:var(--dim);max-height:180px;overflow:auto;white-space:pre-wrap}
select,input[type=text]{width:100%;padding:8px;border-radius:6px;border:1px solid #2b3442;background:#0f1216;color:var(--fg)}
.viewer .op{display:none}.bat.low{color:var(--bad)}
.modes{display:flex;gap:6px;margin-left:auto}.modes button{width:auto;padding:6px 14px;background:#2b3442}.modes button.sel{background:var(--acc)}
body.manual .mission-only{display:none}body.mission .manual-only{display:none}
.pads{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:10px}.pad{position:relative;aspect-ratio:1;border-radius:50%;background:#0f1216;border:2px solid #2b3442;touch-action:none;user-select:none}
.pad .knob{position:absolute;left:50%;top:50%;width:34%;height:34%;border-radius:50%;background:var(--acc);transform:translate(-50%,-50%);opacity:.85}
.pad .lbl{position:absolute;left:0;right:0;bottom:6px;text-align:center;font-size:11px;color:var(--dim)}
.hint{font-size:12px;color:var(--dim);margin:8px 0 0}
#msg{position:fixed;left:50%;bottom:24px;transform:translateX(-50%);background:var(--warn);color:#000;padding:10px 16px;border-radius:8px;font-weight:600;z-index:9;max-width:90vw;box-shadow:0 4px 16px #0008}
button.arm{background:var(--warn)!important;color:#000}input[type=range]{width:100%}
</style></head><body class="mission">
)HTML";

static const char* kPageB = R"HTML(<header><h1>tello_race</h1><span><span class="dot" id="dot"></span><span id="conn">connecting</span></span>
<span class="kv">drone <b id="drone">-</b></span><span class="kv">battery <b id="bat" class="bat">-</b></span>
<span class="kv">height <b id="h">-</b></span><span class="kv">temp <b id="temp">-</b></span>
<span class="kv">video <b id="fps">-</b></span><span class="kv">role <b id="role">-</b></span>
<span class="modes op"><button id="mMission" class="sel">MISSION</button><button id="mManual">MANUAL</button></span></header>
<main>
<section>
 <div class="video"><img id="v" alt=""><div class="tag" id="cam">camera</div></div>
</section>
<aside>
 <div class="panel"><h2>Mission</h2><div class="phase" id="phase">IDLE</div>
  <div class="grid"><div><span>segments</span> <b id="seg">0</b></div><div><span>time</span> <b id="t">0.0 s</b></div>
  <div><span>tag</span> <b id="tag">-</b></div><div><span>rc</span> <b id="rc">-</b></div></div></div>
 <div class="panel op mission-only"><h2>Mission controls</h2><div class="btns">
  <button class="start" id="bStart">START (take off)</button><button class="land" id="bLand">LAND</button>
  <button id="bCam">CAMERA front/belly</button><button id="bRec">RECORD</button>
  <button class="emg" id="bEmg">EMERGENCY STOP</button></div>
  <p class="kv">resume from checkpoint: <select id="resume"><option value="0">start line</option><option value="1">tag 1 (forward)</option>
  <option value="2">tag 2 (left)</option><option value="3">tag 3 (right)</option></select></p></div>
 <div class="panel op manual-only"><h2>Manual control</h2><div class="btns">
  <button class="start" id="bTakeoff">TAKE OFF</button><button class="land" id="bLand2">LAND</button>
  <button id="bCam2">CAMERA front/belly</button><button id="bRec2">RECORD</button>
  <button class="emg" id="bEmg2">EMERGENCY STOP</button></div>
  <p class="kv" style="margin:10px 0 2px">speed <b id="spdv">40</b></p><input type="range" id="spd" min="10" max="100" value="40">
  <div class="pads"><div class="pad" id="padL"><div class="knob"></div><div class="lbl">forward/back - left/right</div></div>
  <div class="pad" id="padR"><div class="knob"></div><div class="lbl">up/down - yaw</div></div></div>
  <p class="hint">keys: W/S forward-back, A/D left-right, R/F up-down, Q/E yaw, space = stop. Sticks release to hover if the link drops.</p></div>
 <div class="panel"><h2>Events</h2><div class="ev" id="ev"></div></div>
 <div class="panel"><h2>Access</h2><input type="text" id="tok" placeholder="token (operator or viewer)"><p><button id="bTok">apply token</button></p></div>
</aside><div id="msg" hidden></div></main>
)HTML";

const char* webPageHtml() {
    static const std::string page = std::string(kPageA) + kPageB + kPageScript;
    return page.c_str();
}
