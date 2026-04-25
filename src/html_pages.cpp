#include <Arduino.h>
#include "html_pages.h"

// ============================================================================
// UPDATE_HTML
// Standalone OTA upload page. Kept reachable via GET /update for direct
// firmware flashing without going through the main tab UI.
// ============================================================================
const char UPDATE_HTML[] PROGMEM = R"HTML(

<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>Firmware Update</title>
  <style>
    body{font-family:system-ui,Segoe UI,Roboto,Arial;max-width:820px;margin:18px auto;padding:0 12px}
    .card{border:1px solid #ddd;border-radius:12px;padding:14px}
    h1{font-size:18px;margin:0 0 10px}
    .small{font-size:12px;color:#666}
    input,button{font-size:14px;padding:10px;border-radius:10px;border:1px solid #bbb}
    button{cursor:pointer}
    .row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
    .mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:12px}
    .bar{width:100%;height:18px}
    .ok{color:#0a7} .warn{color:#c70} .bad{color:#c00}
  </style>
</head>
<body>
  <div class="card">
    <h1>ESP32 Firmware Update</h1>
    <div class="small">Current build: <span id="ver" class="mono">loading...</span></div>
    <div class="small" style="margin-top:8px">
      Select a <b>.bin</b> built for this board, then upload.
    </div>
    <div class="row" style="margin-top:12px">
      <input id="file" type="file" accept=".bin" required/>
      <button id="btnUp" onclick="startUpload()">UPLOAD</button>
      <button onclick="location.href='/'">BACK</button>
    </div>
    <div style="margin-top:12px">
      <progress id="prog" class="bar" value="0" max="100"></progress>
      <div id="ptext" class="small mono">0%</div>
    </div>
    <div id="msg" class="small">Ready.</div>
  </div>
<script>
async function loadVersion(){
  try{ const r=await fetch('/api/version',{cache:'no-store'}); const j=await r.json();
       document.getElementById('ver').textContent=`${j.version} (${j.hash}) built: ${j.built}`;}
  catch(e){ document.getElementById('ver').textContent='unknown';}
}
function setMsg(t,c){const e=document.getElementById('msg');e.className='small '+(c||'');e.textContent=t;}
function setProgress(p){document.getElementById('prog').value=p;document.getElementById('ptext').textContent=p.toFixed(0)+'%';}
function startUpload(){
  const f=document.getElementById('file').files[0];
  if(!f){alert('Select a .bin file');return;}
  document.getElementById('btnUp').disabled=true;
  document.getElementById('file').disabled=true;
  setProgress(0); setMsg('Uploading...','warn');
  const form=new FormData(); form.append('update',f,f.name);
  const xhr=new XMLHttpRequest(); xhr.open('POST','/update',true);
  xhr.upload.onprogress=e=>{if(e.lengthComputable)setProgress((e.loaded/e.total)*100);};
  xhr.onload=()=>{
    if(xhr.status===200){setProgress(100);setMsg((xhr.responseText||'OK')+' (page will disconnect)','ok');}
    else{setMsg('Upload failed: HTTP '+xhr.status+' '+(xhr.responseText||''),'bad');
      document.getElementById('btnUp').disabled=false;document.getElementById('file').disabled=false;}
  };
  xhr.onerror=()=>{setMsg('Upload error (network).','bad');
    document.getElementById('btnUp').disabled=false;document.getElementById('file').disabled=false;};
  xhr.send(form);
}
loadVersion();
</script>
</body>
</html>
)HTML";


// ============================================================================
// ROOT_HTML
// Single-page tabbed UI served at "/". Four tabs:
//   1. Live    (default) -- compact RMS readout + Hz/G adjustment
//   2. Grab    -- metadata-driven motion-triggered recording
//   3. Preview -- detailed live view, file analysis, FFT, calibration
//   4. Update  -- OTA firmware upload (inline form; mirrors /update)
//
// IDs are namespaced per tab (live_ / grab_ / prev_ / up_) to avoid clashes.
// Polling is unified through refreshInfo() so /api/info drives every tab's
// status indicators in one round trip.
// ============================================================================
const char ROOT_HTML[] PROGMEM = R"HTML(

<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>accMeter2</title>
  <style>
    *{box-sizing:border-box}
    body{font-family:system-ui,Segoe UI,Roboto,Arial;margin:0;padding:0;color:#111;background:#f5f5f7}
    header{position:sticky;top:0;z-index:10;background:#fff;border-bottom:1px solid #ddd;padding:8px 12px 0}
    header h1{font-size:16px;margin:0 0 6px;display:flex;align-items:center;gap:8px}
    header .ver{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:11px;color:#888;font-weight:normal}
    nav.tabs{display:flex;gap:2px;overflow-x:auto;-webkit-overflow-scrolling:touch}
    .tab{flex:1 1 auto;min-width:80px;padding:12px 8px;border:none;border-bottom:3px solid transparent;background:#fff;color:#444;font-size:14px;font-weight:600;cursor:pointer;border-radius:0}
    .tab:active{background:#eef}
    .tab.active{color:#36c;border-bottom-color:#36c}
    main{padding:10px;max-width:1040px;margin:0 auto}
    .tab-panel{display:none}
    .tab-panel.active{display:block}
    .card{border:1px solid #ddd;border-radius:12px;padding:14px;margin-bottom:10px;background:#fff}
    .card h2{font-size:14px;margin:0 0 10px;color:#333}
    label{font-size:12px;color:#444;display:block;margin-bottom:4px}
    input,select,button,textarea{font-size:14px;padding:8px 10px;border-radius:10px;border:1px solid #bbb;background:#fff;font-family:inherit}
    button{cursor:pointer}
    button.primary{background:#36c;color:#fff;border-color:#36c;font-weight:600;padding:10px 18px}
    button.primary:disabled{background:#aaa;border-color:#aaa;cursor:not-allowed}
    button.danger{background:#c33;color:#fff;border-color:#c33}
    button.danger:disabled{background:#aaa;border-color:#aaa}
    .grid{display:grid;grid-template-columns:140px 1fr;gap:8px 12px;align-items:center}
    @media(max-width:600px){.grid{grid-template-columns:1fr} .grid label{margin-top:6px}}
    .grid input,.grid select{width:100%}
    .row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
    .small{font-size:12px;color:#666}
    .mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
    .ok{color:#0a7;font-weight:600}.warn{color:#c70;font-weight:600}.bad{color:#c00;font-weight:600}
    .badge{padding:2px 8px;border-radius:8px;font-size:11px;font-weight:600;display:inline-block}
    .badge.cal{background:#e7f7ee;color:#0a7;border:1px solid #0a7}
    .badge.uncal{background:#fff4e0;color:#c70;border:1px solid #c70}
    pre{background:#fafafa;border:1px solid #eee;padding:10px;border-radius:10px;font-size:12px;margin:6px 0;overflow:auto;white-space:pre-wrap}
    .toast{position:fixed;right:14px;bottom:14px;background:#111;color:#fff;padding:12px 14px;border-radius:12px;opacity:0;transform:translateY(10px);transition:all .25s;pointer-events:none;max-width:320px;font-size:13px;z-index:20}
    .toast.show{opacity:0.95;transform:translateY(0)}
    .req::after{content:" *";color:#c33}

    /* Live tab specific - big readable RMS numbers */
    .rms-block{margin:10px 0}
    .rms-block .label{font-size:11px;text-transform:uppercase;letter-spacing:0.5px;color:#888;margin-bottom:2px}
    .rms-block .magnitude{font-size:32px;font-weight:700;font-variant-numeric:tabular-nums;line-height:1.1}
    .rms-block .magnitude .unit{font-size:14px;color:#666;font-weight:400;margin-left:4px}
    .rms-block .axes{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:12px;color:#444;margin-top:2px}
    .rms-block.acc .magnitude{color:#36c}
    .rms-block.vel .magnitude{color:#c33}
    .rms-block.disp .magnitude{color:#0a7}

    /* Grab state label */
    .stateBig{font-size:22px;font-weight:700;letter-spacing:0.5px}
    .stateIdle{color:#666}.stateArmed{color:#c70}.stateTrig{color:#c33}.statePost{color:#36c}.stateDone{color:#0a7}

    /* Preview tab charts */
    canvas{display:block;width:100%;height:auto;background:#fff;border:1px solid #eee;border-radius:10px}
    .row-thr{display:flex;gap:14px;flex-wrap:wrap;align-items:center;font-size:13px}
    .row-thr input[type=number]{width:80px}
  </style>
</head>
<body>
  <header>
    <h1>accMeter2 <span class="ver" id="ver">-</span></h1>
    <nav class="tabs">
      <button class="tab active" data-tab="live"   onclick="switchTab('live')">Live</button>
      <button class="tab"        data-tab="grab"   onclick="switchTab('grab')">Grab</button>
      <button class="tab"        data-tab="preview" onclick="switchTab('preview')">Preview</button>
      <button class="tab"        data-tab="update" onclick="switchTab('update')">Update</button>
    </nav>
  </header>

<main>

<!-- ========== TAB 1: LIVE (default) ========== -->
<section id="tab-live" class="tab-panel active">
  <div class="card">
    <h2>Acceleration RMS</h2>
    <div class="rms-block acc">
      <div class="label">3-axis vector magnitude</div>
      <div class="magnitude"><span id="live_acc_mag">-</span><span class="unit">m/s²</span></div>
      <div class="axes" id="live_acc_axes">x: -  y: -  z: -</div>
    </div>
  </div>
  <div class="card">
    <h2>Velocity RMS</h2>
    <div class="rms-block vel">
      <div class="label">3-axis vector magnitude</div>
      <div class="magnitude"><span id="live_vel_mag">-</span><span class="unit">mm/s</span></div>
      <div class="axes" id="live_vel_axes">x: -  y: -  z: -</div>
    </div>
  </div>
  <div class="card">
    <h2>Displacement RMS</h2>
    <div class="rms-block disp">
      <div class="label">3-axis vector magnitude</div>
      <div class="magnitude"><span id="live_disp_mag">-</span><span class="unit">mm</span></div>
      <div class="axes" id="live_disp_axes">x: -  y: -  z: -</div>
    </div>
  </div>
  <div class="card">
    <h2>Sensor settings</h2>
    <div class="grid">
      <label>Sampling rate</label>
      <select id="live_hz" onchange="setLiveConfig()">
        <option value="100">100 Hz</option>
        <option value="200">200 Hz</option>
        <option value="400">400 Hz</option>
        <option value="800" selected>800 Hz</option>
        <option value="1600">1600 Hz</option>
      </select>
      <label>G range</label>
      <select id="live_fs" onchange="setLiveConfig()">
        <option value="2" selected>±2 g</option>
        <option value="4">±4 g</option>
        <option value="8">±8 g</option>
        <option value="16">±16 g</option>
      </select>
    </div>
    <pre id="live_status" class="small mono">rate: -</pre>
  </div>
  <div class="card">
    <h2>Device</h2>
    <div class="small mono">
      Serial: <span id="live_dev_serial">-</span><br>
      Name:   <span id="live_dev_name">-</span><br>
      Cal:    <span id="live_dev_cal" class="badge uncal">UNCAL</span>
    </div>
  </div>
</section>

<!-- ========== TAB 2: GRAB ========== -->
<section id="tab-grab" class="tab-panel">
  <div class="card">
    <h2>Device</h2>
    <div class="grid">
      <label>Serial</label>
      <span id="grab_serial" class="mono small">-</span>
      <label>Name</label>
      <span class="row">
        <input id="grab_devName" placeholder="accMeter-XXXXXX" maxlength="23" style="flex:1;min-width:120px"/>
        <button onclick="saveDeviceName()">SAVE</button>
      </span>
      <label>Calibration</label>
      <span id="grab_calBadge" class="badge uncal">UNCAL</span>
    </div>
  </div>

  <div class="card">
    <h2>Measurement Metadata</h2>
    <div class="grid">
      <label>Date / Time</label>
      <span id="grab_ts" class="mono small">auto from browser</span>
      <label>Scanned system S/N</label>
      <input id="grab_scannedSerial" placeholder="serial of the machine" maxlength="23"/>
      <label class="req">Measurement point</label>
      <input id="grab_measPoint" placeholder="e.g. Motor-DE" maxlength="23" required/>
      <label class="req">Scan direction</label>
      <select id="grab_scanDir">
        <option value="RADIAL_H">Radial - Horizontal</option>
        <option value="RADIAL_V">Radial - Vertical</option>
        <option value="AXIAL">Axial</option>
        <option value="X_GC">G-&gt;Ç (X axis, Z=gravity)</option>
        <option value="X_CG">Ç-&gt;G (X axis, Z=gravity)</option>
      </select>
      <label>Operator</label>
      <input id="grab_operator" placeholder="(optional)" maxlength="15"/>
      <label>Notes</label>
      <input id="grab_notes" placeholder="(optional)" maxlength="63"/>
    </div>
  </div>

  <div class="card">
    <h2>Acquisition</h2>
    <div class="grid">
      <label>Sampling rate</label>
      <select id="grab_hz">
        <option value="100">100 Hz</option>
        <option value="200">200 Hz</option>
        <option value="400">400 Hz</option>
        <option value="800" selected>800 Hz</option>
        <option value="1600">1600 Hz</option>
      </select>
      <label>G range</label>
      <select id="grab_fs">
        <option value="2" selected>±2 g</option>
        <option value="4">±4 g</option>
        <option value="8">±8 g</option>
        <option value="16">±16 g</option>
      </select>
      <label>Max duration</label>
      <select id="grab_maxS">
        <option value="15">15 s</option>
        <option value="30">30 s</option>
        <option value="45">45 s</option>
        <option value="60" selected>60 s</option>
        <option value="90">90 s</option>
        <option value="120">120 s (max)</option>
      </select>
    </div>
  </div>

  <div class="card">
    <h2>Trigger</h2>
    <div class="grid">
      <label>Pre-roll</label>
      <span class="row"><input id="grab_preS" type="number" min="1" max="10" value="3" style="width:80px"/> s</span>
      <label>Post-roll</label>
      <span class="row"><input id="grab_postS" type="number" min="1" max="30" value="5" style="width:80px"/> s</span>
      <label>Threshold</label>
      <div>
        <div class="row-thr">
          <label><input type="radio" name="grab_thrMode" value="0" checked onchange="updateGrabThrMode()"/>
          Auto baseline × <input id="grab_mult" type="number" min="2" max="20" value="3"/></label>
        </div>
        <div class="row-thr" style="margin-top:6px">
          <label><input type="radio" name="grab_thrMode" value="1" onchange="updateGrabThrMode()"/>
          Manual: <input id="grab_manThr" type="number" min="0.01" max="100" step="0.01" value="0.50"/> m/s²</label>
        </div>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="row" style="gap:14px">
      <button id="grab_armBtn" class="primary" onclick="armGrab()">START GRAB</button>
      <button id="grab_disarmBtn" class="danger" onclick="disarmGrab()" disabled>DISARM</button>
    </div>
  </div>

  <div class="card">
    <h2>Status</h2>
    <div id="grab_stateLabel" class="stateBig stateIdle">IDLE</div>
    <pre id="grab_stateDetail">Fill in the metadata above and press START GRAB.</pre>
  </div>

  <div class="card" id="grab_lastFileCard" style="display:none">
    <h2>Last saved</h2>
    <div id="grab_lastFile" class="mono small"></div>
    <div style="margin-top:8px"><button onclick="switchTab('preview')">Open Preview tab to analyze →</button></div>
  </div>
</section>

<!-- ========== TAB 3: PREVIEW (charts, files, analysis, calibration) ========== -->
<section id="tab-preview" class="tab-panel">
  <div class="card">
    <h2>Status</h2>
    <div id="prev_status" class="small">Loading...</div>
    <div id="prev_fsinfo" class="small">FS: ...</div>
    <pre id="prev_info">...</pre>
    <div class="row" style="margin-top:8px">
      <button onclick="doReset()">RESET DEVICE</button>
    </div>
  </div>

  <div class="card">
    <h2>Manual recording</h2>
    <div class="grid">
      <label>Sampling rate</label>
      <select id="prev_hz">
        <option value="2">1.6 Hz (LP)</option>
        <option value="13">12.5 Hz</option>
        <option value="25">25 Hz</option>
        <option value="50">50 Hz</option>
        <option value="100" selected>100 Hz</option>
        <option value="200">200 Hz</option>
        <option value="400">400 Hz</option>
        <option value="800">800 Hz</option>
        <option value="1600">1600 Hz</option>
      </select>
      <label>G range</label>
      <select id="prev_fs">
        <option value="2" selected>±2 g</option>
        <option value="4">±4 g</option>
        <option value="8">±8 g</option>
        <option value="16">±16 g</option>
      </select>
      <label>Record time</label>
      <select id="prev_sec">
        <option value="15">15 s</option>
        <option value="30">30 s</option>
        <option value="45">45 s</option>
        <option value="60" selected>60 s</option>
        <option value="75">75 s</option>
        <option value="90">90 s</option>
        <option value="120">120 s</option>
      </select>
      <label>Scanned S/N</label>
      <input id="prev_scannedSerial" placeholder="(optional)" maxlength="23"/>
      <label class="req">Meas point</label>
      <input id="prev_measPoint" placeholder="e.g. Motor-DE" maxlength="23"/>
      <label class="req">Scan dir</label>
      <select id="prev_scanDir">
        <option value="RADIAL_H">Radial - Horizontal</option>
        <option value="RADIAL_V">Radial - Vertical</option>
        <option value="AXIAL">Axial</option>
        <option value="X_GC">G-&gt;Ç (X axis)</option>
        <option value="X_CG">Ç-&gt;G (X axis)</option>
      </select>
    </div>
    <div class="row" style="margin-top:10px">
      <button onclick="startManualRec()">START</button>
      <button onclick="stopManualRec()">STOP</button>
    </div>
  </div>

  <div class="card">
    <h2>Files</h2>
    <select id="prev_fileSel" style="width:100%"></select>
    <div class="row" style="margin-top:10px;gap:8px">
      <button onclick="downloadBin()">DOWNLOAD .DAT</button>
      <button onclick="downloadCsv()">DOWNLOAD .CSV</button>
      <button onclick="deleteSel()">DELETE</button>
    </div>
    <div class="row" style="margin-top:10px;gap:8px">
      <button onclick="calibrateStatic()">CAL (STATIC Z-UP)</button>
      <button onclick="calibrate6()">CAL (6-POS)</button>
    </div>
  </div>

  <div class="card">
    <h2>Live preview (250 ms RMS @ live tab settings)</h2>
    <canvas id="prev_chart" width="420" height="160"></canvas>
    <div class="small" id="prev_liveRate">rate: -</div>
    <pre id="prev_liveText">acc: -, vel: -, disp: -</pre>
    <div class="small" style="margin-top:6px">
      <label><input type="checkbox" id="prev_chkRealtime" onchange="toggleRealtime(this.checked)"> Realtime ISO 20816 (mag only)</label>
      <select id="prev_maskBits" onchange="setMaskBits()" style="margin-left:8px">
        <option value="0" selected>Mask: off</option>
        <option value="2">Mask: 2-bit</option>
        <option value="3">Mask: 3-bit</option>
        <option value="4">Mask: 4-bit</option>
      </select>
    </div>
    <pre id="prev_rtStats" class="small mono">-</pre>
  </div>

  <div class="card">
    <h2>Time domain analysis</h2>
    <div class="row">
      <button onclick="analyzeSelected()">ANALYZE</button>
      <button onclick="clearAnalysis()">CLEAR</button>
    </div>
    <div class="small" id="prev_anaMeta" style="margin-top:8px">Select a file and press ANALYZE.</div>
    <pre id="prev_stats">-</pre>
    <canvas id="prev_bigChart" width="980" height="320"></canvas>
  </div>

  <div class="card">
    <h2>Frequency domain (FFT)</h2>
    <div class="row">
      <select id="prev_fftAxis">
        <option value="x">X</option>
        <option value="y">Y</option>
        <option value="z">Z</option>
      </select>
      <button onclick="runFFT()">FFT</button>
    </div>
    <canvas id="prev_fftChart" width="980" height="280" style="margin-top:8px"></canvas>
    <pre id="prev_fftInfo" class="small">-</pre>
  </div>
</section>

<!-- ========== TAB 4: UPDATE ========== -->
<section id="tab-update" class="tab-panel">
  <div class="card">
    <h2>Firmware update</h2>
    <div class="small">Current build: <span id="up_ver" class="mono">loading...</span></div>
    <div class="small" style="margin-top:8px">
      During recording or calibration the device refuses uploads.
      Select a <b>.bin</b> built for esp32doit-devkit-v1.
    </div>
    <div class="row" style="margin-top:12px">
      <input id="up_file" type="file" accept=".bin"/>
      <button id="up_btn" onclick="startUpload()">UPLOAD</button>
    </div>
    <div style="margin-top:12px">
      <progress id="up_prog" value="0" max="100" style="width:100%;height:18px"></progress>
      <div id="up_text" class="small mono">0%</div>
    </div>
    <div id="up_msg" class="small">Ready.</div>
  </div>
</section>

</main>

<div id="toast" class="toast"></div>

<script>
/* =====================================================================
   Shared utilities
   ===================================================================== */
async function getJson(p){
  const r=await fetch(p,{cache:'no-store'});
  if(!r.ok) throw new Error('HTTP '+r.status);
  return await r.json();
}
async function postText(p,params){
  const body=params? Object.entries(params).map(([k,v])=>encodeURIComponent(k)+'='+encodeURIComponent(v)).join('&') : '';
  const r=await fetch(p,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body,cache:'no-store'});
  return {ok:r.ok,text:await r.text()};
}
function esc(s){return encodeURIComponent(s);}
function escHtml(s){return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;');}
let _toastT=null;
function toast(m){const t=document.getElementById('toast');t.textContent=m;t.classList.add('show');
  if(_toastT) clearTimeout(_toastT); _toastT=setTimeout(()=>{t.classList.remove('show');_toastT=null;},2600);}

function tsYYMMDDHHMMSS(){
  const d=new Date();
  const yy=String(d.getFullYear()).slice(-2);
  const MM=String(d.getMonth()+1).padStart(2,'0');
  const DD=String(d.getDate()).padStart(2,'0');
  const HH=String(d.getHours()).padStart(2,'0');
  const mm=String(d.getMinutes()).padStart(2,'0');
  const ss=String(d.getSeconds()).padStart(2,'0');
  return yy+MM+DD+HH+mm+ss;
}
function nowPretty(){
  const d=new Date();
  const DD=String(d.getDate()).padStart(2,'0');
  const MM=String(d.getMonth()+1).padStart(2,'0');
  const YYYY=d.getFullYear();
  const HH=String(d.getHours()).padStart(2,'0');
  const mm=String(d.getMinutes()).padStart(2,'0');
  const ss=String(d.getSeconds()).padStart(2,'0');
  return DD+'/'+MM+'/'+YYYY+' '+HH+':'+mm+':'+ss;
}

/* =====================================================================
   Tab switching
   URL hash "#grab" / "#preview" / "#update" jumps to tab on load.
   Default tab is "live".
   ===================================================================== */
let _currentTab='live';
function switchTab(name){
  _currentTab=name;
  document.querySelectorAll('.tab').forEach(t=>{
    t.classList.toggle('active',t.dataset.tab===name);
  });
  document.querySelectorAll('.tab-panel').forEach(p=>{
    p.classList.toggle('active',p.id==='tab-'+name);
  });
  if(history.replaceState) history.replaceState(null,'','#'+name);
  // First-time-on-tab triggers
  if(name==='preview' && !_prevInited){_prevInited=true; refreshFiles(); refreshFsInfo();}
}

/* =====================================================================
   Shared polling
   /api/info every 1s -- drives state for all tabs
   /api/live every 1s while on Live or Preview tab
   /api/list, /api/fsinfo periodic only when on Preview tab
   ===================================================================== */
const G=9.80665;
let _lastTrigState='Idle';
let _stoppedAt=0, _stoppedFile='';
let _devLoadedOnce=false;
let _prevInited=false;

async function loadVersion(){
  try{
    const j=await getJson('/api/version');
    document.getElementById('ver').textContent=j.version;
    document.getElementById('up_ver').textContent=j.version+' ('+j.hash+') built: '+j.built;
  }catch(e){}
}

async function refreshInfo(){
  try{
    const j=await getJson('/api/info');
    const state=j.trigState||'Idle';
    const calOk=!!j.calibrated;

    // ---------- Header / device fields (Live + Grab cards) ----------
    document.getElementById('live_dev_serial').textContent=j.serial||'-';
    document.getElementById('live_dev_name').textContent=j.deviceName||'-';
    document.getElementById('live_dev_cal').textContent=calOk?'CAL':'UNCAL';
    document.getElementById('live_dev_cal').className='badge '+(calOk?'cal':'uncal');

    document.getElementById('grab_serial').textContent=j.serial||'-';
    document.getElementById('grab_calBadge').textContent=calOk?'CAL':'UNCAL';
    document.getElementById('grab_calBadge').className='badge '+(calOk?'cal':'uncal');
    if(!_devLoadedOnce){
      document.getElementById('grab_devName').value=j.deviceName||'';
      _devLoadedOnce=true;
    }

    // ---------- Live tab dropdowns reflect server's current config ----------
    if(j.liveHz){
      const sel=document.getElementById('live_hz');
      if(sel.value!=String(j.liveHz)) sel.value=String(j.liveHz);
    }
    if(j.liveFs){
      const sel=document.getElementById('live_fs');
      if(sel.value!=String(j.liveFs)) sel.value=String(j.liveFs);
    }

    // ---------- Grab buttons + state label ----------
    document.getElementById('grab_armBtn').disabled=(state!=='Idle');
    document.getElementById('grab_disarmBtn').disabled=(state==='Idle');

    // Latch transient STOPPED state on Triggered/PostTail -> Idle
    if(_lastTrigState!=='Idle' && state==='Idle'){
      _stoppedAt=Date.now();
      _stoppedFile=j.currentFile||'';
      if(j.currentFile){
        document.getElementById('grab_lastFileCard').style.display='';
        document.getElementById('grab_lastFile').textContent=j.currentFile+'  (samples='+j.samples+')';
        toast('Saved: '+j.currentFile);
      }
    }
    if(_stoppedAt>0 && (Date.now()-_stoppedAt)>=10000){_stoppedAt=0;_stoppedFile='';}
    _lastTrigState=state;

    setGrabStateUI(state,j);

    if(state==='Idle' && _stoppedAt===0){
      document.getElementById('grab_ts').textContent=nowPretty()+' (will be auto-set on ARM)';
    }

    // ---------- Preview tab status ----------
    if(_prevInited){
      const stEl=document.getElementById('prev_status');
      let flags=[];
      if(j.calibratingStatic) flags.push('CAL(STATIC)');
      if(j.calibrating6) flags.push('CAL(6POS:'+escHtml(j.calibPose||'')+')');
      const calBadge=calOk?"<span class='ok'>CAL</span>":"<span class='warn'>UNCAL</span>";
      stEl.innerHTML=
        (j.recording?"<span class='warn'>RECORDING</span>":"<span class='ok'>IDLE</span>")
        +' | '+calBadge
        +' | mode: '+escHtml(j.mode||'-')
        +' | currentFile: '+escHtml(j.currentFile||'-')
        +' | samples: '+j.samples
        +' | elapsed: '+Math.round((j.elapsedMs||0)/1000)+' s'
        +' | maxBacklog: '+j.maxBacklog
        +(flags.length?(' | <span class="warn">'+flags.join(' ')+'</span>'):'');
      document.getElementById('prev_info').textContent=JSON.stringify(j,null,2);
    }
  }catch(e){console.warn('refreshInfo',e);}
}

/* =====================================================================
   Live tab
   ===================================================================== */
async function refreshLive(){
  if(_currentTab!=='live' && _currentTab!=='preview') return;
  try{
    const j=await getJson('/api/live');
    if(j.enabled===false) return;

    if(_currentTab==='live'){
      // Compact RMS readings
      if(typeof j.ax==='number'){
        document.getElementById('live_acc_mag').textContent=j.mag.toFixed(4);
        document.getElementById('live_acc_axes').textContent=
          'x: '+j.ax.toFixed(4)+'  y: '+j.ay.toFixed(4)+'  z: '+j.az.toFixed(4);
        document.getElementById('live_vel_mag').textContent=j.vmag_mmps.toFixed(3);
        document.getElementById('live_vel_axes').textContent=
          'x: '+j.vx_mmps.toFixed(3)+'  y: '+j.vy_mmps.toFixed(3)+'  z: '+j.vz_mmps.toFixed(3);
        document.getElementById('live_disp_mag').textContent=j.dmag_mm.toFixed(4);
        document.getElementById('live_disp_axes').textContent=
          'x: '+j.dx_mm.toFixed(4)+'  y: '+j.dy_mm.toFixed(4)+'  z: '+j.dz_mm.toFixed(4);
        const eff=j.eff_hz||0;
        const dt=j.dt_us||0;
        document.getElementById('live_status').textContent=
          'rate: '+eff.toFixed(1)+' Hz  (dt '+dt+' µs)  fc: '+(j.fc||0).toFixed(1)+' Hz';
      }
    }

    if(_currentTab==='preview'){
      _refreshPreviewLive(j);
    }
  }catch(e){console.warn('refreshLive',e);}
}

async function setLiveConfig(){
  const hz=document.getElementById('live_hz').value;
  const fs=document.getElementById('live_fs').value;
  const r=await postText('/api/live_config',{hz,fs});
  if(!r.ok){alert('Live config failed: '+r.text);return;}
  toast('Live: '+hz+' Hz, ±'+fs+' g');
}

/* =====================================================================
   Grab tab
   ===================================================================== */
function setGrabStateUI(state,j){
  const lbl=document.getElementById('grab_stateLabel');
  const det=document.getElementById('grab_stateDetail');
  lbl.classList.remove('stateIdle','stateArmed','stateTrig','statePost','stateDone');
  let txt='';
  const showStopped=(state==='Idle' && _stoppedAt>0 && (Date.now()-_stoppedAt)<10000);
  if(showStopped){
    lbl.textContent='STOPPED — saved';
    lbl.classList.add('stateDone');
    det.textContent=_stoppedFile?('Last file: '+_stoppedFile+'\n(reverting to IDLE in a few seconds)'):'(saved)';
    return;
  }
  switch(state){
    case 'Idle':
      lbl.textContent='IDLE'; lbl.classList.add('stateIdle');
      txt='Fill in the metadata above and press START GRAB to arm.';
      break;
    case 'Armed':{
      lbl.textContent='READY — waiting for motion';
      lbl.classList.add('stateArmed');
      txt='armed for '+((j.trigArmedMs||0)/1000).toFixed(1)+' s\n'
        +'baseline: '+(j.trigBaseline||0).toFixed(4)+' m/s²\n'
        +'threshold: '+(j.trigThreshold||0).toFixed(4)+' m/s²\n'
        +'current window RMS: '+(j.trigCurrentRms||0).toFixed(4)+' m/s²';
      break;}
    case 'Triggered':{
      lbl.textContent='RECORDING'; lbl.classList.add('stateTrig');
      txt='recording for '+((j.trigFiredMs||0)/1000).toFixed(1)+' s\n'
        +'samples written: '+(j.samples||0)+'\n'
        +'current RMS: '+(j.trigCurrentRms||0).toFixed(4)+' m/s² (threshold '+(j.trigThreshold||0).toFixed(4)+')';
      break;}
    case 'PostTail':{
      lbl.textContent='RECORDING — settling'; lbl.classList.add('statePost');
      txt='total '+((j.trigFiredMs||0)/1000).toFixed(1)+' s elapsed since trigger\n'
        +'samples written: '+(j.samples||0)+'\nfinishing tail recording...';
      break;}
  }
  det.textContent=txt;
}
function updateGrabThrMode(){
  const mode=document.querySelector('input[name=grab_thrMode]:checked').value;
  document.getElementById('grab_mult').disabled=(mode==='1');
  document.getElementById('grab_manThr').disabled=(mode==='0');
}
async function saveDeviceName(){
  const name=document.getElementById('grab_devName').value.trim();
  if(!name){alert('Name is required');return;}
  if(name.length>23){alert('Max 23 chars');return;}
  const r=await postText('/api/device',{name});
  if(!r.ok){alert('Save failed: '+r.text);return;}
  toast('Device name saved');
}
async function armGrab(){
  const measPoint=document.getElementById('grab_measPoint').value.trim();
  const scanDir=document.getElementById('grab_scanDir').value;
  if(!measPoint){alert('Measurement point is required');return;}
  if(!scanDir){alert('Scan direction is required');return;}
  const params={
    hz:document.getElementById('grab_hz').value,
    fs:document.getElementById('grab_fs').value,
    max_s:document.getElementById('grab_maxS').value,
    pre_s:document.getElementById('grab_preS').value,
    post_s:document.getElementById('grab_postS').value,
    mode:document.querySelector('input[name=grab_thrMode]:checked').value,
    mult:document.getElementById('grab_mult').value,
    abs_thr:document.getElementById('grab_manThr').value,
    ts:tsYYMMDDHHMMSS(),
    meas_point:measPoint,
    scan_dir:scanDir,
    operator:document.getElementById('grab_operator').value.trim(),
    notes:document.getElementById('grab_notes').value.trim(),
    scanned_serial:document.getElementById('grab_scannedSerial').value.trim()
  };
  const r=await postText('/api/trigger_arm',params);
  if(!r.ok){alert('Arm failed: '+r.text);return;}
  document.getElementById('grab_lastFileCard').style.display='none';
  toast('ARMED');
}
async function disarmGrab(){
  if(!confirm('Disarm? Active recording (if any) will close cleanly.')) return;
  const r=await postText('/api/trigger_disarm');
  if(!r.ok){alert('Disarm failed: '+r.text);return;}
  toast('Disarm requested');
}

/* =====================================================================
   Preview tab (manual recording, files, live chart, analysis, FFT, calibration)
   ===================================================================== */
async function refreshFsInfo(){
  if(_currentTab!=='preview') return;
  try{
    const j=await getJson('/api/fsinfo');
    const fmt=x=> x<1024*1024?(x/1024).toFixed(1)+' KB':(x/1024/1024).toFixed(2)+' MB';
    document.getElementById('prev_fsinfo').textContent=
      'FS: used '+fmt(j.used)+' / total '+fmt(j.total)+' (free '+fmt(j.free)+')';
  }catch(e){}
}
async function refreshFiles(selectName=''){
  if(_currentTab!=='preview') return;
  try{
    const files=await getJson('/api/list');
    const sel=document.getElementById('prev_fileSel');
    const current=selectName||sel.value;
    sel.innerHTML='';
    let keep='';
    files.sort((a,b)=>(b.name.localeCompare(a.name)));
    for(const f of files){
      const opt=document.createElement('option');
      opt.value=f.name;
      opt.textContent=prettyName(f.name)+'  ('+f.size+' B)';
      sel.appendChild(opt);
      if(f.name===current) keep=current;
    }
    if(keep) sel.value=keep;
  }catch(e){}
}
function prettyName(filePath){
  let n=filePath||'';
  if(n.startsWith('/')) n=n.slice(1);
  const m=n.match(/^(accel|grab)(\d{12})(?:_\d{2})?\.dat$/);
  if(!m) return n;
  const tag=(m[1]==='grab')?'[G]':'[M]';
  const ts=m[2];
  return tag+' '+n+'  ['+ts.slice(4,6)+'/'+ts.slice(2,4)+'/20'+ts.slice(0,2)+' '+ts.slice(6,8)+':'+ts.slice(8,10)+':'+ts.slice(10,12)+']';
}

async function startManualRec(){
  const meas_point=document.getElementById('prev_measPoint').value.trim();
  const scan_dir=document.getElementById('prev_scanDir').value;
  const scanned_serial=document.getElementById('prev_scannedSerial').value.trim();
  if(!meas_point){alert('Measurement point is required');return;}
  if(!scan_dir){alert('Scan direction is required');return;}
  const params={
    hz:document.getElementById('prev_hz').value,
    fs:document.getElementById('prev_fs').value,
    sec:document.getElementById('prev_sec').value,
    ts:tsYYMMDDHHMMSS(),
    meas_point,scan_dir,scanned_serial
  };
  const r=await postText('/api/start',params);
  if(!r.ok){alert(r.text);return;}
  toast('STARTED');
  refreshFiles(); refreshFsInfo();
}
async function stopManualRec(){
  const r=await postText('/api/stop');
  if(!r.ok){alert(r.text);return;}
  toast('STOP requested');
}
function downloadBin(){
  const sel=document.getElementById('prev_fileSel').value;
  if(!sel){alert('No file selected');return;}
  window.location.href='/download?file='+esc(sel);
}
function downloadCsv(){
  const sel=document.getElementById('prev_fileSel').value;
  if(!sel){alert('No file selected');return;}
  window.location.href='/download_csv?file='+esc(sel);
}
async function deleteSel(){
  const sel=document.getElementById('prev_fileSel').value;
  if(!sel){alert('No file selected');return;}
  const r=await postText('/api/delete',{file:sel});
  if(!r.ok){alert(r.text);return;}
  toast('Deleted: '+sel);
  refreshFiles(); refreshFsInfo();
}
async function calibrateStatic(){
  if(!confirm('STATIC CAL\nDevice still, +Z UP.\nStart?')) return;
  const r=await postText('/api/calibrate_static');
  if(!r.ok) alert(r.text); else toast('Static cal started');
}
async function calibrate6(){
  alert('6-POS Calibration: follow X+, X-, Y+, Y-, Z+, Z-. Watch calibPose.');
  const r=await postText('/api/calibrate6');
  if(!r.ok) alert(r.text); else toast('6-POS cal started');
}
async function doReset(){
  if(!confirm('Device will reboot. Continue?')) return;
  toast('Rebooting...');
  await postText('/api/reset');
}

// --- Preview live chart (60-sample rolling history of g_acc_mag in g) ---
const CHART_N=60;
let _axBuf=new Array(CHART_N).fill(0), _ayBuf=new Array(CHART_N).fill(0), _azBuf=new Array(CHART_N).fill(0);
let _bufIdx=0, _bufCount=0;
function _pushSample(ax,ay,az){
  _axBuf[_bufIdx]=ax;_ayBuf[_bufIdx]=ay;_azBuf[_bufIdx]=az;
  _bufIdx=(_bufIdx+1)%CHART_N;
  _bufCount=Math.min(_bufCount+1,CHART_N);
}
function _getBuf(buf,i){const start=(_bufIdx-_bufCount+CHART_N)%CHART_N;return buf[(start+i)%CHART_N];}
function _drawPrevChart(){
  const c=document.getElementById('prev_chart'); if(!c) return;
  const ctx=c.getContext('2d'); const w=c.width,h=c.height;
  ctx.clearRect(0,0,w,h);
  const mL=36,mR=10,mT=10,mB=18;
  const pw=w-mL-mR, ph=h-mT-mB;
  let ymin=Infinity,ymax=-Infinity;
  for(let i=0;i<_bufCount;i++){
    const ax=_getBuf(_axBuf,i),ay=_getBuf(_ayBuf,i),az=_getBuf(_azBuf,i);
    ymin=Math.min(ymin,ax,ay,az); ymax=Math.max(ymax,ax,ay,az);
  }
  if(!isFinite(ymin)||!isFinite(ymax)){ymin=-1;ymax=1;}
  if(ymax-ymin<0.01){const m=(ymax+ymin)/2; ymin=m-0.01; ymax=m+0.01;}
  const pad=(ymax-ymin)*0.12; ymin-=pad; ymax+=pad;
  const xAt=i=>mL+(_bufCount<=1?0:(i/(_bufCount-1))*pw);
  const yAt=v=>mT+((ymax-v)/(ymax-ymin))*ph;
  ctx.strokeStyle='#eee';
  for(let g=0;g<=4;g++){const y=mT+(g/4)*ph;ctx.beginPath();ctx.moveTo(mL,y);ctx.lineTo(mL+pw,y);ctx.stroke();}
  ctx.strokeStyle='#ccc'; ctx.strokeRect(mL,mT,pw,ph);
  function plot(buf,col){
    if(_bufCount<2) return;
    ctx.strokeStyle=col; ctx.lineWidth=2; ctx.beginPath();
    for(let i=0;i<_bufCount;i++){const v=_getBuf(buf,i),x=xAt(i),y=yAt(v); if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);}
    ctx.stroke();
  }
  plot(_axBuf,'#d33'); plot(_ayBuf,'#3a3'); plot(_azBuf,'#36c');
}

let _maskBits=0;
function setMaskBits(){
  _maskBits=parseInt(document.getElementById('prev_maskBits').value,10)||0;
  postText('/api/rawmask',{bits:_maskBits});
}
async function toggleRealtime(on){
  const r=await postText('/api/realtime',{enable:on?1:0,mask:_maskBits});
  if(!r.ok){alert(r.text);document.getElementById('prev_chkRealtime').checked=!on;return;}
}
function _refreshPreviewLive(j){
  const eff=j.eff_hz||0, dt=j.dt_us||0;
  document.getElementById('prev_liveRate').textContent='rate: '+eff.toFixed(1)+' Hz (dt '+dt+' µs)';
  if(j.realtime){
    const a1=j.avg1||{}, a5=j.avg5||{}, a10=j.avg10||{};
    document.getElementById('prev_rtStats').textContent=
      'RMS (mean-removed, noise-corrected acc):\n'+
      '1s  acc:'+(a1.acc_mps2||0).toFixed(3)+' m/s²  vel:'+(a1.vel_mmps||0).toFixed(3)+' mm/s  disp:'+(a1.disp_mm||0).toFixed(4)+' mm\n'+
      '5s  acc:'+(a5.acc_mps2||0).toFixed(3)+'         vel:'+(a5.vel_mmps||0).toFixed(3)+'         disp:'+(a5.disp_mm||0).toFixed(4)+'\n'+
      '10s acc:'+(a10.acc_mps2||0).toFixed(3)+'         vel:'+(a10.vel_mmps||0).toFixed(3)+'         disp:'+(a10.disp_mm||0).toFixed(4)+'\n'+
      'noise floor: '+(j.noise_mps2||0).toFixed(4)+' m/s²';
    return;
  }
  if(typeof j.ax!=='number') return;
  document.getElementById('prev_liveText').textContent=
    'ACC RMS (m/s²)  X:'+j.ax.toFixed(4)+'  Y:'+j.ay.toFixed(4)+'  Z:'+j.az.toFixed(4)+'  MAG:'+j.mag.toFixed(4)+'\n'+
    'VEL RMS (mm/s)  X:'+j.vx_mmps.toFixed(3)+'  Y:'+j.vy_mmps.toFixed(3)+'  Z:'+j.vz_mmps.toFixed(3)+'  MAG:'+j.vmag_mmps.toFixed(3)+'\n'+
    'DISP RMS (mm)   X:'+j.dx_mm.toFixed(4)+'  Y:'+j.dy_mm.toFixed(4)+'  Z:'+j.dz_mm.toFixed(4)+'  MAG:'+j.dmag_mm.toFixed(4);
  _pushSample(j.ax/G,j.ay/G,j.az/G);
  _drawPrevChart();
}

/* --- Analyze + FFT --- */
async function analyzeSelected(){
  const file=document.getElementById('prev_fileSel').value;
  if(!file){alert('No file selected');return;}
  try{
    toast('Analyzing...');
    document.getElementById('prev_anaMeta').textContent='Analyzing on device...';
    const j=await getJson('/api/analyze?file='+esc(file));
    _drawBigChart(j.ax,j.ay,j.az,j.eff_hz||j.rate_hz||1);
    document.getElementById('prev_stats').textContent=
      'File: '+j.file+'\nrate_hz: '+j.rate_hz+'  record_s: '+j.record_s
      +'\nsamples_header: '+j.samples_header+'  samples_used: '+j.samples_used
      +'\nfs_g: ±'+j.fs_g+'  res_bits: '+j.res_bits+'  q_bits: '+j.q_bits
      +'\nX min/max/rms: '+j.min[0].toFixed(4)+' / '+j.max[0].toFixed(4)+' / '+j.rms[0].toFixed(4)
      +'\nY min/max/rms: '+j.min[1].toFixed(4)+' / '+j.max[1].toFixed(4)+' / '+j.rms[1].toFixed(4)
      +'\nZ min/max/rms: '+j.min[2].toFixed(4)+' / '+j.max[2].toFixed(4)+' / '+j.rms[2].toFixed(4);
    document.getElementById('prev_anaMeta').textContent='OK. Rendered '+j.pts+' points (downsampled).';
    toast('Analysis done');
  }catch(e){alert('ANALYZE failed: '+(e?.message||e));}
}
function clearAnalysis(){
  document.getElementById('prev_stats').textContent='-';
  document.getElementById('prev_anaMeta').textContent='Select a file and press ANALYZE.';
  const c=document.getElementById('prev_bigChart'); c.getContext('2d').clearRect(0,0,c.width,c.height);
}
function _drawBigChart(ax,ay,az,rateHz){
  const c=document.getElementById('prev_bigChart'); if(!c) return;
  const ctx=c.getContext('2d'); const w=c.width,h=c.height;
  ctx.clearRect(0,0,w,h);
  const mL=50,mR=10,mT=10,mB=30; const pw=w-mL-mR, ph=h-mT-mB;
  let ymin=Infinity,ymax=-Infinity;
  [...ax,...ay,...az].forEach(v=>{ymin=Math.min(ymin,v);ymax=Math.max(ymax,v);});
  if(!isFinite(ymin)||!isFinite(ymax)){ymin=-1;ymax=1;}
  if(ymax-ymin<0.01){const m=(ymax+ymin)/2;ymin=m-0.01;ymax=m+0.01;}
  const xAt=(i,n)=>mL+(i/(n-1))*pw, yAt=v=>mT+((ymax-v)/(ymax-ymin))*ph;
  ctx.strokeStyle='#eee';
  for(let i=0;i<=4;i++){const y=mT+(i/4)*ph;ctx.beginPath();ctx.moveTo(mL,y);ctx.lineTo(mL+pw,y);ctx.stroke();}
  ctx.strokeStyle='#ccc'; ctx.strokeRect(mL,mT,pw,ph);
  function plot(arr,col){if(arr.length<2)return;ctx.strokeStyle=col;ctx.lineWidth=2;ctx.beginPath();
    for(let i=0;i<arr.length;i++){const x=xAt(i,arr.length),y=yAt(arr[i]);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}ctx.stroke();}
  plot(ax,'#d33'); plot(ay,'#2a2'); plot(az,'#26c');
  ctx.fillStyle='#666'; ctx.fillText((ax.length/rateHz).toFixed(2)+' s',w-70,h-8);
}
async function runFFT(){
  const file=document.getElementById('prev_fileSel').value;
  const axis=document.getElementById('prev_fftAxis').value;
  if(!file){alert('Select file');return;}
  try{
    const j=await getJson('/api/fft?file='+esc(file)+'&axis='+axis);
    _drawFFT(j.fft,j.df);
    document.getElementById('prev_fftInfo').textContent='Axis: '+j.axis+'  Peak: '+j.peak_hz.toFixed(2)+' Hz  Mag: '+j.peak_mag.toFixed(4);
  }catch(e){alert('FFT failed: '+(e?.message||e));}
}
function _drawFFT(arr,df){
  const c=document.getElementById('prev_fftChart');
  const ctx=c.getContext('2d'); ctx.clearRect(0,0,c.width,c.height);
  if(!arr||arr.length<2||!df) return;
  const w=c.width,h=c.height,mL=50,mR=10,mT=10,mB=30, pw=w-mL-mR, ph=h-mT-mB;
  let max=Math.max(...arr); if(max<=0) max=1;
  ctx.strokeStyle='#ccc'; ctx.strokeRect(mL,mT,pw,ph);
  ctx.strokeStyle='#36c'; ctx.beginPath();
  const div=Math.max(1,arr.length-1);
  for(let i=0;i<arr.length;i++){const x=mL+(i/div)*pw, y=mT+(1-arr[i]/max)*ph; if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}
  ctx.stroke();
  ctx.fillStyle='#666'; ctx.fillText('0 Hz',mL,h-8); ctx.fillText((arr.length*df).toFixed(1)+' Hz',w-80,h-8);
}

/* =====================================================================
   Update tab (inline OTA upload form)
   ===================================================================== */
function startUpload(){
  const f=document.getElementById('up_file').files[0];
  if(!f){alert('Select a .bin file');return;}
  document.getElementById('up_btn').disabled=true;
  document.getElementById('up_file').disabled=true;
  document.getElementById('up_prog').value=0;
  document.getElementById('up_text').textContent='0%';
  document.getElementById('up_msg').textContent='Uploading...';
  document.getElementById('up_msg').className='small warn';
  const form=new FormData(); form.append('update',f,f.name);
  const xhr=new XMLHttpRequest(); xhr.open('POST','/update',true);
  xhr.upload.onprogress=e=>{if(e.lengthComputable){
    const p=(e.loaded/e.total)*100;
    document.getElementById('up_prog').value=p;
    document.getElementById('up_text').textContent=p.toFixed(0)+'%';
  }};
  xhr.onload=()=>{
    if(xhr.status===200){
      document.getElementById('up_prog').value=100;
      document.getElementById('up_text').textContent='100%';
      document.getElementById('up_msg').textContent=(xhr.responseText||'OK')+' (page will disconnect, device rebooting)';
      document.getElementById('up_msg').className='small ok';
    }else{
      document.getElementById('up_msg').textContent='Upload failed: HTTP '+xhr.status+' '+(xhr.responseText||'');
      document.getElementById('up_msg').className='small bad';
      document.getElementById('up_btn').disabled=false;
      document.getElementById('up_file').disabled=false;
    }
  };
  xhr.onerror=()=>{
    document.getElementById('up_msg').textContent='Upload error (network)';
    document.getElementById('up_msg').className='small bad';
    document.getElementById('up_btn').disabled=false;
    document.getElementById('up_file').disabled=false;
  };
  xhr.send(form);
}

/* =====================================================================
   Init
   ===================================================================== */
const initialHash=(location.hash||'#live').slice(1);
if(['live','grab','preview','update'].includes(initialHash)) switchTab(initialHash);

setInterval(refreshInfo,1000);
setInterval(refreshLive,1000);
setInterval(refreshFiles,3000);
setInterval(refreshFsInfo,5000);

loadVersion();
refreshInfo();
updateGrabThrMode();
</script>
</body>
</html>

)HTML";
