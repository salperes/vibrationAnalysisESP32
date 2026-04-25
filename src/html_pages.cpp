#include <Arduino.h>
#include "html_pages.h"

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
    .bar{width:100%; height:18px}
    .status{margin-top:10px}
    .ok{color:#0a7} .warn{color:#c70} .bad{color:#c00}
  </style>
</head>
<body>
  <div class="card">
    <h1>ESP32 Firmware Update</h1>

    <div class="small">
      Current build:
      <span id="ver" class="mono">loading...</span>
    </div>

    <div class="small" style="margin-top:8px">
      Select the <b>.bin</b> built for this board/partition, then upload.
      Upload completes → device reboots.
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

    <div id="msg" class="status small">Ready.</div>

    <div class="small" style="margin-top:10px">
      During upload do not power off the device.
    </div>
  </div>

<script>
async function loadVersion(){
  try{
    const r = await fetch('/api/version', {cache:'no-store'});
    const j = await r.json();
    const v = `${j.version}  (${j.hash})  built: ${j.built}`;
    document.getElementById('ver').textContent = v;
  }catch(e){
    document.getElementById('ver').textContent = 'unknown';
  }
}

function setMsg(text, cls){
  const el = document.getElementById('msg');
  el.className = 'status small ' + (cls||'');
  el.textContent = text;
}

function setProgress(p){
  const prog = document.getElementById('prog');
  const ptext = document.getElementById('ptext');
  prog.value = p;
  ptext.textContent = `${p.toFixed(0)}%`;
}

function startUpload(){
  const f = document.getElementById('file').files[0];
  if(!f){ alert('Select a .bin file'); return; }

  // UI lock
  document.getElementById('btnUp').disabled = true;
  document.getElementById('file').disabled = true;
  setProgress(0);
  setMsg('Uploading...', 'warn');

  const form = new FormData();
  form.append('update', f, f.name);

  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/update', true);

  xhr.upload.onprogress = (e)=>{
    if(!e.lengthComputable) return;
    const p = (e.loaded / e.total) * 100.0;
    setProgress(p);
  };

  xhr.onload = ()=>{
    // ESP tarafı 200 text/plain dönüyor
    const txt = xhr.responseText || '';
    if(xhr.status === 200){
      setProgress(100);
      setMsg(txt + ' (page will disconnect)', 'ok');
      // reboot sonrası bağlantı kopacak; kullanıcı manuel yeniler
    } else {
      setMsg(`Upload failed: HTTP ${xhr.status} ${txt}`, 'bad');
      document.getElementById('btnUp').disabled = false;
      document.getElementById('file').disabled = false;
    }
  };

  xhr.onerror = ()=>{
    setMsg('Upload error (network).', 'bad');
    document.getElementById('btnUp').disabled = false;
    document.getElementById('file').disabled = false;
  };

  xhr.send(form);
}

loadVersion();
</script>
</body>
</html>

)HTML";

const char LIVE_HTML[] PROGMEM = R"HTML(

<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>accMeter2 - Live View</title>
  <style>
    body{font-family:system-ui,Segoe UI,Roboto,Arial;max-width:1040px;margin:18px auto;padding:0 12px}
    h1{font-size:20px;margin:8px 0 14px}
    nav.top{display:flex;gap:14px;margin:0 0 14px;font-size:14px}
    nav.top a{color:#36c;text-decoration:none;padding:6px 10px;border-radius:8px}
    nav.top a:hover{background:#eef}
    nav.top a.active{background:#36c;color:#fff}
    .row{display:flex;flex-wrap:wrap;gap:12px;align-items:stretch}
    .card{border:1px solid #ddd;border-radius:12px;padding:12px;flex:1;min-width:300px}
    .card h2{font-size:14px;margin:0 0 10px;color:#333}
    label{font-size:12px;color:#444;display:block;margin-bottom:4px}
    select,button{font-size:14px;padding:10px;border-radius:10px;border:1px solid #bbb}
    button{cursor:pointer}
    .btns{display:flex;flex-wrap:wrap;gap:10px}
    .ok{color:#0a7}
    .warn{color:#c70}
    pre{background:#fafafa;border:1px solid #eee;padding:10px;border-radius:10px;overflow:auto;min-height:48px}
    .small{font-size:12px;color:#666}
    .top{margin-bottom:12px}
    .toast{
      position:fixed; right:16px; bottom:16px;
      background:#111; color:#fff; padding:12px 14px;
      border-radius:12px; opacity:0; transform:translateY(10px);
      transition:all .25s ease; pointer-events:none;
      max-width:520px; font-size:13px;
    }
    .toast.show{opacity:0.95; transform:translateY(0)}
  </style>
</head>
<body>
  <h1>accMeter2 <span class="small mono" id="ver">-</span></h1>
  <nav class="top">
    <a href="/">Grab Mode</a>
    <a class="active">Live View</a>
    <a href="/update">Firmware Update</a>
  </nav>

  <div class="card top">
    <h2>Info</h2>
    <div id="status" class="small">Loading...</div>
    <div id="fsinfo" class="small">FS: ...</div>
    <pre id="info">...</pre>

    <div class="btns" style="margin-top:10px">
      <button onclick="goUpdate()">FIRMWARE UPDATE</button>
      <button onclick="doReset()">RESET</button>
    </div>
    <div class="small" style="margin-top:8px">
      Firmware update sırasında kayıt/kalibrasyon yapma.
    </div>
  </div>

  <div class="row">
    <div class="card">
      <h2>Settings</h2>

      <label for="hz">Sampling rate (Hz)</label>
      <select id="hz">
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

      <label for="fs" style="margin-top:10px">G range</label>
      <select id="fs">
        <option value="2" selected>±2 g</option>
        <option value="4">±4 g</option>
        <option value="8">±8 g</option>
        <option value="16">±16 g</option>
      </select>

      <label for="sec" style="margin-top:10px">Record time (s)</label>
      <select id="sec">
        <option value="15">15</option>
        <option value="30">30</option>
        <option value="45">45</option>
        <option value="60" selected>60</option>
        <option value="75">75</option>
        <option value="90">90</option>
        <option value="120">120</option>
      </select>

      <div class="small" style="margin-top:10px">
        Dosya adı browser saatinden alınır: accelYYMMDDHHMMSS.dat
      </div>
    </div>

    <div class="card">
      <h2>Files</h2>

      <label for="fileSel">Select file</label>
      <select id="fileSel"></select>

      <div class="btns" style="margin-top:12px">
        <button onclick="startRec()">START</button>
        <button onclick="stopRec()">STOP</button>
        <button onclick="downloadBin()">DOWNLOAD</button>
        <button onclick="downloadCsv()">DOWNLOAD CSV</button>
        <button onclick="deleteSel()">DELETE</button>
      </div>

      <div class="btns" style="margin-top:12px">
        <button onclick="calibrateStatic()">CALIBRATE (STATIC Z-UP)</button>
        <button onclick="calibrate6()">CALIBRATE (6-POS)</button>
      </div>

      <div class="small" style="margin-top:10px">
        DOWNLOAD / DELETE seçili dosyaya uygulanır.
      </div>
    </div>

    <div class="card">
      <h2>Live RMS (250 ms window @800 Hz target)</h2>
      <canvas id="chart" width="420" height="160" style="width:100%;height:160px;border:1px solid #eee;border-radius:10px;background:#fff"></canvas>
      <div class="small" id="liveRate">rate: -</div>
      <pre id="live">acc: -, vel: -, disp: -</pre>
      <div class="small" style="margin-top:8px">
        <label><input type="checkbox" id="chkRealtime" onchange="toggleRealtime(this.checked)"> Real-time ISO 20816 (mag only)</label>
        <select id="maskBits" onchange="setMaskBits()">
          <option value="0" selected>Mask: off</option>
          <option value="2">Mask: 2-bit</option>
          <option value="3">Mask: 3-bit</option>
          <option value="4">Mask: 4-bit</option>
        </select>
      </div>
      <pre id="rtStats" class="small mono">-</pre>
      <div class="small">Kayit veya kalibrasyon sirasinda live kapalidir.</div>
    </div>

    <div class="card" style="flex-basis:100%">
      <h2>Analysis (selected file)</h2>

      <div class="btns" style="margin-bottom:10px">
        <button onclick="analyzeSelected()">ANALYZE</button>
        <button onclick="clearAnalysis()">CLEAR</button>
      </div>
        <h2>Frequency Domain (FFT)</h2>
        <select id="fftAxis">
        <option value="x">X</option>
        <option value="y">Y</option>
        <option value="z">Z</option>
        </select>
        <button onclick="runFFT()">FFT</button>

        <canvas id="fftChart" width="980" height="320"
        style="width:100%;height:320px;border:1px solid #eee;border-radius:10px"></canvas>

        <pre id="fftInfo">-</pre>


      <div class="small" id="anaMeta">Select a file and press ANALYZE.</div>

      <div class="row" style="gap:12px;margin-top:10px">
        <div class="card" style="min-width:260px;flex:0.9">
          <h2>Stats (g, calibrated)</h2>
          <pre id="stats">-</pre>
        </div>
        <div class="card" style="min-width:420px;flex:2">
          <h2>Chart</h2>
          <canvas id="bigChart" width="980" height="360"
            style="width:100%;height:360px;border:1px solid #eee;border-radius:10px;background:#fff"></canvas>
          <div class="small">Downsample: ~2000 points max (auto).</div>
        </div>
      </div>
    </div>
  </div>

  <div id="toast" class="toast"></div>

<script>
// (Aşağısı senin V3 JS’in aynısı; sadece reset/update fonksiyonları eklendi.)

// ---- Mini chart (canvas) ----
const CHART_N = 60;
const GRAVITY = 9.80665;
let axBuf = new Array(CHART_N).fill(0);
let ayBuf = new Array(CHART_N).fill(0);
let azBuf = new Array(CHART_N).fill(0);
let bufIdx = 0;
let bufCount = 0;
let realtimeMode = false;
let maskBits = 0;

function pushSample(ax, ay, az){
  axBuf[bufIdx] = ax;
  ayBuf[bufIdx] = ay;
  azBuf[bufIdx] = az;
  bufIdx = (bufIdx + 1) % CHART_N;
  bufCount = Math.min(bufCount + 1, CHART_N);
}

function getBuf(buf, i){
  const start = (bufIdx - bufCount + CHART_N) % CHART_N;
  return buf[(start + i) % CHART_N];
}

function drawChart(){
  const c = document.getElementById("chart");
  if(!c) return;
  const ctx = c.getContext("2d");
  const w = c.width, h = c.height;

  ctx.clearRect(0,0,w,h);

  const mL=36, mR=10, mT=10, mB=18;
  const pw = w - mL - mR;
  const ph = h - mT - mB;

  let ymin = +Infinity, ymax = -Infinity;
  for(let i=0;i<bufCount;i++){
    const ax = getBuf(axBuf,i), ay = getBuf(ayBuf,i), az = getBuf(azBuf,i);
    ymin = Math.min(ymin, ax, ay, az);
    ymax = Math.max(ymax, ax, ay, az);
  }
  if(!isFinite(ymin) || !isFinite(ymax)) { ymin=-1; ymax=1; }
  if (ymax - ymin < 0.1) {
    const mid = (ymax + ymin)/2;
    ymin = mid - 0.05; ymax = mid + 0.05;
  }

  const pad = (ymax - ymin) * 0.12;
  ymin -= pad; ymax += pad;

  const xAt = (i)=> mL + (bufCount<=1 ? 0 : (i/(bufCount-1))*pw);
  const yAt = (v)=> mT + ( (ymax - v) / (ymax - ymin) ) * ph;

  ctx.lineWidth = 1;
  ctx.strokeStyle = "#f0f0f0";
  ctx.fillStyle = "#888";
  ctx.font = "11px system-ui,Segoe UI,Roboto,Arial";

  const gridN = 4;
  for(let g=0; g<=gridN; g++){
    const y = mT + (g/gridN)*ph;
    ctx.beginPath();
    ctx.moveTo(mL, y);
    ctx.lineTo(mL+pw, y);
    ctx.stroke();

    const val = (ymax - (g/gridN)*(ymax-ymin));
    ctx.fillText(val.toFixed(2), 4, y+4);
  }

  ctx.strokeStyle = "#e6e6e6";
  ctx.beginPath();
  ctx.rect(mL, mT, pw, ph);
  ctx.stroke();

  function drawSeries(buf, color){
    if(bufCount < 2) return;
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    for(let i=0;i<bufCount;i++){
      const v = getBuf(buf,i);
      const x = xAt(i);
      const y = yAt(v);
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }

  drawSeries(axBuf, "#d33");
  drawSeries(ayBuf, "#3a3");
  drawSeries(azBuf, "#36c");

  ctx.fillStyle = "#111";
  ctx.fillText("ax", mL+6, mT+12);
  ctx.fillStyle = "#3a3";
  ctx.fillText("ay", mL+34, mT+12);
  ctx.fillStyle = "#36c";
  ctx.fillText("az", mL+62, mT+12);

  ctx.fillStyle = "#666";
  ctx.fillText(`${Math.max(0,bufCount-1)}s window`, w-110, h-5);
}

async function getJson(path){
  const r = await fetch(path, {cache:"no-store"});
  if(!r.ok) throw new Error(`HTTP ${r.status}`);
  return await r.json();
}
async function getText(path){
  const r = await fetch(path, {cache:"no-store"});
  const t = await r.text();
  return {ok:r.ok, text:t};
}
async function postText(path, params){
  const body = params ? Object.entries(params).map(([k,v])=>encodeURIComponent(k)+'='+encodeURIComponent(v)).join('&') : '';
  const r = await fetch(path, {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded'}, body, cache:'no-store'});
  const t = await r.text();
  return {ok:r.ok, text:t};
}
function esc(s){ return encodeURIComponent(s); }
function escHtml(s){ return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/"/g,'&quot;'); }

let _toastTimer = null;
function toast(msg){
  const t = document.getElementById("toast");
  t.textContent = msg;
  t.classList.add("show");
  if(_toastTimer) clearTimeout(_toastTimer);
  _toastTimer = setTimeout(()=>{ t.classList.remove("show"); _toastTimer=null; }, 2600);
}

function tsYYMMDDHHMMSS(){
  const d = new Date();
  const yy = String(d.getFullYear()).slice(-2);
  const MM = String(d.getMonth()+1).padStart(2,'0');
  const DD = String(d.getDate()).padStart(2,'0');
  const HH = String(d.getHours()).padStart(2,'0');
  const mm = String(d.getMinutes()).padStart(2,'0');
  const ss = String(d.getSeconds()).padStart(2,'0');
  return `${yy}${MM}${DD}${HH}${mm}${ss}`;
}

function prettyName(filePath){
  let n = filePath || "";
  if (n.startsWith("/")) n = n.slice(1);
  const m = n.match(/^accel(\d{12})(?:_\d{2})?\.dat$/);
  if (!m) return n;
  const ts = m[1];
  const yy = ts.slice(0,2);
  const MM = ts.slice(2,4);
  const DD = ts.slice(4,6);
  const HH = ts.slice(6,8);
  const mm = ts.slice(8,10);
  const ss = ts.slice(10,12);
  return `${n}  [${yy}-${MM}-${DD} ${HH}:${mm}:${ss}]`;
}

let lastRecording = null;
let _inFlight = {info:false, files:false, fs:false, live:false};

async function refreshFiles(selectName=""){
  if(_inFlight.files) return;
  _inFlight.files = true;
  try{
    const files = await getJson("/api/list");
    const sel = document.getElementById("fileSel");
    const current = selectName || sel.value;

    sel.innerHTML = "";
    let keep = "";
    files.sort((a,b)=> (b.name.localeCompare(a.name)));

    for(const f of files){
      const opt = document.createElement("option");
      opt.value = f.name;
      opt.textContent = `${prettyName(f.name)}  (${f.size} B)`;
      sel.appendChild(opt);
      if (f.name === current) keep = current;
    }
    if (keep) sel.value = keep;
  }catch(e){ console.warn("refreshFiles:", e); }
  finally{ _inFlight.files = false; }
}

async function refreshFsInfo(){
  if(_inFlight.fs) return;
  _inFlight.fs = true;
  try{
    const j = await getJson("/api/fsinfo");
    const el = document.getElementById("fsinfo");
    const fmt = (x)=> x < 1024*1024 ? (x/1024).toFixed(1)+" KB" : (x/1024/1024).toFixed(2)+" MB";
    el.textContent = `FS: used ${fmt(j.used)} / total ${fmt(j.total)} (free ${fmt(j.free)})`;
  }catch(e){ console.warn("refreshFsInfo:", e); }
  finally{ _inFlight.fs = false; }
}

async function refreshInfo(){
  if(_inFlight.info) return;
  _inFlight.info = true;
  try{
    const j = await getJson("/api/info");
    const st = document.getElementById("status");

    let flags = [];
    if (j.calibratingStatic) flags.push("CAL(STATIC)");
    if (j.calibrating6) flags.push("CAL(6POS:" + escHtml(j.calibPose) + ")");

    const calBadge = j.calibrated
      ? "<span class='ok'>CAL</span>"
      : "<span class='warn'>UNCAL</span>";

    st.innerHTML =
      (j.recording ? "<span class='warn'>RECORDING</span>" : "<span class='ok'>IDLE</span>")
      + " | " + calBadge
      + " | mode: " + escHtml(j.mode || "-")
      + " | currentFile: " + escHtml(j.currentFile || "-")
      + " | samples: " + j.samples
      + " | elapsed: " + Math.round(j.elapsedMs/1000) + " s"
      + " | maxBacklog: " + j.maxBacklog
      + (flags.length ? (" | <span class='warn'>" + flags.join(" ") + "</span>") : "");

    document.getElementById("info").textContent = JSON.stringify(j, null, 2);

    if (lastRecording === true && j.recording === false) {
      toast(`DONE: ${j.currentFile}  samples=${j.samples}`);
      await refreshFiles(j.currentFile);
      await refreshFsInfo();
    }
    lastRecording = j.recording;
  }catch(e){ console.warn("refreshInfo:", e); }
  finally{ _inFlight.info = false; }
}

function setMaskBits() {
  const bits = document.getElementById("maskBits").value;
  maskBits = parseInt(bits, 10) || 0;
  postText('/api/rawmask', {bits});
}

async function toggleRealtime(on) {
  const r = await postText('/api/realtime', {enable: on ? 1 : 0, mask: maskBits});
  if (!r.ok) {
    alert(r.text);
    document.getElementById("chkRealtime").checked = !on;
    return;
  }
  realtimeMode = on;
  document.getElementById("chart").style.display = on ? "none" : "";
  if (on) {
    document.getElementById("live").textContent = "Realtime mode (preview off)";
  } else {
    document.getElementById("rtStats").textContent = "-";
  }
}

async function refreshLive() {
  if(_inFlight.live) return;
  _inFlight.live = true;
  try{
  const j = await getJson("/api/live");
  if (j.enabled === false) { _inFlight.live=false; return; }
  if (j.realtime) {
    realtimeMode = true;
    document.getElementById("chkRealtime").checked = true;
    document.getElementById("chart").style.display = "none";

    const a1 = j.avg1 || {};
    const a5 = j.avg5 || {};
    const a10 = j.avg10 || {};
    const eff = (typeof j.eff_hz === "number") ? j.eff_hz : 0;
    const dt  = (typeof j.dt_us === "number") ? j.dt_us : 0;
    const txt =
      `RMS (mean-removed, noise-corrected acc):\n` +
      `1s  acc:${(a1.acc_mps2 || 0).toFixed(3)} m/s^2  vel:${(a1.vel_mmps || 0).toFixed(3)} mm/s  disp:${(a1.disp_mm || 0).toFixed(4)} mm\n` +
      `5s  acc:${(a5.acc_mps2 || 0).toFixed(3)} m/s^2  vel:${(a5.vel_mmps || 0).toFixed(3)} mm/s  disp:${(a5.disp_mm || 0).toFixed(4)} mm\n` +
      `10s acc:${(a10.acc_mps2 || 0).toFixed(3)} m/s^2  vel:${(a10.vel_mmps || 0).toFixed(3)} mm/s  disp:${(a10.disp_mm || 0).toFixed(4)} mm\n` +
      `noise floor: ${(j.noise_mps2 || 0).toFixed(4)} m/s^2  rate: ${eff.toFixed(1)} Hz (dt ${dt} us)`;
    document.getElementById("rtStats").textContent = txt;
    return;
  }

  realtimeMode = false;
  document.getElementById("chkRealtime").checked = false;
  document.getElementById("chart").style.display = "";

  if (typeof j.ax !== "number") return;

  const rateEl = document.getElementById("liveRate");
  if (rateEl) {
    const eff = (typeof j.eff_hz === "number") ? j.eff_hz : 0;
    const dt  = (typeof j.dt_us === "number") ? j.dt_us : 0;
    rateEl.textContent = `rate: ${eff.toFixed(1)} Hz  (dt = ${dt} us, target 800 Hz)`;
  }

  const accLine  = `ACC RMS (m/s^2)  X:${j.ax.toFixed(4)}  Y:${j.ay.toFixed(4)}  Z:${j.az.toFixed(4)}  MAG:${j.mag.toFixed(4)}`;
  const velLine  = `VEL RMS (mm/s)   X:${j.vx_mmps.toFixed(3)}  Y:${j.vy_mmps.toFixed(3)}  Z:${j.vz_mmps.toFixed(3)}  MAG:${j.vmag_mmps.toFixed(3)}`;
  const dispLine = `DISP RMS (mm)    X:${j.dx_mm.toFixed(4)}  Y:${j.dy_mm.toFixed(4)}  Z:${j.dz_mm.toFixed(4)}  MAG:${j.dmag_mm.toFixed(4)}`;
  document.getElementById("live").textContent = `${accLine}\n${velLine}\n${dispLine}`;

  const gx = j.ax / GRAVITY;
  const gy = j.ay / GRAVITY;
  const gz = j.az / GRAVITY;
  pushSample(gx, gy, gz);
  drawChart();
  }catch(e){ console.warn("refreshLive:", e); }
  finally{ _inFlight.live = false; }
}

async function startRec(){
  const hz = document.getElementById("hz").value;
  const fs = document.getElementById("fs").value;
  const sec = document.getElementById("sec").value;
  const ts = tsYYMMDDHHMMSS();

  const r = await postText('/api/start', {hz, fs, sec, ts});
  if(!r.ok) alert(r.text);
  else toast("STARTED");

  await refreshInfo();
  await refreshFiles();
  await refreshFsInfo();
}

async function stopRec(){
  const r = await postText('/api/stop');
  if(!r.ok) alert(r.text);
  else toast("STOP requested");

  await refreshInfo();
}

function downloadBin(){
  const sel = document.getElementById("fileSel").value;
  if(!sel){ alert("No file selected"); return; }
  window.location.href = `/download?file=${esc(sel)}`;
}

function downloadCsv(){
  const sel = document.getElementById("fileSel").value;
  if(!sel){ alert("No file selected"); return; }
  window.location.href = `/download_csv?file=${esc(sel)}`;
}

async function deleteSel(){
  const sel = document.getElementById("fileSel").value;
  if(!sel){ alert("No file selected"); return; }
  const r = await postText('/api/delete', {file: sel});
  if(!r.ok) alert(r.text);
  else toast(`Deleted: ${sel}`);

  await refreshFiles();
  await refreshFsInfo();
}

async function calibrateStatic(){
  if(!confirm("STATIC CALIBRATION\nDevice still, +Z UP.\nStart?")) return;
  const r = await postText('/api/calibrate_static');
  if(!r.ok) alert(r.text);
  else toast("Static calibration started");
}

async function calibrate6(){
  alert(
`6-POS Calibration Wizard
Follow steps: X+, X-, Y+, Y-, Z+, Z-
Keep device still at each step.
(You can watch calibPose in Info)`
  );
  const r = await postText('/api/calibrate6');
  if(!r.ok) alert(r.text);
  else toast("6-POS calibration started");
}

// -------- NEW: reset & firmware update ----------
function goUpdate(){
  // no need to fetch; open upload page
  window.location.href = "/update";
}
async function doReset(){
  if(!confirm("Device will reboot now.\nContinue?")) return;
  toast("Rebooting...");
  await postText('/api/reset');
  // page will drop; user refresh after reconnect
}

// (Analysis JS burada devam ediyor; senin V3 içeriğini aynen bırakabilirsin.)
// Senin mevcut V3 analysis kodu bu dosyada var sayılıyor.

async function runFFT(){
  const file = document.getElementById("fileSel").value;
  const axis = document.getElementById("fftAxis").value;
  if(!file) return alert("Select file");

  try{
    const j = await getJson(`/api/fft?file=${esc(file)}&axis=${axis}`);
    drawFFT(j.fft, j.df);
    document.getElementById("fftInfo").textContent =
      `Axis: ${j.axis}\nPeak: ${j.peak_hz.toFixed(2)} Hz\nMagnitude: ${j.peak_mag.toFixed(4)}`;
  }catch(e){
    console.error(e);
    alert("FFT failed: " + (e?.message || e));
  }
}

function drawFFT(arr, df){
    const c = document.getElementById("fftChart");
    const ctx = c.getContext("2d");
    ctx.clearRect(0,0,c.width,c.height);
    if(!arr || arr.length < 2 || !df) return;

    const w=c.width,h=c.height;
    const mL=50,mR=10,mT=10,mB=30;
    const pw=w-mL-mR,ph=h-mT-mB;

    let max=Math.max(...arr);
    if(max<=0) max=1;

    ctx.strokeStyle="#ccc";
    ctx.strokeRect(mL,mT,pw,ph);

    ctx.beginPath();
    ctx.strokeStyle="#36c";
    const div = Math.max(1, arr.length-1);
    for(let i=0;i<arr.length;i++){
        const x=mL+(i/div)*pw;
        const y=mT+(1-arr[i]/max)*ph;
        if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();

    ctx.fillStyle="#666";
    ctx.fillText(`0 Hz`,mL,h-8);
    ctx.fillText(`${(arr.length*df).toFixed(1)} Hz`,w-80,h-8);
    }


function clearAnalysis(){
  document.getElementById("stats").textContent = "-";
  document.getElementById("anaMeta").textContent = "Select a file and press ANALYZE.";
  const c=document.getElementById("bigChart");
  const ctx=c.getContext("2d");
  ctx.clearRect(0,0,c.width,c.height);
}

function drawBigChart(ax, ay, az, rateHz){
  const c = document.getElementById("bigChart");
  if(!c) return;
  const ctx = c.getContext("2d");

  const w = c.width, h = c.height;
  ctx.clearRect(0,0,w,h);

  const mL=50, mR=10, mT=10, mB=30;
  const pw=w-mL-mR, ph=h-mT-mB;

  let ymin=Infinity, ymax=-Infinity;
  [...ax,...ay,...az].forEach(v=>{
    ymin=Math.min(ymin,v);
    ymax=Math.max(ymax,v);
  });
  if(!isFinite(ymin)||!isFinite(ymax)){ ymin=-1; ymax=1; }
  if(ymax-ymin<0.01){
    const m=(ymax+ymin)/2;
    ymin=m-0.01; ymax=m+0.01;
  }

  const xAt=(i,n)=>mL+(i/(n-1))*pw;
  const yAt=v=>mT+((ymax-v)/(ymax-ymin))*ph;

  // grid
  ctx.strokeStyle="#eee";
  ctx.fillStyle="#666";
  ctx.font="11px system-ui";
  for(let i=0;i<=4;i++){
    const y=mT+(i/4)*ph;
    ctx.beginPath(); ctx.moveTo(mL,y); ctx.lineTo(mL+pw,y); ctx.stroke();
    const val=(ymax-(i/4)*(ymax-ymin));
    ctx.fillText(val.toFixed(3),4,y+4);
  }

  // frame
  ctx.strokeStyle="#ccc";
  ctx.strokeRect(mL,mT,pw,ph);

  function plot(arr,color){
    if(arr.length<2) return;
    ctx.strokeStyle=color;
    ctx.lineWidth=2;
    ctx.beginPath();
    for(let i=0;i<arr.length;i++){
      const x=xAt(i,arr.length);
      const y=yAt(arr[i]);
      if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
    }
    ctx.stroke();
  }

  plot(ax,"#d33"); // X
  plot(ay,"#2a2"); // Y
  plot(az,"#26c"); // Z

  ctx.fillStyle="#000";
  ctx.fillText("X",mL+10,mT+12);
  ctx.fillStyle="#2a2";
  ctx.fillText("Y",mL+30,mT+12);
  ctx.fillStyle="#26c";
  ctx.fillText("Z",mL+50,mT+12);

  ctx.fillStyle="#666";
  ctx.fillText(`${(ax.length/rateHz).toFixed(2)} s`, w-70, h-8);
}




async function analyzeSelected(){
  const file = document.getElementById("fileSel").value;
  if(!file){ alert("No file selected"); return; }

  try{
    toast("Analyzing on device...");
    document.getElementById("anaMeta").textContent = "Analyzing on device...";

    const j = await getJson(`/api/analyze?file=${esc(file)}`);

    // chart
    drawBigChart(j.ax, j.ay, j.az, j.eff_hz || j.rate_hz || 1);

    // stats text
    const stats =
`File: ${j.file}
rate_hz: ${j.rate_hz}
record_s: ${j.record_s}
samples(header): ${j.samples_header}
samples(used): ${j.samples_used}
fs_g: ±${j.fs_g}g
res_bits: ${j.res_bits}
q_bits: ${j.q_bits}

X: min=${j.min[0].toFixed(4)} g  max=${j.max[0].toFixed(4)} g  rms=${j.rms[0].toFixed(4)} g
Y: min=${j.min[1].toFixed(4)} g  max=${j.max[1].toFixed(4)} g  rms=${j.rms[1].toFixed(4)} g
Z: min=${j.min[2].toFixed(4)} g  max=${j.max[2].toFixed(4)} g  rms=${j.rms[2].toFixed(4)} g`;

    document.getElementById("stats").textContent = stats;
    document.getElementById("anaMeta").textContent =
      `OK. Rendered ${j.pts} points (downsampled on ESP32).`;

    toast("Analysis done.");
  }catch(e){
    console.error(e);
    alert("ANALYZE failed: " + (e?.message || e));
    document.getElementById("anaMeta").textContent = "ANALYZE failed.";
  }
}


async function loadVersion(){
  try{
    const j = await getJson('/api/version');
    document.getElementById('ver').textContent = j.version + ' ' + j.hash;
  }catch(e){}
}

setInterval(refreshInfo, 1000);
setInterval(()=>refreshFiles(), 2000);
setInterval(refreshFsInfo, 3000);
setInterval(refreshLive, 1000);

loadVersion();
refreshInfo(); refreshFiles(); refreshFsInfo(); refreshLive(); drawChart();
</script>
</body>
</html>

)HTML";


// ============================================================================
// GRAB MODE PAGE — default landing. Captures measurement metadata, configures
// trigger thresholds, arms the device for motion-triggered recording.
// ============================================================================
const char GRAB_HTML[] PROGMEM = R"HTML(

<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>accMeter2 - Grab Mode</title>
  <style>
    body{font-family:system-ui,Segoe UI,Roboto,Arial;max-width:920px;margin:18px auto;padding:0 12px;color:#111}
    h1{font-size:20px;margin:8px 0 14px}
    h2{font-size:14px;margin:0 0 10px;color:#333}
    nav.top{display:flex;gap:14px;margin:0 0 14px;font-size:14px}
    nav.top a{color:#36c;text-decoration:none;padding:6px 10px;border-radius:8px}
    nav.top a:hover{background:#eef}
    nav.top a.active{background:#36c;color:#fff}
    .card{border:1px solid #ddd;border-radius:12px;padding:14px;margin-bottom:12px;background:#fff}
    label{font-size:12px;color:#444;display:block;margin-bottom:4px}
    input,select,button{font-size:14px;padding:8px 10px;border-radius:10px;border:1px solid #bbb;background:#fff}
    button{cursor:pointer}
    button.primary{background:#36c;color:#fff;border-color:#36c;font-weight:600;padding:10px 18px}
    button.primary:disabled{background:#aaa;border-color:#aaa;cursor:not-allowed}
    button.danger{background:#c33;color:#fff;border-color:#c33}
    .grid{display:grid;grid-template-columns:160px 1fr;gap:8px 12px;align-items:center}
    .grid input,.grid select{width:100%;box-sizing:border-box}
    .row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
    .small{font-size:12px;color:#666}
    .mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
    .ok{color:#0a7;font-weight:600}
    .warn{color:#c70;font-weight:600}
    .bad{color:#c00;font-weight:600}
    .badge{padding:2px 8px;border-radius:8px;font-size:11px;font-weight:600}
    .badge.cal{background:#e7f7ee;color:#0a7;border:1px solid #0a7}
    .badge.uncal{background:#fff4e0;color:#c70;border:1px solid #c70}
    .stateBig{font-size:22px;font-weight:700;letter-spacing:0.5px}
    .stateIdle{color:#666}
    .stateArmed{color:#c70}
    .stateTrig{color:#c33}
    .statePost{color:#36c}
    .stateDone{color:#0a7}
    pre{background:#fafafa;border:1px solid #eee;padding:10px;border-radius:10px;font-size:12px;margin:8px 0 0}
    .req::after{content:" *";color:#c33}
    .toast{position:fixed;right:16px;bottom:16px;background:#111;color:#fff;padding:12px 14px;border-radius:12px;opacity:0;transform:translateY(10px);transition:all .25s ease;pointer-events:none;max-width:520px;font-size:13px}
    .toast.show{opacity:0.95;transform:translateY(0)}
    .row-thr{display:flex;gap:14px;flex-wrap:wrap;align-items:center;font-size:13px}
    .row-thr input[type=number]{width:80px}
    @media(max-width:600px){.grid{grid-template-columns:1fr}}
  </style>
</head>
<body>
  <h1>accMeter2 <span class="small mono" id="ver">-</span></h1>
  <nav class="top">
    <a class="active">Grab Mode</a>
    <a href="/live">Live View</a>
    <a href="/update">Firmware Update</a>
  </nav>

  <div class="card">
    <h2>Device</h2>
    <div class="grid">
      <label>Serial</label>
      <span id="serial" class="mono small">-</span>

      <label>Name</label>
      <span class="row">
        <input id="devName" placeholder="accMeter-XXXXXX" maxlength="23" style="flex:1;min-width:160px"/>
        <button onclick="saveName()">SAVE</button>
      </span>

      <label>Calibration</label>
      <span id="calBadge" class="badge uncal">UNCAL</span>
    </div>
  </div>

  <div class="card">
    <h2>Measurement Metadata</h2>
    <div class="grid">
      <label>Date / Time</label>
      <span id="ts" class="mono small">auto from browser</span>

      <label class="req">Measurement point</label>
      <input id="measPoint" placeholder="e.g. Motor-DE" maxlength="23" required/>

      <label class="req">Scan direction</label>
      <select id="scanDir">
        <option value="RADIAL_H">Radial - Horizontal</option>
        <option value="RADIAL_V">Radial - Vertical</option>
        <option value="AXIAL">Axial</option>
      </select>

      <label>Operator</label>
      <input id="operator" placeholder="(optional)" maxlength="15"/>

      <label>Notes</label>
      <input id="notes" placeholder="(optional)" maxlength="63"/>
    </div>
  </div>

  <div class="card">
    <h2>Acquisition Settings</h2>
    <div class="grid">
      <label>Sampling rate</label>
      <select id="hz">
        <option value="100">100 Hz</option>
        <option value="200">200 Hz</option>
        <option value="400">400 Hz</option>
        <option value="800" selected>800 Hz</option>
        <option value="1600">1600 Hz</option>
      </select>

      <label>G range</label>
      <select id="fs">
        <option value="2" selected>+/- 2 g</option>
        <option value="4">+/- 4 g</option>
        <option value="8">+/- 8 g</option>
        <option value="16">+/- 16 g</option>
      </select>

      <label>Max duration</label>
      <select id="maxS">
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
      <span class="row">
        <input id="preS" type="number" min="1" max="10" value="3" style="width:80px"/> s
      </span>
      <label>Post-roll</label>
      <span class="row">
        <input id="postS" type="number" min="1" max="30" value="5" style="width:80px"/> s
      </span>
      <label>Threshold</label>
      <div>
        <div class="row-thr">
          <label><input type="radio" name="thrMode" value="0" checked onchange="updateThrMode()"/>
          Auto baseline x <input id="mult" type="number" min="2" max="20" value="3"/></label>
        </div>
        <div class="row-thr" style="margin-top:6px">
          <label><input type="radio" name="thrMode" value="1" onchange="updateThrMode()"/>
          Manual: <input id="manThr" type="number" min="0.01" max="100" step="0.01" value="0.50"/> m/s^2</label>
        </div>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="row" style="gap:14px">
      <button id="armBtn" class="primary" onclick="armGrab()">START GRAB (ARM)</button>
      <button id="disarmBtn" class="danger" onclick="disarmGrab()" disabled>DISARM</button>
    </div>
  </div>

  <div class="card">
    <h2>Status</h2>
    <div id="stateLabel" class="stateBig stateIdle">IDLE</div>
    <pre id="stateDetail">Fill in the metadata above and press ARM to start.</pre>
  </div>

  <div class="card" id="lastFileCard" style="display:none">
    <h2>Last saved</h2>
    <div id="lastFile" class="mono small"></div>
    <div style="margin-top:8px"><a href="/live">Open Live View to analyze -&gt;</a></div>
  </div>

  <div id="toast" class="toast"></div>

<script>
async function getJson(p){
  const r = await fetch(p, {cache:'no-store'});
  if(!r.ok) throw new Error('HTTP ' + r.status);
  return await r.json();
}
async function postText(p, params){
  const body = params ? Object.entries(params)
    .map(([k,v]) => encodeURIComponent(k)+'='+encodeURIComponent(v))
    .join('&') : '';
  const r = await fetch(p, {
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body, cache:'no-store'
  });
  return {ok:r.ok, text: await r.text()};
}
let _toastTimer = null;
function toast(msg){
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.classList.add('show');
  if(_toastTimer) clearTimeout(_toastTimer);
  _toastTimer = setTimeout(()=>{t.classList.remove('show'); _toastTimer=null;}, 2600);
}

function tsYYMMDDHHMMSS(){
  const d = new Date();
  const yy = String(d.getFullYear()).slice(-2);
  const MM = String(d.getMonth()+1).padStart(2,'0');
  const DD = String(d.getDate()).padStart(2,'0');
  const HH = String(d.getHours()).padStart(2,'0');
  const mm = String(d.getMinutes()).padStart(2,'0');
  const ss = String(d.getSeconds()).padStart(2,'0');
  return yy+MM+DD+HH+mm+ss;
}
function nowPretty(){
  const d = new Date();
  return d.toLocaleString();
}

async function loadVersion(){
  try{
    const j = await getJson('/api/version');
    document.getElementById('ver').textContent = j.version + ' ' + j.hash;
  }catch(e){}
}

let _devLoaded = false;
async function loadDevice(){
  try{
    const j = await getJson('/api/device');
    document.getElementById('serial').textContent = j.serial || '-';
    if(!_devLoaded){
      document.getElementById('devName').value = j.name || '';
      _devLoaded = true;
    }
  }catch(e){ console.warn('loadDevice', e); }
}
async function saveName(){
  const name = document.getElementById('devName').value.trim();
  if(!name){ alert('Name is required'); return; }
  if(name.length > 23){ alert('Max 23 chars'); return; }
  const r = await postText('/api/device', {name});
  if(!r.ok){ alert('Save failed: ' + r.text); return; }
  toast('Device name saved');
}

function updateThrMode(){
  const mode = document.querySelector('input[name=thrMode]:checked').value;
  document.getElementById('mult').disabled   = (mode === '1');
  document.getElementById('manThr').disabled = (mode === '0');
}

function setStateUI(state, j){
  const lbl = document.getElementById('stateLabel');
  const det = document.getElementById('stateDetail');
  lbl.classList.remove('stateIdle','stateArmed','stateTrig','statePost','stateDone');
  let txt = '';
  switch(state){
    case 'Idle':
      lbl.textContent = 'IDLE';
      lbl.classList.add('stateIdle');
      txt = 'Fill in the metadata above and press ARM to start.';
      break;
    case 'Armed': {
      lbl.textContent = 'ARMED — waiting for motion';
      lbl.classList.add('stateArmed');
      const armed = ((j.trigArmedMs||0)/1000).toFixed(1);
      const baseline = (j.trigBaseline||0).toFixed(4);
      const thr = (j.trigThreshold||0).toFixed(4);
      const cur = (j.trigCurrentRms||0).toFixed(4);
      txt = `armed for ${armed} s\nbaseline: ${baseline} m/s^2\nthreshold: ${thr} m/s^2\ncurrent window RMS: ${cur} m/s^2`;
      break;
    }
    case 'Triggered': {
      lbl.textContent = 'TRIGGERED — recording';
      lbl.classList.add('stateTrig');
      const fired = ((j.trigFiredMs||0)/1000).toFixed(1);
      const cur = (j.trigCurrentRms||0).toFixed(4);
      const thr = (j.trigThreshold||0).toFixed(4);
      const samples = j.samples || 0;
      txt = `recording for ${fired} s\nsamples written: ${samples}\ncurrent RMS: ${cur} m/s^2 (threshold ${thr})`;
      break;
    }
    case 'PostTail': {
      lbl.textContent = 'POST-TAIL — finalizing';
      lbl.classList.add('statePost');
      const fired = ((j.trigFiredMs||0)/1000).toFixed(1);
      const samples = j.samples || 0;
      txt = `total ${fired} s elapsed since trigger\nsamples written: ${samples}\nfinishing tail recording...`;
      break;
    }
  }
  det.textContent = txt;
}

let _lastTrigState = 'Idle';
let _lastFile = '';
async function refreshInfo(){
  try{
    const j = await getJson('/api/info');
    const state = j.trigState || 'Idle';
    setStateUI(state, j);

    document.getElementById('calBadge').textContent = j.calibrated ? 'CAL' : 'UNCAL';
    document.getElementById('calBadge').className   = 'badge ' + (j.calibrated ? 'cal' : 'uncal');

    document.getElementById('armBtn').disabled    = (state !== 'Idle');
    document.getElementById('disarmBtn').disabled = (state === 'Idle');

    // Detect transition Triggered/PostTail -> Idle (capture finished)
    if(_lastTrigState !== 'Idle' && state === 'Idle' && j.currentFile){
      document.getElementById('lastFileCard').style.display = '';
      document.getElementById('lastFile').textContent =
        `${j.currentFile}  (samples=${j.samples})`;
      toast('Saved: ' + j.currentFile);
    }
    _lastTrigState = state;

    // Update timestamp display while idle
    if(state === 'Idle'){
      document.getElementById('ts').textContent = nowPretty() + ' (will be auto-set on ARM)';
    }
  }catch(e){ console.warn('refreshInfo', e); }
}

async function armGrab(){
  const measPoint = document.getElementById('measPoint').value.trim();
  const scanDir   = document.getElementById('scanDir').value;
  if(!measPoint){ alert('Measurement point is required'); return; }
  if(!scanDir){   alert('Scan direction is required');    return; }

  const params = {
    hz:    document.getElementById('hz').value,
    fs:    document.getElementById('fs').value,
    max_s: document.getElementById('maxS').value,
    pre_s: document.getElementById('preS').value,
    post_s:document.getElementById('postS').value,
    mode:  document.querySelector('input[name=thrMode]:checked').value,
    mult:  document.getElementById('mult').value,
    abs_thr: document.getElementById('manThr').value,
    ts:    tsYYMMDDHHMMSS(),
    meas_point: measPoint,
    scan_dir:   scanDir,
    operator:   document.getElementById('operator').value.trim(),
    notes:      document.getElementById('notes').value.trim()
  };

  const r = await postText('/api/trigger_arm', params);
  if(!r.ok){ alert('Arm failed: ' + r.text); return; }
  document.getElementById('lastFileCard').style.display = 'none';
  toast('ARMED');
}

async function disarmGrab(){
  if(!confirm('Disarm? Active recording (if any) will close cleanly.')) return;
  const r = await postText('/api/trigger_disarm');
  if(!r.ok){ alert('Disarm failed: ' + r.text); return; }
  toast('Disarm requested');
}

setInterval(refreshInfo, 1000);
loadVersion();
loadDevice();
refreshInfo();
updateThrMode();
</script>
</body>
</html>

)HTML";
