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

const char INDEX_HTML[] PROGMEM = R"HTML(

<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width, initial-scale=1"/>
  <title>ESP32 LIS2DW12 Recorder</title>
  <style>
    body{font-family:system-ui,Segoe UI,Roboto,Arial;max-width:1040px;margin:18px auto;padding:0 12px}
    h1{font-size:20px;margin:8px 0 14px}
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
  <h1>ESP32 LIS2DW12 Recorder</h1>

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
        <option value="180">180</option>
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
      <h2>Live (1s preview @800 Hz)</h2>
      <canvas id="chart" width="420" height="160" style="width:100%;height:160px;border:1px solid #eee;border-radius:10px;background:#fff"></canvas>
      <pre id="live">acc: -, vel: -, disp: -</pre>
      <div class="small">Kayıt veya kalibrasyon sırasında live kapalıdır.</div>
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
  return await r.json();
}
async function getText(path){
  const r = await fetch(path, {cache:"no-store"});
  const t = await r.text();
  return {ok:r.ok, text:t};
}
function esc(s){ return encodeURIComponent(s); }

function toast(msg){
  const t = document.getElementById("toast");
  t.textContent = msg;
  t.classList.add("show");
  setTimeout(()=>t.classList.remove("show"), 2600);
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

async function refreshFiles(selectName=""){
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
}

async function refreshFsInfo(){
  const j = await getJson("/api/fsinfo");
  const el = document.getElementById("fsinfo");
  const fmt = (x)=> x < 1024*1024 ? (x/1024).toFixed(1)+" KB" : (x/1024/1024).toFixed(2)+" MB";
  el.textContent = `FS: used ${fmt(j.used)} / total ${fmt(j.total)} (free ${fmt(j.free)})`;
}

async function refreshInfo(){
  const j = await getJson("/api/info");
  const st = document.getElementById("status");

  let flags = [];
  if (j.calibratingStatic) flags.push("CAL(STATIC)");
  if (j.calibrating6) flags.push("CAL(6POS:"+j.calibPose+")");

  st.innerHTML =
    (j.recording ? "<span class='warn'>RECORDING</span>" : "<span class='ok'>IDLE</span>")
    + " | mode: " + (j.mode || "-")
    + " | currentFile: " + (j.currentFile || "-")
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
}

async function refreshLive(){
  const j = await getJson("/api/live");
  if(j.enabled === false) return;
  if(typeof j.ax !== "number") return;

  const accLine = `ACC (m/s²)  X:${j.ax.toFixed(3)}  Y:${j.ay.toFixed(3)}  Z:${j.az.toFixed(3)}  MAG:${j.mag.toFixed(3)}`;
  const velLine = `VEL (mm/s)  X:${j.vx_mmps.toFixed(2)}  Y:${j.vy_mmps.toFixed(2)}  Z:${j.vz_mmps.toFixed(2)}  MAG:${j.vmag_mmps.toFixed(2)}`;
  const dispLine = `DISP (mm)   X:${j.dx_mm.toFixed(2)}  Y:${j.dy_mm.toFixed(2)}  Z:${j.dz_mm.toFixed(2)}  MAG:${j.dmag_mm.toFixed(2)}`;
  document.getElementById("live").textContent = `${accLine}\n${velLine}\n${dispLine}`;

  const gx = j.ax / GRAVITY;
  const gy = j.ay / GRAVITY;
  const gz = j.az / GRAVITY;
  pushSample(gx, gy, gz);
  drawChart();
}

async function startRec(){
  const hz = document.getElementById("hz").value;
  const fs = document.getElementById("fs").value;
  const sec = document.getElementById("sec").value;
  const ts = tsYYMMDDHHMMSS();

  const r = await getText(`/api/start?hz=${esc(hz)}&fs=${esc(fs)}&sec=${esc(sec)}&ts=${esc(ts)}`);
  if(!r.ok) alert(r.text);
  else toast("STARTED");

  await refreshInfo();
  await refreshFiles();
  await refreshFsInfo();
}

async function stopRec(){
  const r = await getText("/api/stop");
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
  const r = await getText(`/api/delete?file=${esc(sel)}`);
  if(!r.ok) alert(r.text);
  else toast(`Deleted: ${sel}`);

  await refreshFiles();
  await refreshFsInfo();
}

async function calibrateStatic(){
  if(!confirm("STATIC CALIBRATION\nDevice still, +Z UP.\nStart?")) return;
  const r = await getText("/api/calibrate_static");
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
  const r = await getText("/api/calibrate6");
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
  await getText("/api/reset");
  // page will drop; user refresh after reconnect
}

// (Analysis JS burada devam ediyor; senin V3 içeriğini aynen bırakabilirsin.)
// Senin mevcut V3 analysis kodu bu dosyada var sayılıyor.

async function runFFT(){
  const file = document.getElementById("fileSel").value;
  const axis = document.getElementById("fftAxis").value;
  if(!file) return alert("Select file");

  const j = await getJson(`/api/fft?file=${esc(file)}&axis=${axis}`);
  drawFFT(j.fft, j.df);
  document.getElementById("fftInfo").textContent =
    `Axis: ${j.axis}\nPeak: ${j.peak_hz.toFixed(2)} Hz\nMagnitude: ${j.peak_mag.toFixed(4)}`;
}

function drawFFT(arr, df){
    const c = document.getElementById("fftChart");
    const ctx = c.getContext("2d");
    ctx.clearRect(0,0,c.width,c.height);

    const w=c.width,h=c.height;
    const mL=50,mR=10,mT=10,mB=30;
    const pw=w-mL-mR,ph=h-mT-mB;

    let max=Math.max(...arr);
    if(max<=0) max=1;

    ctx.strokeStyle="#ccc";
    ctx.strokeRect(mL,mT,pw,ph);

    ctx.beginPath();
    ctx.strokeStyle="#36c";
    for(let i=0;i<arr.length;i++){
        const x=mL+(i/(arr.length-1))*pw;
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


setInterval(refreshInfo, 1000);
setInterval(()=>refreshFiles(), 2000);
setInterval(refreshFsInfo, 3000);
setInterval(refreshLive, 1000);

refreshInfo(); refreshFiles(); refreshFsInfo(); refreshLive(); drawChart();
</script>
</body>
</html>

)HTML";
