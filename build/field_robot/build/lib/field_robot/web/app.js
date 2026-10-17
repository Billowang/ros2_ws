/* =====================================================================
   資料流（與 8 節點架構一致，網頁只做這幾件事，且「有變化才送」）：
     送出  mode    自動/手動切換            （切頁時 1 次）
     送出  start   地圖型號 + 開始任務      （按鈕 1 次）
     送出  stop    停止任務                 （按鈕 1 次）
     送出  manual  左右輪轉速 rpm           （值改變時；非零時每 0.5 s 心跳）
     送出  estop   急停                     （按鈕 1 次）
     收到  hello         連線時 1 次（目前任務是否進行中）
     收到  mission_done  任務完成時 1 次
   ===================================================================== */
const WS_URL = (location.protocol === "https:" ? "wss://" : "ws://") + location.hostname + ":8765";
const MAX_RPM = 300;          // throttle 100% 對應的輪速 (rpm)，請依實機調整
const STEER_GAIN = 0.5;       // steer 100% 造成的左右輪差速比例
const TICK_MS = 50;           // 檢查「手動輪速是否改變」的間隔
const HEARTBEAT_MS = 500;     // 非零輪速時的心跳（伺服器 1 s 沒收到會自動歸零）
const STEP = 4, REPEAT_MS = 60;

const state = { throttle: 0, steer: 0, lock: true, estop: false, cruise: false,
                mode: "auto", mission: "idle" };   // mission: idle | running | done

const $ = (id) => document.getElementById(id);
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const lockedOut = () => state.lock || state.estop;

/* ---------- 時鐘 ---------- */
function tickClock(){ $("clock").textContent = new Date().toTimeString().slice(0,5); }
tickClock(); setInterval(tickClock, 15000);

/* ---------- WebSocket ---------- */
let ws = null, wsReady = false;
let lastSent = { l: NaN, r: NaN }, lastSentAt = 0;

function send(obj){
  if(!wsReady) return false;
  ws.send(JSON.stringify(obj));
  return true;
}
function connect(){
  ws = new WebSocket(WS_URL);
  ws.onopen = () => {
    wsReady = true;
    $("conn-dot").classList.add("live");
    $("conn-label").textContent = "已連線";
    $("connStatus").textContent = "良好";
    send({ type: "mode", value: state.mode });       // 重新同步模式
    lastSent = { l: NaN, r: NaN };                   // 讓手動輪速重送一次
  };
  ws.onclose = () => {
    wsReady = false;
    $("conn-dot").classList.remove("live");
    $("conn-label").textContent = "已斷線 - 重連中…";
    $("connStatus").textContent = "中斷";
    setTimeout(connect, 1500);
  };
  ws.onerror = () => ws.close();
  ws.onmessage = (evt) => {
    let m; try{ m = JSON.parse(evt.data); }catch{ return; }
    if(m.type === "hello"){
      if(m.running) setMission("running");
      else if(state.mission === "running") setMission("idle");
    } else if(m.type === "mission_done"){
      setMission("done");
    }
  };
}
connect();

/* =====================================================================
   兩頁切換：頁 1 = 自動模式、頁 2 = 手動模式（切頁即送 mode）
   ===================================================================== */
const pageTitles = ["自動整備模式", "手動控制"];
let currentPage = 0;

function goToPage(idx){
  currentPage = clamp(idx, 0, 1);
  $("pagesTrack").style.transform = `translateX(-${currentPage * 50}%)`;
  $("pageTitle").textContent = pageTitles[currentPage];
  $("prevPage").disabled = currentPage === 0;
  $("nextPage").disabled = currentPage === 1;
  setMode(currentPage === 1 ? "manual" : "auto");
}
$("prevPage").addEventListener("click", () => goToPage(currentPage - 1));
$("nextPage").addEventListener("click", () => goToPage(currentPage + 1));

(function enableSwipe(){
  let startX = null;
  const vp = document.querySelector(".pages-viewport");
  vp.addEventListener("pointerdown", (e) => {
    if(e.target.closest(".vhandle, .hhandle, .arrow, .lock-btn, .estop-btn, .cruise-btn, button, select, input")) return;
    startX = e.clientX;
  });
  vp.addEventListener("pointerup", (e) => {
    if(startX === null) return;
    const dx = e.clientX - startX; startX = null;
    if(dx > 60) goToPage(currentPage - 1); else if(dx < -60) goToPage(currentPage + 1);
  });
})();

function setMode(mode){
  if(state.mode === mode) return;
  state.mode = mode;
  if(mode === "manual"){
    if(state.mission === "running"){ send({ type: "stop" }); setMission("idle"); }
    send({ type: "mode", value: "manual" });
    lastSent = { l: 0, r: 0 }; lastSentAt = Date.now();   // 控制節點切模式時已把輪速歸零
  } else {
    setThrottle(0); setSteer(0);
    state.lock = true; state.cruise = false; setLockUI(); setCruiseUI();
    send({ type: "mode", value: "auto" });
  }
  $("modeStatus").textContent = mode === "auto" ? "自動" : "手動";
}

/* =====================================================================
   手動控制：滑桿 -> 左右輪 rpm（有變化才送）
   ===================================================================== */
function wheelCmd(){
  if(lockedOut()) return { l: 0, r: 0 };
  const l = clamp(state.throttle + state.steer * STEER_GAIN, -100, 100);
  const r = clamp(state.throttle - state.steer * STEER_GAIN, -100, 100);
  return { l: Math.round(l * MAX_RPM / 100), r: Math.round(r * MAX_RPM / 100) };
}

function renderCmd(){
  const { l, r } = wheelCmd();
  $("leftRpm").textContent = l;  $("rightRpm").textContent = r;
  $("leftBar").style.width  = clamp(Math.abs(l) / MAX_RPM * 100, 0, 100) + "%";
  $("rightBar").style.width = clamp(Math.abs(r) / MAX_RPM * 100, 0, 100) + "%";
  $("wheelRpm").textContent = `L ${l}   R ${r}`;
}

setInterval(() => {
  if(state.mode !== "manual") return;
  const { l, r } = wheelCmd();
  const changed = l !== lastSent.l || r !== lastSent.r;
  const beat = (l !== 0 || r !== 0) && Date.now() - lastSentAt >= HEARTBEAT_MS;
  if((changed || beat) && send({ type: "manual", left: l, right: r })){
    lastSent = { l, r }; lastSentAt = Date.now();
  }
}, TICK_MS);

/* ---------- 滑桿顯示 ---------- */
const throttleTrack = $("throttleTrack"), steerTrack = $("steerTrack");
function renderThrottle(){
  $("throttleVal").textContent = state.throttle;
  const pct = Math.abs(state.throttle) / 2;
  $("throttleFill").style.height = pct + "%";
  $("throttleFill").style.bottom = state.throttle >= 0 ? "50%" : (50 - pct) + "%";
  $("throttleHandle").style.bottom = `calc(50% + ${(state.throttle / 100) * (throttleTrack.clientHeight / 2)}px)`;
}
function renderSteer(){
  $("steerVal").textContent = state.steer;
  const pct = Math.abs(state.steer) / 2;
  $("steerFill").style.width = pct + "%";
  $("steerFill").style.left = state.steer >= 0 ? "50%" : (50 - pct) + "%";
  $("steerHandle").style.left = `calc(50% + ${(state.steer / 100) * (steerTrack.clientWidth / 2)}px)`;
}
function setThrottle(v){ state.throttle = clamp(Math.round(v), -100, 100); renderThrottle(); renderCmd(); }
function setSteer(v){ state.steer = clamp(Math.round(v), -100, 100); renderSteer(); renderCmd(); }

/* ---------- 箭頭按鈕：按住連續增量 ---------- */
function holdRepeat(btn, fn){
  let iv = null;
  const stop = () => { clearInterval(iv); iv = null; };
  btn.addEventListener("pointerdown", (e) => { e.preventDefault(); e.stopPropagation(); fn(); iv = setInterval(fn, REPEAT_MS); });
  ["pointerup", "pointerleave", "pointercancel"].forEach(t => btn.addEventListener(t, stop));
}
holdRepeat($("throttleUp"),   () => { if(!lockedOut()) setThrottle(state.throttle + STEP); });
holdRepeat($("throttleDown"), () => { if(!lockedOut()) setThrottle(state.throttle - STEP); });
holdRepeat($("steerRight"),   () => { if(!lockedOut()) setSteer(state.steer + STEP); });
holdRepeat($("steerLeft"),    () => { if(!lockedOut()) setSteer(state.steer - STEP); });

/* ---------- 拖曳滑桿 ---------- */
const centerTimers = {};
function autoCenter(key, setter, getter){
  clearInterval(centerTimers[key]);
  centerTimers[key] = setInterval(() => {
    const v = getter();
    if(Math.abs(v) < 3){ setter(0); clearInterval(centerTimers[key]); return; }
    setter(v * 0.75);
  }, 40);
}
function enableDrag(handle, track, vertical, key, setter, getter){
  let dragging = false;
  const move = (e) => {
    const r = track.getBoundingClientRect();
    const rel = vertical ? (r.bottom - e.clientY) / r.height : (e.clientX - r.left) / r.width;
    setter((clamp(rel, 0, 1) - 0.5) * 200);
  };
  handle.addEventListener("pointerdown", (e) => {
    if(lockedOut()) return;
    clearInterval(centerTimers[key]);
    dragging = true; handle.setPointerCapture(e.pointerId); e.stopPropagation();
  });
  handle.addEventListener("pointermove", (e) => { if(dragging) move(e); });
  ["pointerup", "pointercancel"].forEach(t => handle.addEventListener(t, () => {
    if(!dragging) return;
    dragging = false;
    if(key === "steer" || !state.cruise) autoCenter(key, setter, getter);   // 巡航時油門保持
  }));
}
enableDrag($("throttleHandle"), throttleTrack, true,  "throttle", setThrottle, () => state.throttle);
enableDrag($("steerHandle"),    steerTrack,    false, "steer",    setSteer,    () => state.steer);

/* ---------- 鎖定（長按 2 秒解鎖） ---------- */
const lockBtn = $("lockBtn");
let lockTimer = null;
function setLockUI(){
  lockBtn.classList.toggle("unlocked", !state.lock);
  $("lockIcon").textContent = state.lock ? "🔒" : "🔓";
  $("lockText").textContent = state.lock ? "LOCK" : "UNLOCKED";
  $("lockSub").textContent  = state.lock ? "安全鎖定" : "可操作";
  renderCmd();
}
lockBtn.addEventListener("pointerdown", (e) => {
  e.stopPropagation();
  lockBtn.setPointerCapture(e.pointerId);
  if(state.lock){
    lockBtn.classList.add("holding");
    lockTimer = setTimeout(() => { state.lock = false; lockBtn.classList.remove("holding"); setLockUI(); }, 2000);
  } else {
    state.lock = true; setThrottle(0); setSteer(0); setLockUI();
  }
});
["pointerup", "pointercancel"].forEach(t => lockBtn.addEventListener(t, () => {
  clearTimeout(lockTimer); lockBtn.classList.remove("holding");
}));

/* ---------- 急停 ---------- */
$("estopBtn").addEventListener("click", (e) => {
  e.stopPropagation();
  state.estop = !state.estop;
  $("estopBtn").classList.toggle("tripped", state.estop);
  if(state.estop){
    setThrottle(0); setSteer(0);
    state.cruise = false; setCruiseUI();
    state.lock = true; setLockUI();
    send({ type: "estop" });
    lastSent = { l: 0, r: 0 }; lastSentAt = Date.now();
    setMission("idle");
  }
  renderCmd();
});

/* ---------- 定速巡航（只在前端保持油門，不另外傳資料） ---------- */
function setCruiseUI(){
  $("cruiseBtn").classList.toggle("on", state.cruise);
  $("cruiseTitle").textContent = state.cruise ? "CRUISE ON" : "CRUISE OFF";
}
$("cruiseBtn").addEventListener("click", (e) => {
  e.stopPropagation();
  if(lockedOut()) return;
  state.cruise = !state.cruise;
  setCruiseUI();
  if(!state.cruise) autoCenter("throttle", setThrottle, () => state.throttle);
});

/* =====================================================================
   自動整備：地圖型號 + 開始/停止
   ===================================================================== */
const fieldSpec = $("fieldSpec"), pathSpec = $("pathSpec"), baseSpacing = $("baseSpacing");
const SVG_NS = "http://www.w3.org/2000/svg";
const baseGroups = { home: $("baseHome"), first: $("baseFirst"), third: $("baseThird"), second: $("baseSecond") };
const SEG_STATUS = { idle: "pending", running: "active", done: "done" };
const FOOTER = {
  idle:    "選擇球場規格與路徑後，按「開始整備」。",
  running: "整備進行中…（切到手動頁會中止任務）",
  done:    "整備完成。",
};

let renderToken = 0;
async function renderDiamond(){
  const token = ++renderToken;
  try{
    const spacing = parseFloat(baseSpacing.value) || 27.43;
    const coords = FieldLibrary.getBaseCoords(spacing);
    for(const [name, g] of Object.entries(baseGroups)){
      const p = FieldLibrary.worldToSvg(coords[name], { spacingM: spacing });
      g.setAttribute("transform", `translate(${p.x},${p.y})`);
    }
    const segs = await FieldLibrary.getPath(pathSpec.value, spacing);
    if(token !== renderToken) return;
    const layer = $("pathLayer");
    layer.innerHTML = "";
    for(const s of segs){
      const a = FieldLibrary.worldToSvg(s.from, { spacingM: spacing });
      const b = FieldLibrary.worldToSvg(s.to,   { spacingM: spacing });
      const ln = document.createElementNS(SVG_NS, "line");
      ln.setAttribute("x1", a.x); ln.setAttribute("y1", a.y);
      ln.setAttribute("x2", b.x); ln.setAttribute("y2", b.y);
      ln.setAttribute("class", `path ${SEG_STATUS[state.mission]}`);
      layer.appendChild(ln);
    }
  }catch(err){ console.error("[renderDiamond]", err); }
}

function setMission(status){
  state.mission = status;
  const running = status === "running";
  $("startPrep").classList.toggle("stop", running);
  $("startIcon").textContent = running ? "■" : "▶";
  $("startLabel").textContent = running ? "停止整備" : "開始整備";
  [fieldSpec, pathSpec, baseSpacing].forEach(el => el.disabled = running);
  $("missionStatus").textContent = { idle: "待命", running: "整備中", done: "已完成" }[status];
  $("missionFooter").textContent = FOOTER[status];
  renderDiamond();
}

$("startPrep").addEventListener("click", (e) => {
  e.stopPropagation();
  if(state.mission === "running"){ send({ type: "stop" }); setMission("idle"); return; }
  const spacing = parseFloat(baseSpacing.value);
  if(!(spacing >= 5 && spacing <= 50)){ $("missionFooter").textContent = "壘包間距需介於 5 – 50 m。"; return; }
  if(!wsReady){ $("missionFooter").textContent = "尚未連線，無法開始。"; return; }
  send({ type: "mode", value: "auto" });
  send({ type: "start", map: { preset: fieldSpec.value, spacing, path: pathSpec.value } });
  setMission("running");
});

fieldSpec.addEventListener("change", async (e) => {
  e.stopPropagation();
  baseSpacing.value = (await FieldLibrary.getPreset(fieldSpec.value)).base_spacing_m.toFixed(2);
  renderDiamond();
});
pathSpec.addEventListener("change", renderDiamond);
baseSpacing.addEventListener("input", renderDiamond);
[fieldSpec, pathSpec, baseSpacing].forEach(el => el.addEventListener("pointerdown", (e) => e.stopPropagation()));

(async function init(){
  const lib = await FieldLibrary.load();
  for(const name of Object.keys(lib.presets)) fieldSpec.add(new Option(name, name));
  for(const [k, v] of Object.entries(lib.path_templates)) pathSpec.add(new Option(v.label || k, k));
  baseSpacing.value = lib.presets[fieldSpec.value].base_spacing_m.toFixed(2);
  setMission("idle");
})();

goToPage(0);
renderThrottle(); renderSteer(); renderCmd(); setLockUI(); setCruiseUI();
