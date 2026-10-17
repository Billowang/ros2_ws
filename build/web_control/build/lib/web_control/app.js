/* ---------- 設定 ---------- */
const WS_URL = (location.protocol === "https:" ? "wss://" : "ws://") + location.hostname + ":8765";
const SEND_HZ = 20;                 // 傳送頻率
const STEP = 4;                     // 按住箭頭按鈕時每個 tick 的增量
const REPEAT_MS = 60;               // 長按箭頭的重複間隔

/* ---------- 狀態 ---------- */
const state = {
  throttle: 0,   // -100..100
  steer: 0,      // -100..100
  lock: true,    // true = 鎖定中（無法移動）
  estop: false,
  cruise: false,
};

/* ---------- DOM ---------- */
const el = (id) => document.getElementById(id);
const clockEl = el("clock");
const connDot = el("conn-dot");
const connLabel = el("conn-label");
const pageTitle = el("pageTitle");

const lockBtn = el("lockBtn"), lockIcon = el("lockIcon"), lockText = el("lockText"), lockSub = el("lockSub");
const estopBtn = el("estopBtn");
const cruiseBtn = el("cruiseBtn"), cruiseTitle = el("cruiseTitle");

const throttleTrack = el("throttleTrack"), throttleFill = el("throttleFill"), throttleHandle = el("throttleHandle"), throttleVal = el("throttleVal");
const steerTrack = el("steerTrack"), steerFill = el("steerFill"), steerHandle = el("steerHandle"), steerVal = el("steerVal");

const leftRpm = el("leftRpm"), rightRpm = el("rightRpm"), leftBar = el("leftBar"), rightBar = el("rightBar");
const battPct = el("battPct"), battRing = el("battRing"), battTime = el("battTime");
const voltage = el("voltage"), current = el("current");

/* ---------- 時鐘 ---------- */
function tickClock(){
  const d = new Date();
  clockEl.textContent = d.toTimeString().slice(0,5);
}
tickClock();
setInterval(tickClock, 1000 * 15);

/* =====================================================================
   兩頁式切換（任務設定 <-> 手動控制），左右箭頭 / 滑動皆可切換
   ===================================================================== */
const pagesTrack = el("pagesTrack");
const prevPageBtn = el("prevPage");
const nextPageBtn = el("nextPage");
const pageTitles = ["自動整備模式", "手動控制"];
let currentPage = 0;
const PAGE_COUNT = 2;

function renderPage(){
  pagesTrack.style.transform = `translateX(-${currentPage * 50}%)`;
  pageTitle.textContent = pageTitles[currentPage];
  prevPageBtn.disabled = currentPage === 0;
  nextPageBtn.disabled = currentPage === PAGE_COUNT - 1;
}
function goToPage(idx){
  currentPage = Math.max(0, Math.min(PAGE_COUNT - 1, idx));
  renderPage();
}
prevPageBtn.addEventListener("click", () => goToPage(currentPage - 1));
nextPageBtn.addEventListener("click", () => goToPage(currentPage + 1));

/* 滑動手勢切頁（觸控/滑鼠拖曳整個畫面時） */
(function enableSwipe(){
  let startX = null;
  const viewport = document.querySelector(".pages-viewport");
  viewport.addEventListener("pointerdown", (e) => {
    // 避免滑桿把手、按鈕自己的拖曳被畫面滑動蓋掉
    if(e.target.closest(".vhandle, .hhandle, .arrow, .lock-btn, .estop-btn, .cruise-btn, button, select, input")) return;
    startX = e.clientX;
  });
  viewport.addEventListener("pointerup", (e) => {
    if(startX === null) return;
    const dx = e.clientX - startX;
    startX = null;
    if(dx > 60) goToPage(currentPage - 1);
    else if(dx < -60) goToPage(currentPage + 1);
  });
})();

renderPage();

/* ---------- 節流/轉向 顯示更新 ---------- */
function renderThrottle(){
  throttleVal.textContent = state.throttle;
  const pct = Math.abs(state.throttle) / 2; // 0..50 (%)
  throttleFill.style.height = pct + "%";
  throttleFill.style.bottom = state.throttle >= 0 ? "50%" : (50 - pct) + "%";
  const trackH = throttleTrack.clientHeight;
  const offset = (state.throttle / 100) * (trackH / 2);
  throttleHandle.style.bottom = `calc(50% + ${offset}px)`;
}

function renderSteer(){
  steerVal.textContent = state.steer;
  const pct = Math.abs(state.steer) / 2;
  steerFill.style.width = pct + "%";
  steerFill.style.left = state.steer >= 0 ? "50%" : (50 - pct) + "%";
  const trackW = steerTrack.clientWidth;
  const offset = (state.steer / 100) * (trackW / 2);
  steerHandle.style.left = `calc(50% + ${offset}px)`;
}

function clamp(v, lo, hi){ return Math.max(lo, Math.min(hi, v)); }

function setThrottle(v){
  state.throttle = clamp(Math.round(v), -100, 100);
  renderThrottle();
}
function setSteer(v){
  state.steer = clamp(Math.round(v), -100, 100);
  renderSteer();
}

/* ---------- 箭頭按鈕：按住連續增量 ---------- */
function holdRepeat(btn, fn){
  let iv = null;
  const start = (e) => { e.preventDefault(); e.stopPropagation(); fn(); iv = setInterval(fn, REPEAT_MS); };
  const stop = () => { clearInterval(iv); iv = null; };
  btn.addEventListener("pointerdown", start);
  btn.addEventListener("pointerup", stop);
  btn.addEventListener("pointerleave", stop);
  btn.addEventListener("pointercancel", stop);
}
holdRepeat(el("throttleUp"),   () => { if(!lockedOut()) setThrottle(state.throttle + STEP); });
holdRepeat(el("throttleDown"), () => { if(!lockedOut()) setThrottle(state.throttle - STEP); });
holdRepeat(el("steerRight"),   () => { if(!lockedOut()) setSteer(state.steer + STEP); });
holdRepeat(el("steerLeft"),    () => { if(!lockedOut()) setSteer(state.steer - STEP); });

/* ---------- 拖曳滑桿 ---------- */
function enableVerticalDrag(handle, track, onChange){
  let dragging = false;
  const move = (clientY) => {
    const rect = track.getBoundingClientRect();
    const rel = clamp((rect.bottom - clientY) / rect.height, 0, 1); // 0 bottom .. 1 top
    onChange((rel - 0.5) * 200); // -100..100
  };
  handle.addEventListener("pointerdown", (e) => {
    if(lockedOut()) return;
    dragging = true;
    handle.setPointerCapture(e.pointerId);
    e.stopPropagation();
  });
  handle.addEventListener("pointermove", (e) => { if(dragging) move(e.clientY); });
  handle.addEventListener("pointerup", () => dragging = false);
  handle.addEventListener("pointercancel", () => dragging = false);
}
function enableHorizontalDrag(handle, track, onChange){
  let dragging = false;
  const move = (clientX) => {
    const rect = track.getBoundingClientRect();
    const rel = clamp((clientX - rect.left) / rect.width, 0, 1); // 0 left .. 1 right
    onChange((rel - 0.5) * 200);
  };
  handle.addEventListener("pointerdown", (e) => {
    if(lockedOut()) return;
    dragging = true;
    handle.setPointerCapture(e.pointerId);
    e.stopPropagation();
  });
  handle.addEventListener("pointermove", (e) => { if(dragging) move(e.clientX); });
  handle.addEventListener("pointerup", () => dragging = false);
  handle.addEventListener("pointercancel", () => dragging = false);
}
enableVerticalDrag(throttleHandle, throttleTrack, setThrottle);
enableHorizontalDrag(steerHandle, steerTrack, setSteer);

/* 放開後緩緩回中（模擬自動回正的搖桿；如需保持定值可移除這段） */
["pointerup","pointercancel"].forEach(evt=>{
  throttleHandle.addEventListener(evt, () => autoCenter(setThrottle, () => state.throttle));
  steerHandle.addEventListener(evt, () => autoCenter(setSteer, () => state.steer));
});
function autoCenter(setter, getter){
  const iv = setInterval(() => {
    const v = getter();
    if(Math.abs(v) < 3){ setter(0); clearInterval(iv); return; }
    setter(v * 0.75);
  }, 40);
}

/* ---------- 鎖定（長按 2 秒解鎖） ---------- */
let lockTimer = null;
function lockedOut(){ return state.lock || state.estop; }
function setLockUI(){
  lockBtn.classList.toggle("unlocked", !state.lock);
  lockIcon.textContent = state.lock ? "🔒" : "🔓";
  lockText.textContent = state.lock ? "LOCK" : "UNLOCKED";
  lockSub.textContent = state.lock ? "安全鎖定" : "可操作";
}
lockBtn.addEventListener("pointerdown", (e) => {
  e.stopPropagation();
  lockBtn.setPointerCapture(e.pointerId);   // 讓瀏覽器持續追蹤，滑鼠稍微移出範圍也不會取消長按
  if(state.lock){
    lockBtn.classList.add("holding");
    lockTimer = setTimeout(() => {
      state.lock = false;
      lockBtn.classList.remove("holding");
      setLockUI();
      sendControl();
    }, 2000);
  } else {
    state.lock = true; setThrottle(0); setSteer(0); setLockUI(); sendControl();
  }
});
lockBtn.addEventListener("pointerup", () => {
  clearTimeout(lockTimer);
  lockBtn.classList.remove("holding");
});
setLockUI();

/* ---------- 急停 ---------- */
estopBtn.addEventListener("click", (e) => {
  e.stopPropagation();
  state.estop = !state.estop;
  estopBtn.classList.toggle("tripped", state.estop);
  if(state.estop){ setThrottle(0); setSteer(0); state.cruise = false; setCruiseUI(); }
  sendControl();
});

/* ---------- 定速巡航 ---------- */
function setCruiseUI(){
  cruiseBtn.classList.toggle("on", state.cruise);
  cruiseTitle.textContent = state.cruise ? "CRUISE ON" : "CRUISE OFF";
}
cruiseBtn.addEventListener("click", (e) => {
  e.stopPropagation();
  if(lockedOut()) return;
  state.cruise = !state.cruise;
  setCruiseUI();
  sendControl();
});
setCruiseUI();

/* ---------- WebSocket ---------- */
let ws = null;
let wsReady = false;

function connect(){
  ws = new WebSocket(WS_URL);
  ws.onopen = () => { wsReady = true; connDot.classList.add("live"); connLabel.textContent = "已連線"; };
  ws.onclose = () => { wsReady = false; connDot.classList.remove("live"); connLabel.textContent = "已斷線 - 重連中…"; setTimeout(connect, 1500); };
  ws.onerror = () => ws.close();
  ws.onmessage = (evt) => {
    try{
      const msg = JSON.parse(evt.data);
      if(msg.type === "telemetry") applyTelemetry(msg);
    }catch(err){ console.error("bad message", err); }
  };
}
connect();

function sendControl(){
  if(!wsReady) return;
  ws.send(JSON.stringify({
    type: "control",
    throttle: state.throttle,
    steer: state.steer,
    lock: state.lock,
    estop: state.estop,
    cruise: state.cruise,
    ts: Date.now(),
  }));
}
setInterval(sendControl, 1000 / SEND_HZ);

/* ---------- 遙測資料顯示（頁 2：手動控制） ---------- */
function applyTelemetry(t){
  if(t.left_rpm  !== undefined){ leftRpm.textContent  = Math.round(t.left_rpm);  leftBar.style.width  = clamp(Math.abs(t.left_rpm)/6,0,100)+"%"; }
  if(t.right_rpm !== undefined){ rightRpm.textContent = Math.round(t.right_rpm); rightBar.style.width = clamp(Math.abs(t.right_rpm)/6,0,100)+"%"; }
  if(t.battery_pct !== undefined){
    battPct.textContent = Math.round(t.battery_pct);
    const circumference = 326.7;
    battRing.style.strokeDashoffset = circumference * (1 - t.battery_pct/100);
    battRing.style.stroke = t.battery_pct < 20 ? "#e5372d" : "#1fae5d";
  }
  if(t.batt_time_min !== undefined){
    const h = Math.floor(t.batt_time_min/60), m = Math.round(t.batt_time_min%60);
    battTime.textContent = `${h}h ${m}m`;
  }
  if(t.voltage !== undefined) voltage.textContent = t.voltage.toFixed(1) + " V";
  if(t.current !== undefined) current.textContent = t.current.toFixed(1) + " A";

  /* 同步更新頁 1（任務設定）的系統狀態卡片 */
  const missionBatt = el("missionBatt"), wheelRpm = el("wheelRpm");
  if(t.battery_pct !== undefined && missionBatt) missionBatt.textContent = Math.round(t.battery_pct) + "%";
  if(wheelRpm && (t.left_rpm !== undefined || t.right_rpm !== undefined)){
    wheelRpm.textContent = `L ${Math.round(t.left_rpm||0)} rpm   R ${Math.round(t.right_rpm||0)} rpm`;
  }
}

/* =====================================================================
   任務地圖繪製（使用 field-library.js 的場地/路徑資料）
   座標系統：本壘=原點(0,0)，X=本壘→一壘，Y=本壘→三壘
   ===================================================================== */
const fieldSpecSelect = el("fieldSpec");
const baseSpacingInput = el("baseSpacing");
const pathLayer = el("pathLayer");
const baseGroups = { home: el("baseHome"), first: el("baseFirst"), third: el("baseThird"), second: el("baseSecond") };
const robotDot = el("robotDot");

let missionProgress = 1;          // 目前走到路徑的第幾段（0-based），示範用
const ACTIVE_PATH = "perimeter_full";

const SVG_NS = "http://www.w3.org/2000/svg";

async function renderDiamond(){
  try{
    const spacing = parseFloat(baseSpacingInput.value) || 27.43;

    // 1) 四個壘包位置
    const coords = FieldLibrary.getBaseCoords(spacing);
    for(const [name, group] of Object.entries(baseGroups)){
      const p = FieldLibrary.worldToSvg(coords[name], { spacingM: spacing });
      group.setAttribute("transform", `translate(${p.x},${p.y})`);
    }

    // 2) 路徑線段（done / active / pending），數量依路徑範本而定
    const segments = await FieldLibrary.getPath(ACTIVE_PATH, spacing, missionProgress);
    pathLayer.innerHTML = "";
    let activeMid = null;
    segments.forEach((seg) => {
      const p1 = FieldLibrary.worldToSvg(seg.from, { spacingM: spacing });
      const p2 = FieldLibrary.worldToSvg(seg.to,   { spacingM: spacing });
      const line = document.createElementNS(SVG_NS, "line");
      line.setAttribute("x1", p1.x); line.setAttribute("y1", p1.y);
      line.setAttribute("x2", p2.x); line.setAttribute("y2", p2.y);
      line.setAttribute("class", `path ${seg.status}`);
      pathLayer.appendChild(line);
      if(seg.status === "active") activeMid = { x: (p1.x + p2.x) / 2, y: (p1.y + p2.y) / 2 };
    });

    // 3) 機器人目前位置：示範用，畫在進行中那一段的中點
    if(activeMid){
      robotDot.setAttribute("cx", activeMid.x);
      robotDot.setAttribute("cy", activeMid.y);
      robotDot.style.display = "";
    } else {
      robotDot.style.display = "none";
    }
  }catch(err){
    console.error("[renderDiamond] 畫面渲染失敗，請檢查 field_library.json / field-library.js 是否都放在同一個資料夾：", err);
  }
}

fieldSpecSelect.addEventListener("change", async (e) => {
  e.stopPropagation();
  const preset = await FieldLibrary.getPreset(fieldSpecSelect.value);
  baseSpacingInput.value = preset.base_spacing_m.toFixed(2);
  renderDiamond();
});
baseSpacingInput.addEventListener("input", (e) => { e.stopPropagation(); renderDiamond(); });
baseSpacingInput.addEventListener("pointerdown", (e) => e.stopPropagation());

renderDiamond();

/* 開始整備按鈕：目前僅為前端示意，之後可在這裡送出對應的 WebSocket/ROS 2 指令 */
el("startPrep").addEventListener("click", (e) => {
  e.stopPropagation();
  missionProgress = Math.min(missionProgress + 1, 3);
  renderDiamond();
  el("missionFooter").textContent = "整備已開始，機器人正沿路徑移動…";
});

/* 初次進畫面把滑桿畫在正中央 */
renderThrottle();
renderSteer();
