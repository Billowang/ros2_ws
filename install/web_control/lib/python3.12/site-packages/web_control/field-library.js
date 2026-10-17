/* field-library.js
 * ===================
 * 場地 / 路徑資料庫（前端）。跟 field_library.json、field_library.py
 * 共用同一份資料檔與同一套座標系統：
 *
 *   原點 (0,0) = 本壘
 *   X 軸：本壘 → 一壘 方向
 *   Y 軸：本壘 → 三壘 方向
 *   二壘 = (壘包間距, 壘包間距)
 *
 * 這份檔案只負責「資料」跟「座標換算」，不碰 DOM——畫面繪製邏輯留在 app.js。
 */
const FieldLibrary = (() => {
  let cache = null;

  // 萬一 field_library.json 抓不到（例如漏放檔案、或用 file:// 直接開網頁導致
  // fetch 被瀏覽器擋下），退回這份內建預設值，讓畫面至少能正常顯示，
  // 不會整個 JS 卡住、後面的按鈕/滑桿都失效。
  const FALLBACK = {
    presets: {
      "成棒（國際規格）": { base_spacing_m: 27.43 },
      "青少棒": { base_spacing_m: 25.00 },
      "少棒": { base_spacing_m: 18.29 },
      "壘球": { base_spacing_m: 18.29 },
    },
    path_templates: {
      perimeter_full: { label: "整備路徑（全場）", waypoints: ["home", "third", "second", "first", "home"] },
      perimeter_half: { label: "整備路徑（半場）", waypoints: ["home", "third", "second"] },
    },
  };

  async function load() {
    if (cache) return cache;
    try {
      const res = await fetch("field_library.json");
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      cache = await res.json();
    } catch (err) {
      console.warn("[FieldLibrary] 讀取 field_library.json 失敗，改用內建預設資料：", err);
      cache = FALLBACK;
    }
    return cache;
  }

  /** 本壘=原點、X=本壘→一壘、Y=本壘→三壘 的四壘包座標（公尺）。 */
  function getBaseCoords(spacingM) {
    return {
      home:   { x: 0,        y: 0 },
      first:  { x: spacingM*0.5, y: spacingM*0.5 },
      third:  { x: -spacingM*0.5,        y: spacingM*0.5 },
      second: { x: 0 , y: spacingM },
    };
  }

  async function listPresetNames() {
    const lib = await load();
    return Object.keys(lib.presets);
  }

  async function getPreset(name) {
    const lib = await load();
    const preset = lib.presets[name];
    if (!preset) throw new Error(`找不到場地規格：${name}`);
    return preset;
  }

  /**
   * 展開指定路徑範本成線段列表。
   * progressIndex：目前走到第幾段（0-based）；之前的段落回傳 status="done"，
   * 目前這段 "active"，之後的 "pending"。
   */
  async function getPath(pathName, spacingM, progressIndex = 0) {
    const lib = await load();
    const template = lib.path_templates[pathName];
    if (!template) throw new Error(`找不到路徑範本：${pathName}`);
    const coords = getBaseCoords(spacingM);

    const segments = [];
    for (let i = 0; i < template.waypoints.length - 1; i++) {
      const a = template.waypoints[i], b = template.waypoints[i + 1];
      const status = i < progressIndex ? "done" : (i === progressIndex ? "active" : "pending");
      segments.push({ fromBase: a, toBase: b, from: coords[a], to: coords[b], status });
    }
    return segments;
  }

  /**
   * 把場地座標（公尺，本壘為原點）投影成 SVG 像素座標。
   * 依 spacingM 自動置中、縮放，flipY 讓「本壘朝下、二壘朝上」符合畫面直覺。
   */
  function worldToSvg(point, { spacingM, viewW = 360, viewH = 320, margin = 50, flipY = true }) {
    const scale = (Math.min(viewW, viewH) - margin * 2) / spacingM;
    const x = margin + point.x * scale;
    const y = flipY ? (viewH - margin - point.y * scale) : (margin + point.y * scale);
    return { x, y };
  }

  return { load, getBaseCoords, listPresetNames, getPreset, getPath, worldToSvg };
})();
