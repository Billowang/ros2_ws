/* field-library.js — 場地 / 路徑資料庫（前端，只負責資料與座標換算，不碰 DOM）
 * 座標系統與 field_library.py 完全一致：
 *   原點 (0,0)=本壘，X=本壘→一壘，Y=本壘→三壘，二壘=(間距,間距)，單位公尺。
 * 畫面上為了讓內野「菱形朝上」，在 worldToSvg() 內才旋轉 45°，資料本身不旋轉。
 */
const FieldLibrary = (() => {
  let cache = null;

  // 抓不到 field_library.json（例如用 file:// 開啟）時的內建備援
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

  function getBaseCoords(s) {
    return { home: { x: 0, y: 0 }, first: { x: s, y: 0 }, third: { x: 0, y: s }, second: { x: s, y: s } };
  }

  async function getPreset(name) {
    const lib = await load();
    if (!lib.presets[name]) throw new Error(`找不到場地規格：${name}`);
    return lib.presets[name];
  }

  /** 展開路徑範本成線段（不帶進度；狀態由 app.js 依任務狀態決定）。 */
  async function getPath(pathName, spacingM) {
    const lib = await load();
    const t = lib.path_templates[pathName];
    if (!t) throw new Error(`找不到路徑範本：${pathName}`);
    const c = getBaseCoords(spacingM);
    const segs = [];
    for (let i = 0; i < t.waypoints.length - 1; i++) {
      const a = t.waypoints[i], b = t.waypoints[i + 1];
      segs.push({ fromBase: a, toBase: b, from: c[a], to: c[b] });
    }
    return segs;
  }

  /** 場地座標 → SVG 像素：旋轉 45°（本壘在下、二壘在上），並依間距自動縮放、水平置中。 */
  function worldToSvg(p, { spacingM, viewW = 360, viewH = 320, margin = 50 }) {
    const k = Math.SQRT1_2;
    const dx = (p.x - p.y) * k, dy = (p.x + p.y) * k;
    const scale = (Math.min(viewW, viewH) - margin * 2) / (spacingM * 2 * k);
    return { x: viewW / 2 + dx * scale, y: viewH - margin - dy * scale };
  }

  return { load, getBaseCoords, getPreset, getPath, worldToSvg };
})();
