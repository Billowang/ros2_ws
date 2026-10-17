"""
field_library.py
=================
場地 / 路徑資料庫的 Python 端存取介面，跟 field_library.json、field-library.js
共用同一份資料檔與同一套座標系統：

    原點 (0, 0) = 本壘
    X 軸：本壘 → 一壘 方向
    Y 軸：本壘 → 三壘 方向
    二壘 = (壘包間距, 壘包間距)

這樣設計的好處是，之後要接 ROS 2 時，「場地形狀」相關的數字全部集中在
field_library.json 這一份檔案，server.py／你自己寫的節點都從這裡讀，
不會出現前端一份座標、後端又手key另一份座標、兩邊對不起來的狀況。
要接到 RTK 的世界座標（例如 /map frame）時，只需要在這層外面再乘上一個
旋轉＋平移矩陣即可，場地本身的相對幾何完全不用改。

用法範例：
    from field_library import get_base_coords, get_path, get_preset

    coords = get_base_coords(27.43)
    print(coords["second"])           # Point(x=27.43, y=27.43)

    segs = get_path("perimeter_full", 27.43, progress_index=1)
    for seg in segs:
        print(seg["from_base"], "->", seg["to_base"], seg["status"])
"""

import json
import os
from dataclasses import dataclass, asdict

LIBRARY_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "field_library.json")


@dataclass
class Point:
    x: float
    y: float


# ---------------------------------------------------------------------------
# 資料庫讀寫
# ---------------------------------------------------------------------------
def load_library() -> dict:
    with open(LIBRARY_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def save_library(data: dict) -> None:
    """寫回 field_library.json（例如你要用程式新增/修改場地規格或路徑範本時使用）。"""
    with open(LIBRARY_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


# ---------------------------------------------------------------------------
# 場地規格
# ---------------------------------------------------------------------------
def list_presets() -> dict:
    return load_library()["presets"]


def get_preset(name: str) -> dict:
    presets = list_presets()
    if name not in presets:
        raise KeyError(f"找不到場地規格：{name}")
    return presets[name]


def add_preset(name: str, base_spacing_m: float) -> None:
    lib = load_library()
    lib["presets"][name] = {"base_spacing_m": base_spacing_m}
    save_library(lib)


# ---------------------------------------------------------------------------
# 座標計算（核心：本壘為原點，X=本壘→一壘，Y=本壘→三壘）
# ---------------------------------------------------------------------------
def get_base_coords(spacing_m: float) -> dict:
    """回傳四個壘包在場地座標系下的位置（公尺）。"""
    return {
        "home":   Point(0.0, 0.0),
        "first":  Point(spacing_m, 0.0),
        "third":  Point(0.0, spacing_m),
        "second": Point(spacing_m, spacing_m),
    }


# ---------------------------------------------------------------------------
# 路徑範本
# ---------------------------------------------------------------------------
def list_path_templates() -> dict:
    return load_library()["path_templates"]


def get_path(path_name: str, spacing_m: float, progress_index: int = 0) -> list:
    """
    展開指定路徑範本成線段列表。

    progress_index：目前走到第幾段（0-based）。
        小於 progress_index 的段落標記為 "done"，
        等於的標記為 "active"，其餘標記為 "pending"。

    回傳的每個線段是一個 dict：
        {"from_base": "home", "to_base": "third",
         "from": Point(0,0), "to": Point(0, 27.43), "status": "done"}
    """
    templates = list_path_templates()
    if path_name not in templates:
        raise KeyError(f"找不到路徑範本：{path_name}")
    waypoints = templates[path_name]["waypoints"]
    coords = get_base_coords(spacing_m)

    segments = []
    for i in range(len(waypoints) - 1):
        a, b = waypoints[i], waypoints[i + 1]
        if i < progress_index:
            status = "done"
        elif i == progress_index:
            status = "active"
        else:
            status = "pending"
        segments.append({
            "from_base": a, "to_base": b,
            "from": coords[a], "to": coords[b],
            "status": status,
        })
    return segments


def path_as_json_safe(segments: list) -> list:
    """把 get_path() 回傳的 Point 物件轉成 dict，方便直接 json.dumps 或透過 WebSocket 送給前端。"""
    out = []
    for seg in segments:
        out.append({
            "from_base": seg["from_base"], "to_base": seg["to_base"],
            "from": asdict(seg["from"]), "to": asdict(seg["to"]),
            "status": seg["status"],
        })
    return out


if __name__ == "__main__":
    # 自我測試：直接執行 `python3 field_library.py` 就能看到輸出
    spacing = get_preset("成棒（國際規格）")["base_spacing_m"]
    print(f"壘包間距：{spacing} m\n")

    coords = get_base_coords(spacing)
    for name, p in coords.items():
        print(f"  {name:>6s}: ({p.x:6.2f}, {p.y:6.2f})")

    print()
    for seg in get_path("perimeter_full", spacing, progress_index=1):
        print(f"  {seg['from_base']:>6s} -> {seg['to_base']:<6s}  [{seg['status']}]")
