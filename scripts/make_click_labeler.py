#!/usr/bin/env python3
"""生成「顶视方块点选」页面 —— 人工点一下，几何算坐标。

为什么要有这个
==============
`propose_block_positions.py` 用颜色找候选、用倒角精修，能自动认出大部分方块，
但有**一整类它结构上做不到的事**，本轮实测撞到的三条：

  · **米色方块 S≈27，白纸 S≈24** —— 任何色相/饱和度阈值都分不开。「白纸上的
    浅色方块」这件事在颜色空间里根本不存在。
  · **阴影里的紫方块 V≈71** —— 亮度下限 80 把它整个排除掉；调低下限，
    台面杂物（香蕉、纸边）全部涌进来。
  · **急停按钮的红色蘑菇头** —— 饱和、够大、外轮廓倒角 0.00 px、体检尺寸
    28.5 mm，三道闸门全部放行。它是**形状正确**的假阳性。

这不是调参没调好，是**用错了仪器**：颜色分不开的东西，再好的颜色阈值也分不开。
而「哪块是方块」对人是一眼的事，对阈值是永远做不到的事。

所以这一步把**检测交给人**（点一下顶面中心），把**几何留给代码**
（`_rough_seed`：像素 → 靶系，往返实测 0.6 mm）。19 个方块大约 40 秒点完，
而且每一个都精确到毫米级 —— 比自动提案器更准，因为它不猜。

它解决的是**每一轮采集**的成本，不是这一次。`docs/待执行命令.md` §1-7 里
「人工过一遍"提案结果"」那一步，从此变成「点一遍」，而不再需要我事后
在对话里根据你的描述去猜像素坐标。

用法
====
    python3 scripts/make_click_labeler.py \\
        --info /tmp/ds_scene2/info.json \\
        --extrinsics /tmp/ds_scene2/labeler_extrinsics.json \\
        --image /tmp/ds_scene2/frame.png \\
        --size-mm 28.5 \\
        --out /tmp/ds_scene2/click.html

`--extrinsics` 收的是 `board_extrinsics.py cmd_labeler` 产出的那份
（`TfProjector` 要的 target←camera 形式，键为 `rotation`/`translation`）。

页面点完 → 「复制 JSON」→ 得到的列表**直接就是 `blocks.json` 的格式**
（`[{"x","y","z","size"}]`），可以直接喂给 `auto_label_grid.py`。
"""

import argparse
import base64
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from board_extrinsics import quat_to_matrix                        # noqa: E402
import auto_label_grid as L                                        # noqa: E402


TEMPLATE = r"""<title>顶视方块点选</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<style>
  /* 布局：顶部条 + [画面 | 侧栏] + 底部输出；窄屏塌成一列。
     配色：冷中性底 + 仪器绿强调；画面是唯一的高饱和内容，界面不跟它抢。 */
  :root {
    --bg: #f1f2f0;
    --surface: #fbfcfa;
    --sunken: #e7e9e5;
    --fg: #161a18;
    --muted: #66706a;
    --line: #d8dcd6;
    --line-strong: #b9c0b8;
    --accent: #16705c;
    --accent-soft: #dcece6;
    --mark: #c8402c;
    --mark-soft: rgba(200, 64, 44, 0.14);
    --on-accent: #ffffff;
    --on-mark: #ffffff;
    --font-ui: "IBM Plex Sans", system-ui, -apple-system, "Segoe UI", "Noto Sans SC", sans-serif;
    --font-data: "IBM Plex Mono", ui-monospace, "SFMono-Regular", Menlo, monospace;
    color-scheme: light;
  }
  @media (prefers-color-scheme: dark) {
    :root:not([data-theme="light"]) {
      --bg: #121614;
      --surface: #1a1f1c;
      --sunken: #0d100f;
      --fg: #e6eae7;
      --muted: #8e9a93;
      --line: #2b332e;
      --line-strong: #3d473f;
      --accent: #4fbf9c;
      --accent-soft: #1b2e28;
      --mark: #e8756a;
      --mark-soft: rgba(232, 117, 106, 0.16);
      --on-accent: #0b1512;
      --on-mark: #24100d;
      color-scheme: dark;
    }
  }
  :root[data-theme="dark"] {
    --bg: #121614;
    --surface: #1a1f1c;
    --sunken: #0d100f;
    --fg: #e6eae7;
    --muted: #8e9a93;
    --line: #2b332e;
    --line-strong: #3d473f;
    --accent: #4fbf9c;
    --accent-soft: #1b2e28;
    --mark: #e8756a;
    --mark-soft: rgba(232, 117, 106, 0.16);
    --on-accent: #0b1512;
    --on-mark: #24100d;
    color-scheme: dark;
  }

  * { box-sizing: border-box; }
  body {
    margin: 0;
    background: var(--bg);
    color: var(--fg);
    font-family: var(--font-ui);
    font-size: 14px;
    line-height: 1.5;
  }
  .shell { display: flex; flex-direction: column; gap: 14px; padding-block: 0 28px; }

  header {
    position: sticky; top: env(safe-area-inset-top, 0px); z-index: 20;
    display: flex; flex-wrap: wrap; align-items: baseline; gap: 6px 18px;
    padding: 12px 18px; background: var(--surface);
    border-bottom: 1px solid var(--line);
  }
  header h1 { margin: 0; font-size: 16px; font-weight: 600; letter-spacing: -0.01em; }
  header .sub { color: var(--muted); font-size: 12.5px; font-family: var(--font-data); }
  header .spacer { flex: 1 1 auto; }
  header .stat {
    font-family: var(--font-data); font-size: 12.5px; color: var(--muted);
    font-variant-numeric: tabular-nums;
  }
  header .stat b { color: var(--accent); font-weight: 500; font-size: 15px; }

  main {
    display: grid; grid-template-columns: minmax(0, 1fr) 264px;
    gap: 14px; padding-inline: 18px; align-items: start;
  }
  @media (max-width: 860px) { main { grid-template-columns: minmax(0, 1fr); } }

  .stage { min-width: 0; display: flex; flex-direction: column; gap: 10px; }
  .wrap {
    position: relative; display: block; width: 100%;
    background: var(--sunken); border: 1px solid var(--line-strong);
    border-radius: 3px; overflow: hidden; cursor: crosshair; touch-action: none;
  }
  .wrap img { display: block; width: 100%; height: auto; user-select: none; -webkit-user-drag: none; }
  .overlay { position: absolute; inset: 0; pointer-events: none; }
  .mk {
    position: absolute; width: 17px; height: 17px; margin: -8.5px 0 0 -8.5px;
    border: 1.5px solid var(--mark); border-radius: 50%;
    background: var(--mark-soft); box-shadow: 0 0 0 1.5px rgba(255, 255, 255, 0.55);
  }
  .mk::before, .mk::after {
    content: ""; position: absolute; background: var(--mark);
  }
  .mk::before { left: 50%; top: -4px; bottom: -4px; width: 1.5px; margin-left: -0.75px; }
  .mk::after { top: 50%; left: -4px; right: -4px; height: 1.5px; margin-top: -0.75px; }
  .mk span {
    position: absolute; left: 15px; top: -6px; font: 500 11px/1.4 var(--font-data);
    color: var(--on-mark); background: var(--mark); border-radius: 2px; padding: 1px 4px;
    font-variant-numeric: tabular-nums;
  }
  .hint {
    padding: 9px 12px; background: var(--surface); border: 1px solid var(--line);
    border-radius: 3px; color: var(--muted); font-size: 12.5px;
  }
  .hint b { color: var(--fg); font-weight: 500; }

  aside { min-width: 0; display: flex; flex-direction: column; gap: 12px; }
  .panel {
    background: var(--surface); border: 1px solid var(--line);
    border-radius: 3px; padding: 11px 12px;
  }
  .panel h2 {
    margin: 0 0 9px; font-size: 11px; font-weight: 600; letter-spacing: 0.09em;
    text-transform: uppercase; color: var(--muted);
  }
  .lens { display: flex; gap: 12px; align-items: center; }
  canvas#mag {
    width: 108px; height: 108px; flex: none; border: 1px solid var(--line-strong);
    border-radius: 3px; background: var(--sunken); image-rendering: pixelated;
  }
  .readout { display: flex; flex-direction: column; gap: 5px; min-width: 0; }
  .readout .v {
    font-family: var(--font-data); font-size: 12.5px; font-variant-numeric: tabular-nums;
    color: var(--fg); white-space: nowrap;
  }
  .readout label { font-size: 11px; color: var(--muted); letter-spacing: 0.05em; }
  .readout input {
    width: 68px; font: 400 13px var(--font-data); font-variant-numeric: tabular-nums;
    padding: 3px 5px; border: 1px solid var(--line-strong); border-radius: 2px;
    background: var(--bg); color: var(--fg);
  }

  ol#list {
    list-style: none; margin: 0; padding: 0; max-height: 340px; overflow-y: auto;
    display: flex; flex-direction: column;
  }
  ol#list li {
    display: grid; grid-template-columns: 22px 1fr auto; align-items: center; gap: 7px;
    padding: 4px 2px; border-bottom: 1px solid var(--line);
    font: 400 12px var(--font-data); font-variant-numeric: tabular-nums;
  }
  ol#list li:last-child { border-bottom: 0; }
  ol#list .i { color: var(--muted); }
  ol#list .c { white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
  ol#list button {
    border: 0; background: transparent; color: var(--muted); cursor: pointer;
    font: 400 15px/1 var(--font-ui); padding: 2px 5px; border-radius: 2px;
  }
  ol#list button:hover { color: var(--mark); background: var(--mark-soft); }
  .empty { color: var(--muted); font-size: 12.5px; padding: 6px 2px; }

  .row { display: flex; gap: 8px; flex-wrap: wrap; }
  button.act {
    font: 500 13px var(--font-ui); padding: 6px 11px; border-radius: 3px;
    border: 1px solid var(--line-strong); background: var(--bg); color: var(--fg);
    cursor: pointer;
  }
  button.act:hover { border-color: var(--accent); color: var(--accent); }
  button.act:disabled { opacity: 0.4; cursor: default; }
  button.act:disabled:hover { border-color: var(--line-strong); color: var(--fg); }
  button.primary {
    background: var(--accent); border-color: var(--accent); color: var(--on-accent);
  }
  button.primary:hover { filter: brightness(1.08); color: var(--on-accent); }
  button:focus-visible, input:focus-visible { outline: 2px solid var(--accent); outline-offset: 1px; }

  footer { padding-inline: 18px; }
  .out {
    background: var(--surface); border: 1px solid var(--line); border-radius: 3px;
    padding: 11px 12px; display: flex; flex-direction: column; gap: 9px;
  }
  .out .top { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; }
  .out h2 {
    margin: 0; font-size: 11px; font-weight: 600; letter-spacing: 0.09em;
    text-transform: uppercase; color: var(--muted);
  }
  .out .note { font-size: 12px; color: var(--muted); }
  .out .note.ok { color: var(--accent); }
  textarea {
    width: 100%; min-height: 128px; resize: vertical;
    font: 400 12px/1.55 var(--font-data); font-variant-numeric: tabular-nums;
    padding: 9px 10px; border: 1px solid var(--line); border-radius: 2px;
    background: var(--bg); color: var(--fg); white-space: pre;
  }
</style>

<div class="shell">
  <header>
    <h1>顶视方块点选</h1>
    <span class="sub" id="meta"></span>
    <span class="spacer"></span>
    <span class="stat"><b id="count">0</b> 个方块</span>
  </header>

  <main>
    <section class="stage">
      <div class="wrap" id="wrap">
        <img id="img" alt="顶视相机画面">
        <div class="overlay" id="overlay"></div>
      </div>
      <p class="hint">
        点每个方块的<b>顶面中心</b>。点已标记的圆点可以删掉它。<b>Z</b> 撤销，
        <b>Esc</b> 撤销。坐标按边长换算到靶系 —— 边长为 0 时用台面平面。
      </p>
    </section>

    <aside>
      <div class="panel">
        <h2>光标</h2>
        <div class="lens">
          <canvas id="mag" width="216" height="216"></canvas>
          <div class="readout">
            <div class="v" id="cur">—</div>
            <label for="size">方块边长 (mm)</label>
            <input id="size" type="number" step="0.1" min="1" value="28.5">
          </div>
        </div>
      </div>

      <div class="panel">
        <h2>已点</h2>
        <ol id="list"></ol>
        <div class="row" style="margin-top:9px">
          <button class="act" id="undo">撤销</button>
          <button class="act" id="clear">清空</button>
        </div>
      </div>
    </aside>
  </main>

  <footer>
    <div class="out">
      <div class="top">
        <h2>blocks.json</h2>
        <button class="act primary" id="copy">复制 JSON</button>
        <span class="note" id="note"></span>
      </div>
      <textarea id="out" readonly spellcheck="false"></textarea>
    </div>
  </footer>
</div>

<script>
(function () {
  "use strict";
  var CFG = __CFG__;

  var wrap = document.getElementById("wrap");
  var img = document.getElementById("img");
  var overlay = document.getElementById("overlay");
  var listEl = document.getElementById("list");
  var outEl = document.getElementById("out");
  var noteEl = document.getElementById("note");
  var countEl = document.getElementById("count");
  var curEl = document.getElementById("cur");
  var sizeEl = document.getElementById("size");
  var mag = document.getElementById("mag");
  var mctx = mag.getContext("2d");

  document.getElementById("meta").textContent =
    CFG.W + "×" + CFG.H + "  ·  靶系 x/y  ·  fx " + CFG.fx.toFixed(1);

  img.src = CFG.img;

  var marks = [];          // [{u, v}]，单位是原始像素
  var hover = null;        // [u, v] 或 null
  var STORE = "click-labeler-marks-v1";

  // ---------------------------------------------------------- 几何
  // 与 scripts/propose_block_positions.py::_rough_seed 同一套数学。
  // 正向是 p_target = R·p_cam + t（见 auto_label_grid.TfProjector），
  // 所以「相机系方向 → 靶系方向」乘 **R**，不是 R.T。写成 R.T 不报错，
  // 只是每次都偏 ~80 mm —— 那个坑上一轮已经踩过一次。
  function zsign() { return CFG.t[2] > 0 ? 1 : -1; }

  function planeZ(sizeMM) {
    // 点击落在**顶面**上（方块是竖直侧面的，顶面中心与方块中心的 x/y 相同）。
    // box_corners 把中心放在 ±size/2，两个面分别在 0 与 -size（zsign=-1）。
    // 相机在 t[2]<0 一侧，所以顶面是更负的那个 = -size。
    var s = sizeMM / 1000;
    if (s <= 0) { return 0; }                 // 边长填 0 → 退化成台面平面
    return (zsign() - 1) * s / 2;
  }

  function toBoard(u, v, sizeMM) {
    var zc = planeZ(sizeMM);
    var d = [(u - CFG.cx) / CFG.fx, (v - CFG.cy) / CFG.fy, 1.0];
    var r = CFG.R;
    var dt = [
      r[0][0] * d[0] + r[0][1] * d[1] + r[0][2] * d[2],
      r[1][0] * d[0] + r[1][1] * d[1] + r[1][2] * d[2],
      r[2][0] * d[0] + r[2][1] * d[1] + r[2][2] * d[2]
    ];
    if (Math.abs(dt[2]) < 1e-9) { return null; }
    var lam = (zc - CFG.t[2]) / dt[2];
    return [CFG.t[0] + lam * dt[0], CFG.t[1] + lam * dt[1]];
  }

  function sizeMM() {
    var s = parseFloat(sizeEl.value);
    return (isFinite(s) && s > 0) ? s : 0;
  }

  function fmt(x) { return (x >= 0 ? "+" : "") + x.toFixed(4); }

  // ---------------------------------------------------------- 坐标换算
  function toImage(ev) {
    var r = wrap.getBoundingClientRect();
    if (!r.width || !r.height) { return null; }
    return [
      (ev.clientX - r.left) / r.width * CFG.W,
      (ev.clientY - r.top) / r.height * CFG.H
    ];
  }

  function hitIndex(uv) {
    // 命中半径按显示尺寸折算，让「点掉它」在任何缩放下手感一致。
    var r = wrap.getBoundingClientRect();
    var rad = 13 * (CFG.W / (r.width || CFG.W));
    var best = -1, bestD = rad * rad;
    for (var i = 0; i < marks.length; i++) {
      var du = marks[i].u - uv[0], dv = marks[i].v - uv[1];
      var d = du * du + dv * dv;
      if (d < bestD) { bestD = d; best = i; }
    }
    return best;
  }

  // ---------------------------------------------------------- 存储
  function save() {
    try { localStorage.setItem(STORE, JSON.stringify({ marks: marks, size: sizeEl.value })); }
    catch (e) { /* 隐私模式等：存不了就算了，页面照样能用 */ }
  }
  function load() {
    try {
      var raw = localStorage.getItem(STORE);
      if (!raw) { return; }
      var doc = JSON.parse(raw);
      if (doc && Array.isArray(doc.marks)) {
        marks = doc.marks.filter(function (m) {
          return m && isFinite(m.u) && isFinite(m.v);
        });
      }
      if (doc && doc.size != null) { sizeEl.value = doc.size; }
    } catch (e) { /* 同上 */ }
  }

  // ---------------------------------------------------------- 渲染
  function render() {
    var s = sizeMM();
    var frag = document.createDocumentFragment();
    for (var i = 0; i < marks.length; i++) {
      var d = document.createElement("div");
      d.className = "mk";
      d.style.left = (marks[i].u / CFG.W * 100) + "%";
      d.style.top = (marks[i].v / CFG.H * 100) + "%";
      var tag = document.createElement("span");
      tag.textContent = String(i + 1);
      d.appendChild(tag);
      frag.appendChild(d);
    }
    overlay.replaceChildren(frag);

    var rows = document.createDocumentFragment();
    for (var j = 0; j < marks.length; j++) {
      var b = toBoard(marks[j].u, marks[j].v, s);
      var li = document.createElement("li");
      var idx = document.createElement("span");
      idx.className = "i"; idx.textContent = String(j + 1);
      var c = document.createElement("span");
      c.className = "c";
      c.textContent = b ? (fmt(b[0]) + "  " + fmt(b[1])) : "—";
      var del = document.createElement("button");
      del.type = "button";
      del.textContent = "×";
      del.title = "删掉第 " + (j + 1) + " 个";
      del.setAttribute("data-del", String(j));
      li.appendChild(idx); li.appendChild(c); li.appendChild(del);
      rows.appendChild(li);
    }
    if (!marks.length) {
      var p = document.createElement("li");
      p.className = "empty";
      p.textContent = "还没点。";
      rows.appendChild(p);
    }
    listEl.replaceChildren(rows);

    countEl.textContent = String(marks.length);
    document.getElementById("undo").disabled = !marks.length;
    document.getElementById("clear").disabled = !marks.length;

    var doc = marks.map(function (m) {
      var b = toBoard(m.u, m.v, s);
      return {
        x: b ? round(b[0], 5) : null,
        y: b ? round(b[1], 5) : null,
        z: s > 0 ? round((zsign() * s / 1000) / 2, 5) : 0,
        size: s > 0 ? round(s / 1000, 5) : null
      };
    });
    outEl.value = JSON.stringify(doc, null, 2);
    save();
  }

  function round(v, n) { var p = Math.pow(10, n); return Math.round(v * p) / p; }

  function renderHover() {
    var r = wrap.getBoundingClientRect();
    var cw = mag.width, ch = mag.height;
    mctx.fillStyle = "#000";
    mctx.fillRect(0, 0, cw, ch);

    if (hover && r.width) {
      var scale = r.width / CFG.W;             // 显示像素 / 原始像素
      var span = Math.max(12, Math.round(cw / scale / 4));   // 取 4 倍放大
      var su = Math.round(hover[0] - span / 2);
      var sv = Math.round(hover[1] - span / 2);
      var s0 = Math.max(0, su), s1 = Math.min(CFG.W, su + span);
      var t0 = Math.max(0, sv), t1 = Math.min(CFG.H, sv + span);
      if (s1 > s0 && t1 > t0 && img.complete && img.naturalWidth) {
        var dx = (s0 - su) / span * cw, dy = (t0 - sv) / span * ch;
        var dw = (s1 - s0) / span * cw, dh = (t1 - t0) / span * ch;
        mctx.imageSmoothingEnabled = false;
        mctx.drawImage(img, s0, t0, s1 - s0, t1 - t0, dx, dy, dw, dh);
      }
      // 十字线固定在正中，即光标位置
      mctx.strokeStyle = "rgba(200,64,44,0.95)";
      mctx.lineWidth = 2;
      mctx.beginPath();
      mctx.moveTo(cw / 2, 0); mctx.lineTo(cw / 2, ch);
      mctx.moveTo(0, ch / 2); mctx.lineTo(cw, ch / 2);
      mctx.stroke();
    } else {
      mctx.fillStyle = "rgba(255,255,255,0.35)";
      mctx.font = "13px sans-serif";
      mctx.textAlign = "center";
      mctx.fillText("移入画面", cw / 2, ch / 2);
    }

    var b = hover ? toBoard(hover[0], hover[1], sizeMM()) : null;
    curEl.textContent = b
      ? fmt(b[0]) + "  " + fmt(b[1]) + " m"
      : "—";
  }

  // ---------------------------------------------------------- 交互
  wrap.addEventListener("pointerdown", function (ev) {
    if (ev.button !== 0) { return; }
    var uv = toImage(ev);
    if (!uv) { return; }
    if (uv[0] < 0 || uv[1] < 0 || uv[0] > CFG.W || uv[1] > CFG.H) { return; }
    var hit = hitIndex(uv);
    if (hit >= 0) { marks.splice(hit, 1); } else { marks.push({ u: uv[0], v: uv[1] }); }
    noteEl.textContent = "";
    noteEl.className = "note";
    render();
    ev.preventDefault();
  });

  wrap.addEventListener("pointermove", function (ev) {
    hover = toImage(ev);
    renderHover();
  });
  wrap.addEventListener("pointerleave", function () {
    hover = null;
    renderHover();
  });

  listEl.addEventListener("click", function (ev) {
    var i = ev.target && ev.target.getAttribute("data-del");
    if (i == null) { return; }
    marks.splice(Number(i), 1);
    render();
  });

  document.getElementById("undo").addEventListener("click", function () {
    marks.pop();
    render();
  });
  document.getElementById("clear").addEventListener("click", function () {
    marks = [];
    render();
  });

  sizeEl.addEventListener("input", function () {
    render();
    renderHover();
  });

  document.addEventListener("keydown", function (ev) {
    if (ev.target && ev.target.tagName === "INPUT" && ev.key !== "Escape") { return; }
    if (ev.key === "Escape" || ev.key === "z" || ev.key === "Z") {
      if (!marks.length) { return; }
      marks.pop();
      render();
      ev.preventDefault();
    }
  });

  document.getElementById("copy").addEventListener("click", function () {
    var text = outEl.value;
    function fallback() {
      try {
        outEl.focus();
        outEl.select();
        noteEl.textContent = "已选中，按 Ctrl/Cmd+C 复制";
      } catch (e) {
        noteEl.textContent = "复制失败，请手动选中上面的文本";
      }
      noteEl.className = "note";
    }
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(text).then(function () {
        noteEl.textContent = "已复制 " + marks.length + " 个方块";
        noteEl.className = "note ok";
      }, fallback);
    } else {
      fallback();
    }
  });

  load();
  img.addEventListener("load", function () { renderHover(); });
  render();
  renderHover();
})();
</script>
"""


def plane_z(size_m, zsign):
    """点选平面：方块**顶面**（离相机更近的那个面）。

    与页面里 ``planeZ()`` 同一式。方块中心在 ``zsign*size/2``，两个面分别在
    ``0`` 与 ``-size``（zsign=-1 时）；相机在 ``t[2]<0`` 一侧，所以顶面是更负的
    那个 = ``-size`` = ``(zsign-1)*size/2``。

    ⚠️ 别"顺手统一"成中心平面：提案器 ``_rough_seed`` 吃的确实是中心平面，
    但它吃的是**连通域包围盒中心**（对称于立方体中心）；这里吃的是**人点的
    顶面中心**。两者差 14.25 mm，变异实测坐标整体偏十几毫米。
    """
    return (zsign - 1) * size_m / 2.0


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="生成「顶视方块点选」页面（单文件 HTML，可直接发布）")
    ap.add_argument('--info', required=True, help='相机 info.json（fx/fy/cx/cy/width/height）')
    ap.add_argument('--extrinsics', required=True,
                    help='labeler 外参 JSON（rotation/translation，target←camera）')
    ap.add_argument('--image', required=True, help='顶视画面 PNG')
    ap.add_argument('--size-mm', type=float, default=28.5, help='方块边长默认值')
    ap.add_argument('--out', required=True, help='输出 HTML 路径')
    a = ap.parse_args(argv)

    info = json.load(open(a.info))
    extr = json.load(open(a.extrinsics))
    for k in ('fx', 'fy', 'cx', 'cy'):
        if k not in info:
            sys.exit("info.json 缺 %s —— 板上的 grab_one_frame.py 只写 K/D，"
                     "要从 K 展开出 fx/fy/cx/cy" % k)
    if 'rotation' not in extr or 'translation' not in extr:
        sys.exit("外参文件缺 rotation/translation —— 该文件应是 board_extrinsics.py "
                 "cmd_labeler 的产物；cam_from_board.json 用的是 R/t，不是它")

    R = quat_to_matrix(extr['rotation'])
    t = list(extr['translation'])
    proj = L.TfProjector(info, extr)

    # 自检：模拟**人真正会点的那个像素**，看它反投回来差多少。
    #
    # ⚠️ 这里和 `propose_block_positions._rough_seed` 有一处**故意的不同**，
    #    改之前先读懂：
    #      · 提案器吃的是**颜色连通域的包围盒中心** —— 那是方块**整体**的视觉
    #        中心，对称于立方体中心，所以它反投到**中心平面** zc 上才对（实测 0.6 mm）。
    #      · 本页面吃的是**人点的顶面中心** —— 那是顶面那个亮方块的中心，
    #        所以它反投到**顶面** z_top 上才对。
    #    把两者统一成同一个平面，其中一个必然错十几毫米。别"顺手对齐"。
    #
    # 于是自检就该模拟后者：把顶面 4 个角投到像素、取它们的**像素质心**
    # （透视下它和「顶面中心的投影」差一点点 —— 这点差就是本节要量的东西）。
    size = a.size_mm / 1000.0
    zsign = 1.0 if proj.t[2] > 0 else -1.0
    h = size / 2.0
    zc = zsign * size / 2.0                   # 方块中心（labeler 投影用）
    # 顶面**独立地**从立体几何取：两个面里离相机更近的那个。
    # ⚠️ 曾经这里直接写 `z_top = plane_z(...)` 去造探针 —— 于是「被测的式子」
    #    和「造探针的式子」是同一个变量，任何平面都完美往返，自检**恒真**。
    #    （变异实测：换成中心平面后 8 PASS / exit 0，一个都没抓到。）
    z_face = min([zc - h, zc + h], key=lambda z: abs(z - t[2]))
    probe = (0.200, -0.050)
    top4 = [(probe[0] + dx, probe[1] + dy, z_face)
            for dx in (-h, h) for dy in (-h, h)]
    pix, dropped = L.project_pinhole([proj.to_camera(c) for c in top4],
                                     proj.fx, proj.fy, proj.cx, proj.cy)
    if dropped or len(pix) != 4:
        sys.exit("自检失败：探针方块投影掉出了画面，换 --size-mm 或换一帧")
    u = sum(p[0] for p in pix) / 4.0
    v = sum(p[1] for p in pix) / 4.0

    z_click = plane_z(size, zsign)
    d = [(u - proj.cx) / proj.fx, (v - proj.cy) / proj.fy, 1.0]
    dt = R @ d
    lam = (z_click - t[2]) / dt[2]
    back = t + lam * dt
    err_mm = ((back[0] - probe[0]) ** 2 + (back[1] - probe[1]) ** 2) ** 0.5 * 1000

    with open(a.image, 'rb') as f:
        b64 = base64.b64encode(f.read()).decode('ascii')

    cfg = {
        'fx': float(info['fx']), 'fy': float(info['fy']),
        'cx': float(info['cx']), 'cy': float(info['cy']),
        'W': int(info['width']), 'H': int(info['height']),
        'R': [[float(x) for x in row] for row in R],
        't': [float(x) for x in t],
        'img': 'data:image/png;base64,' + b64,
    }
    html = TEMPLATE.replace('__CFG__', json.dumps(cfg))

    with open(a.out, 'w') as f:
        f.write(html)

    print('写出 %s（%.2f MB）' % (a.out, os.path.getsize(a.out) / 1e6))
    print('  画面 %d×%d  边长默认 %.1f mm  点选平面 z=%.5f（顶面）'
          % (cfg['W'], cfg['H'], a.size_mm, z_click))
    print('  自检：探针顶面 (%.3f, %.3f, z=%.5f) → 投影 → 反投 (%.3f, %.3f)，'
          ' 差 **%.2f mm**' % (probe[0], probe[1], z_face, back[0], back[1], err_mm))
    if err_mm > 3.0:
        print('  ⚠️ 反投误差超过 3 mm —— 点选平面与探针顶面对不上（或外参与 '
              'info.json 不配套），先别用')
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
