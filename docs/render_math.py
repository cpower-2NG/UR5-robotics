#!/usr/bin/env python3
"""将 Markdown 中的 LaTeX 数学公式渲染为 SVG 并替换。"""

import re, os, hashlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

DOC = os.path.join(os.path.dirname(__file__), 'chapter03_yolo_perception.md')
IMG_DIR = os.path.join(os.path.dirname(__file__), 'images')
os.makedirs(IMG_DIR, exist_ok=True)

DISPLAY_RE = re.compile(r'\$\$(.+?)\$\$', re.DOTALL)
# 行内 $...$：先替换了 $$ 之后剩余的单 $ 就是行内公式
INLINE_RE = re.compile(r'\$([^$]+?)\$')


def render_svg(latex, display=False):
    """用 matplotlib LaTeX 渲染公式为 SVG。"""
    h = hashlib.md5(latex.encode()).hexdigest()[:8]
    fname = f"math_{h}.svg"
    fpath = os.path.join(IMG_DIR, fname)
    if os.path.exists(fpath):
        return fname

    fontsize = 16 if display else 12
    # 用 {...} 包裹，避免 [ 被 LaTeX 误解析为可选参数
    safe_latex = f"{{{latex}}}"
    fig, ax = plt.subplots(figsize=(0.01, 0.01))
    text = ax.text(0.5, 0.5, f"${safe_latex}$", fontsize=fontsize,
                   ha='center', va='center', transform=ax.transAxes)
    ax.axis('off')
    fig.canvas.draw()
    bbox = text.get_window_extent(renderer=fig.canvas.get_renderer())
    bbox = bbox.transformed(fig.dpi_scale_trans.inverted())
    plt.close(fig)

    w, h = bbox.width + 0.2, bbox.height + 0.1
    fig, ax = plt.subplots(figsize=(w, h))
    ax.text(0.5, 0.5, f"${safe_latex}$", fontsize=fontsize,
            ha='center', va='center', transform=ax.transAxes)
    ax.axis('off')
    ax.set_xlim(0, 1); ax.set_ylim(0, 1)
    fig.savefig(fpath, format='svg', bbox_inches='tight', pad_inches=0.0,
                transparent=True, dpi=150)
    plt.close(fig)
    return fname


def main():
    with open(DOC, 'r') as f:
        content = f.read()

    # ---- 保护代码块：替换为占位符 ----
    code_blocks = {}
    code_idx = 0

    def protect_code(m):
        nonlocal code_idx
        key = f"__CODEBLOCK_{code_idx}__"
        code_blocks[key] = m.group(0)
        code_idx += 1
        return key

    content = re.sub(r'```.*?```', protect_code, content, flags=re.DOTALL)

    # ---- 替换 display math ----
    def replace_display(m):
        latex = m.group(1).strip()
        if len(latex) > 300:
            return m.group(0)
        try:
            fname = render_svg(latex, display=True)
            return f'\n<p style="text-align:center"><img src="images/{fname}" alt="{latex}" style="max-width:100%;display:inline" /></p>\n'
        except Exception as e:
            print(f"  [SKIP display] {latex[:60]} — {e}")
            return m.group(0)

    content = DISPLAY_RE.sub(replace_display, content)

    # ---- 替换 inline math ----
    def replace_inline(m):
        latex = m.group(1).strip()
        if len(latex) > 200 or '\n' in latex:
            return m.group(0)
        try:
            fname = render_svg(latex, display=False)
            # HTML img 标签，强制 inline，基线对齐
            return f'<img src="images/{fname}" alt="{latex}" style="display:inline;vertical-align:middle;max-width:none;margin:0" />'
        except Exception as e:
            print(f"  [SKIP inline] {latex[:60]} — {e}")
            return m.group(0)

    content = INLINE_RE.sub(replace_inline, content)

    # ---- 恢复代码块 ----
    for key, block in code_blocks.items():
        content = content.replace(key, block)

    out_path = DOC.replace('.md', '_svg.md')
    with open(out_path, 'w') as f:
        f.write(content)

    svgs = [f for f in os.listdir(IMG_DIR) if f.startswith('math_')]
    print(f"生成 {len(svgs)} 个 SVG 公式")
    print(f"输出: {out_path}")


if __name__ == '__main__':
    main()
