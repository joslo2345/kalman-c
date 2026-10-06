"""Render the README benchmark charts as SVG from results/results.csv.

Usage: python scripts/make_readme_charts.py [results.csv] [out-dir]

Writes bench-desktop-{light,dark}.svg and bench-embedded-{light,dark}.svg to
docs/assets/ (by default). Values are the latest row for each cell, as in
make_table.py. Each chart is small multiples of horizontal bars: one panel per
scenario or target, each with its own scale, every bar labeled with its value.

Colors follow the library, never its rank, in the validated categorical order
(blue, orange, aqua, yellow, magenta); light and dark steps were checked with
the dataviz palette validator against GitHub's page backgrounds.
"""
import csv
import os
import sys
from collections import defaultdict

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SERIES = [  # (key, label, light, dark) in display order
    ("kalman-c", "kalman-c", "#2a78d6", "#3987e5"),
    ("kalman-c-specialized", "kalman-c (specialized)", "#eb6834", "#d95926"),
    ("kalman-c-fixed", "kalman-c (fixed-point)", "#1baf7a", "#199e70"),
    ("naive", "Textbook KF", "#eda100", "#c98500"),
    ("tinyekf", "TinyEKF", "#e87ba4", "#d55181"),
]
THEMES = {
    "light": {"ink": "#0b0b0b", "ink2": "#52514e", "muted": "#6e6c66", "base": "#c3c2b7"},
    "dark": {"ink": "#ffffff", "ink2": "#c3c2b7", "muted": "#9a9890", "base": "#383835"},
}
FONT = "system-ui, -apple-system, 'Segoe UI', Helvetica, Arial, sans-serif"

W = 860              # total width
LABEL_W = 168        # category label column
PANEL_GAP = 28
BAR_H = 14           # bar thickness (<= 24px)
ROW_H = 26
VALUE_W = 64         # room for the value label at the bar tip


def latest(csv_path):
    cells = {}
    for r in csv.DictReader(open(csv_path, newline="")):
        key = (r["library"], r["scenario"], r["filter"], r["precision"], r["metric"])
        cells[key] = float(r["value"])
    return cells


def fmt(v, unit):
    if unit == "ns" and v >= 1000:
        return f"{v / 1000:.2f} µs"
    if unit == "ns":
        return f"{v:.0f} ns"
    return f"{v:,.0f}"


def bar_path(x0, y, length, h, r=4):
    """Horizontal bar: square at the baseline (x0), 4px rounded data end."""
    r = min(r, length / 2, h / 2)
    x1 = x0 + length
    return (f"M{x0:.1f},{y:.1f} H{x1 - r:.1f} Q{x1:.1f},{y:.1f} {x1:.1f},{y + r:.1f} "
            f"V{y + h - r:.1f} Q{x1:.1f},{y + h:.1f} {x1 - r:.1f},{y + h:.1f} H{x0:.1f} Z")


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def render(title, subtitle, panels, unit, theme):
    """panels: list of (panel title, [(series key, value or None)])."""
    t = THEMES[theme]
    color = {k: (light if theme == "light" else dark) for k, _, light, dark in SERIES}
    label = {k: lab for k, lab, _, _ in SERIES}
    keys = [k for k, *_ in SERIES if any(k == s for _, rows in panels for s, v in rows if v is not None)]

    n_rows = max(sum(1 for _, v in rows if v is not None) for _, rows in panels)
    top = 104
    height = top + n_rows * ROW_H + 4
    panel_w = (W - LABEL_W - PANEL_GAP * (len(panels) - 1)) / len(panels)

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{height}" '
           f'viewBox="0 0 {W} {height}" role="img" aria-label="{esc(title)}" '
           f'font-family="{FONT}">',
           f'<title>{esc(title)}</title>',
           f'<text x="0" y="22" font-size="17" font-weight="600" fill="{t["ink"]}">{esc(title)}</text>',
           f'<text x="0" y="42" font-size="12.5" fill="{t["ink2"]}">{esc(subtitle)}</text>']

    # Legend: always present for two or more series; swatch beside ink-colored text.
    lx = 0
    for k in keys:
        out.append(f'<rect x="{lx}" y="56" width="10" height="10" rx="2" fill="{color[k]}"/>')
        out.append(f'<text x="{lx + 15}" y="65" font-size="12" fill="{t["ink2"]}">{esc(label[k])}</text>')
        lx += 15 + 6.4 * len(label[k]) + 20

    # Category labels: one per series present in any panel, in display order.
    rows_keys = [k for k in keys]
    for i, k in enumerate(rows_keys):
        y = top + i * ROW_H + BAR_H / 2 + 4
        out.append(f'<text x="{LABEL_W - 12}" y="{y:.1f}" font-size="12.5" text-anchor="end" '
                   f'fill="{t["ink2"]}">{esc(label[k])}</text>')

    for p, (ptitle, rows) in enumerate(panels):
        x0 = LABEL_W + p * (panel_w + PANEL_GAP)
        vals = dict(rows)
        vmax = max(v for v in vals.values() if v is not None)
        scale = (panel_w - VALUE_W) / vmax
        out.append(f'<text x="{x0:.1f}" y="{top - 10}" font-size="12.5" font-weight="600" '
                   f'fill="{t["ink"]}">{esc(ptitle)}</text>')
        out.append(f'<line x1="{x0:.1f}" y1="{top - 4}" x2="{x0:.1f}" '
                   f'y2="{top + len(rows_keys) * ROW_H - 8}" stroke="{t["base"]}" stroke-width="1"/>')
        for i, k in enumerate(rows_keys):
            y = top + i * ROW_H
            v = vals.get(k)
            if v is None:
                out.append(f'<text x="{x0 + 6:.1f}" y="{y + BAR_H / 2 + 4:.1f}" font-size="11.5" '
                           f'fill="{t["muted"]}">n/a</text>')
                continue
            length = max(v * scale, 2)
            out.append(f'<path d="{bar_path(x0, y, length, BAR_H)}" fill="{color[k]}">'
                       f'<title>{esc(label[k])}: {fmt(v, unit)}</title></path>')
            out.append(f'<text x="{x0 + length + 6:.1f}" y="{y + BAR_H / 2 + 4:.1f}" font-size="12" '
                       f'fill="{t["ink"]}">{fmt(v, unit)}</text>')
    out.append("</svg>")
    return "\n".join(out) + "\n"


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "results", "results.csv")
    out_dir = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, "docs", "assets")
    os.makedirs(out_dir, exist_ok=True)
    c = latest(csv_path)

    def get(lib, scen, prec, metric, filt="KF"):
        return c.get((lib, scen, filt, prec, metric))

    desktop = []
    for scen, name in (("S1", "S1 · 2 states"), ("S2", "S2 · 4 states"), ("S5", "S5 · 15 states")):
        desktop.append((name, [
            ("kalman-c", get("kalman-c", scen, "float32", "time_per_step")),
            ("kalman-c-specialized", get("kalman-c-specialized", scen, "float32", "time_per_step")),
            ("naive", get("naive", scen, "float32", "time_per_step")),
            ("tinyekf", get("tinyekf", scen, "float32", "time_per_step")),
        ]))
    embedded = []
    for suffix, name in (("", "Cortex-M4F · hardware float"), ("@m3", "Cortex-M3 · no FPU")):
        m = "instructions_per_step"
        embedded.append((name, [
            ("kalman-c", get("kalman-c", "S2", "float32" + suffix, m)),
            ("kalman-c-specialized", get("kalman-c-specialized", "S2", "float32" + suffix, m)),
            ("kalman-c-fixed", get("kalman-c", "S2", "q20" + suffix, m)),
            ("naive", get("naive", "S2", "float32" + suffix, m)),
            ("tinyekf", get("tinyekf", "S2", "float32" + suffix, m)),
        ]))

    charts = {
        "bench-desktop": ("Time per predict + update (lower is better)",
                          "float32 · Apple M3 Pro · median of 30 runs · each panel has its own scale",
                          desktop, "ns"),
        "bench-embedded": ("Instructions per predict + update on scenario S2 (lower is better)",
                           "Emulated STM32 in QEMU · exact instruction counts, not cycles",
                           embedded, "instructions"),
    }
    for name, (title, subtitle, panels, unit) in charts.items():
        for theme in THEMES:
            path = os.path.join(out_dir, f"{name}-{theme}.svg")
            with open(path, "w", newline="\n") as f:
                f.write(render(title, subtitle, panels, unit, theme))
            print(f"wrote {path}")


if __name__ == "__main__":
    main()
