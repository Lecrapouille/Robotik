# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# This file is part of RobotIK. See LICENSING.md for details.
"""Small helper to draw the RobotIK doc diagrams in one style."""
import html

STYLE = """<style>
text {font-family: sans-serif; font-size: 18px; fill: #20334b;}
.heading {font-size: 23px; font-weight: bold;}
.small {font-size: 15px;}
.note {font-size: 15px; fill: #52657a; font-style: italic;}
.box {fill: #fff; stroke: #397baf; stroke-width: 2;}
.yours {fill: #fff7e6; stroke: #c98a1b; stroke-width: 2;}
.file {fill: #eef4ea; stroke: #5f8f4e; stroke-width: 2;}
.lib {fill: #eceff3; stroke: #7a8796; stroke-width: 2;}
.future {fill: #f5f8fc; stroke: #7a8796; stroke-width: 2; stroke-dasharray: 6 4;}
.fail {fill: #fdecee; stroke: #b5345a; stroke-width: 2;}
.ok {fill: #e9f5ec; stroke: #3d8b57; stroke-width: 2;}
.line {fill: none; stroke: #52657a; stroke-width: 2;}
.dashed {fill: none; stroke: #52657a; stroke-width: 2; stroke-dasharray: 6 5;}
.arrow {fill: none; stroke: #52657a; stroke-width: 2; marker-end: url(#arrow);}
.darrow {fill: none; stroke: #52657a; stroke-width: 2; stroke-dasharray: 6 5; marker-end: url(#arrow);}
</style>"""

KIND_LEGEND = {"yours": "your code", "file": "your YAML", "box": "RobotIK",
               "lib": "third-party library", "future": "planned"}


def esc(s):
    return html.escape(str(s), quote=False)


class Svg:
    def __init__(self, w, h, title, desc, heading=None):
        self.w, self.h, self.title, self.desc = w, h, title, desc
        self.parts = []
        self.heading = heading

    def text(self, x, y, s, cls=None, anchor=None):
        a = f' class="{cls}"' if cls else ""
        t = f' text-anchor="{anchor}"' if anchor else ""
        self.parts.append(f'<text x="{x}" y="{y}"{a}{t}>{esc(s)}</text>')

    def box(self, x, y, w, h, title=None, lines=(), kind="box", center=False):
        self.parts.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="10" class="{kind}"/>')
        if center:
            n = (1 if title else 0) + len(lines)
            ty = y + h / 2 - (n - 1) * 11 + 6
            if title:
                self.text(x + w / 2, ty, title, anchor="middle"); ty += 24
            for l in lines:
                self.text(x + w / 2, ty, l, "small", anchor="middle"); ty += 20
            return
        ty = y + 30
        if title:
            self.text(x + 16, ty, title); ty += 26
        for l in lines:
            self.text(x + 16, ty, l, "small"); ty += 20

    def path(self, d, cls="arrow"):
        self.parts.append(f'<path d="{d}" class="{cls}"/>')

    def arrow(self, pts, cls="arrow"):
        d = "M" + " L".join(f"{x} {y}" for x, y in pts)
        self.path(d, cls)

    def circle(self, x, y, r, fill="#fff", stroke="#20334b"):
        self.parts.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{fill}" stroke="{stroke}" stroke-width="3"/>')

    def legend(self, kinds, x=None, y=24):
        # right-aligned row of small squares
        widths = [36 + 8.2 * len(KIND_LEGEND[k]) for k in kinds]
        x = x if x is not None else self.w - 24 - sum(widths)
        for k, wd in zip(kinds, widths):
            self.parts.append(f'<rect x="{x}" y="{y}" width="18" height="18" rx="3" class="{k}"/>')
            self.text(x + 26, y + 15, KIND_LEGEND[k], "small")
            x += wd

    def render(self):
        head = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {self.w} {self.h}" role="img" aria-labelledby="title desc">\n'
                '<!-- SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial -->\n'
                f'<title id="title">{esc(self.title)}</title>\n'
                f'<desc id="desc">{esc(self.desc)}</desc>\n'
                '<defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0 10 5 0 10Z" fill="#52657a"/></marker></defs>\n'
                + STYLE + "\n"
                f'<rect width="{self.w}" height="{self.h}" rx="16" fill="#f5f8fc"/>\n')
        if self.heading:
            head += f'<text x="32" y="40" class="heading">{esc(self.heading)}</text>\n'
        return head + "\n".join(self.parts) + "\n</svg>\n"

    def save(self, path):
        with open(path, "w") as f:
            f.write(self.render())
