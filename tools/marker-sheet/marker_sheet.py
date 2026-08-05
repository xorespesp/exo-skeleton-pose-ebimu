# /// script
# requires-python = ">=3.11"
# dependencies = ["reportlab>=4.0"]
# ///
"""Printable color-marker sheet for the sagittal color-marker detection mode.

Emits a vector PDF at exact millimetre scale: filled discs in the chosen colors, each inside a
black ring, plus a scale bar to verify the print was not resized and solid color patches to
measure through the camera.

The printed color's exact value does not have to match anything, because the detector's color
model is sampled from the camera. What the sheet has to guarantee is that every marker of one
class is the same color, that the classes sit far apart, and that the diameter is what it claims.

Run without installing:

    uv run tools/marker-sheet/marker_sheet.py --out markers.pdf
"""

from __future__ import annotations

import argparse
import datetime
import itertools
import math
import sys
from dataclasses import dataclass

from reportlab.lib.colors import Color
from reportlab.lib.pagesizes import A4, LETTER
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas as pdf_canvas

# ---------------------------------------------------------------------------
# color presets
# ---------------------------------------------------------------------------
#
# Saturated hues that are rare on lab and industrial surfaces. Reds sit next to skin tones and
# safety markings, yellows collapse toward white under warm light, greens land on safety signage,
# and blues meet denim and work clothing, so none of those are offered here.
PRESETS: dict[str, str] = {
    "magenta": "#E6007E",
    "cyan": "#00A0E9",
    "orange": "#F26522",
    "green": "#00A651",
}

# Default pair. Magenta and cyan sit in opposite halves of the a*b* plane, which is the widest
# separation among presets that also stays clear of safety signage.
DEFAULT_COLORS = ["magenta", "cyan"]

PAPER_SIZES = {"a4": A4, "letter": LETTER}


# ---------------------------------------------------------------------------
# sRGB -> CIE Lab (D65), so the sheet can report how far apart the classes sit
# ---------------------------------------------------------------------------

def _srgb_to_linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _f_lab(t: float) -> float:
    delta3 = (6.0 / 29.0) ** 3
    return t ** (1.0 / 3.0) if t > delta3 else t / (3 * (6.0 / 29.0) ** 2) + 4.0 / 29.0


def srgb_to_lab(rgb: tuple[float, float, float]) -> tuple[float, float, float]:
    """(r, g, b) in 0..1 -> (L*, a*, b*)."""
    r, g, b = (_srgb_to_linear(v) for v in rgb)

    # sRGB D65 primaries
    x = 0.4124564 * r + 0.3575761 * g + 0.1804375 * b
    y = 0.2126729 * r + 0.7151522 * g + 0.0721750 * b
    z = 0.0193339 * r + 0.1191920 * g + 0.9503041 * b

    # D65 white point
    fx, fy, fz = _f_lab(x / 0.95047), _f_lab(y / 1.00000), _f_lab(z / 1.08883)
    return (116 * fy - 16, 500 * (fx - fy), 200 * (fy - fz))


def hex_to_rgb(value: str) -> tuple[float, float, float]:
    s = value.lstrip("#")
    if len(s) != 6:
        raise ValueError(f"expected a 6-digit hex color, got '{value}'")
    return tuple(int(s[i:i + 2], 16) / 255.0 for i in (0, 2, 4))  # type: ignore[return-value]


@dataclass(frozen=True)
class MarkerColor:
    name: str
    hex: str

    @property
    def rgb(self) -> tuple[float, float, float]:
        return hex_to_rgb(self.hex)

    @property
    def lab(self) -> tuple[float, float, float]:
        return srgb_to_lab(self.rgb)

    @property
    def pdf_color(self) -> Color:
        r, g, b = self.rgb
        return Color(r, g, b)


def ab_distance(a: MarkerColor, b: MarkerColor) -> float:
    """Separation of two colors in the a*b* plane, which is where classification happens."""
    _, a1, b1 = a.lab
    _, a2, b2 = b.lab
    return math.hypot(a1 - a2, b1 - b2)


# ---------------------------------------------------------------------------
# sheet drawing
# ---------------------------------------------------------------------------

MARGIN_MM = 15.0
GAP_MM = 6.0        # clearance between neighbouring markers
CUT_GAP_MM = 1.5    # cut guide sits this far outside the black ring


def _draw_marker(c: pdf_canvas.Canvas, cx: float, cy: float,
                 diameter_mm: float, ring_mm: float, color: MarkerColor,
                 cut_guides: bool) -> None:
    """One marker at (cx, cy) in points: black ring under a filled color disc.

    The ring keeps the background from bleeding into the disc's rim through defocus, which would
    otherwise drag the measured centroid.
    """
    r_disc = diameter_mm * mm / 2.0
    r_ring = r_disc + ring_mm * mm

    if cut_guides:
        c.saveState()
        c.setStrokeColorRGB(0.72, 0.72, 0.72)
        c.setLineWidth(0.25)
        c.setDash(2, 2)
        c.circle(cx, cy, r_ring + CUT_GAP_MM * mm, stroke=1, fill=0)
        c.restoreState()

    c.setFillColorRGB(0, 0, 0)
    c.circle(cx, cy, r_ring, stroke=0, fill=1)

    c.setFillColor(color.pdf_color)
    c.circle(cx, cy, r_disc, stroke=0, fill=1)


def _draw_scale_bar(c: pdf_canvas.Canvas, x: float, y: float, length_mm: float = 100.0) -> float:
    """A ruler to confirm the print was not scaled. Returns the height consumed, in points."""
    c.setStrokeColorRGB(0, 0, 0)
    c.setLineWidth(0.6)
    c.line(x, y, x + length_mm * mm, y)

    for i in range(int(length_mm) + 1):
        if i % 10 == 0:
            tick = 3.0 * mm
        elif i % 5 == 0:
            tick = 2.0 * mm
        else:
            tick = 1.0 * mm
        tx = x + i * mm
        c.line(tx, y, tx, y + tick)

    c.setFont("Helvetica", 7)
    for i in range(0, int(length_mm) + 1, 20):
        c.drawCentredString(x + i * mm, y + 3.6 * mm, str(i))

    c.setFont("Helvetica-Bold", 8)
    c.drawString(x + (length_mm + 4) * mm, y + 0.5 * mm,
                 f"{length_mm:.0f} mm  (measure this: it must read exactly {length_mm:.0f} mm)")
    return 8.0 * mm


PATCH_MM = 18.0


def _draw_color_patches(c: pdf_canvas.Canvas, x: float, y: float,
                        colors: list[MarkerColor], patch_mm: float = PATCH_MM) -> None:
    """Solid patches to measure through the camera. `y` is the baseline: squares sit above it,
    their labels below."""
    step = (patch_mm + 8.0) * mm
    for i, color in enumerate(colors):
        px = x + i * step
        c.setFillColor(color.pdf_color)
        c.rect(px, y, patch_mm * mm, patch_mm * mm, stroke=0, fill=1)

        c.setStrokeColorRGB(0, 0, 0)
        c.setLineWidth(0.4)
        c.rect(px, y, patch_mm * mm, patch_mm * mm, stroke=1, fill=0)

        _, a_star, b_star = color.lab
        c.setFillColorRGB(0, 0, 0)
        c.setFont("Helvetica-Bold", 7)
        c.drawString(px, y - 3.2 * mm, color.name)
        c.setFont("Helvetica", 6)
        c.drawString(px, y - 6.0 * mm, f"{color.hex}  a*{a_star:+.0f} b*{b_star:+.0f}")

    # A neutral patch: the reference the runtime illumination correction reads each frame.
    px = x + len(colors) * step
    c.setFillColorRGB(0.5, 0.5, 0.5)
    c.rect(px, y, patch_mm * mm, patch_mm * mm, stroke=0, fill=1)
    c.setStrokeColorRGB(0, 0, 0)
    c.setLineWidth(0.4)
    c.rect(px, y, patch_mm * mm, patch_mm * mm, stroke=1, fill=0)
    c.setFillColorRGB(0, 0, 0)
    c.setFont("Helvetica-Bold", 7)
    c.drawString(px, y - 3.2 * mm, "grey ref")
    c.setFont("Helvetica", 6)
    c.drawString(px, y - 6.0 * mm, "#808080  illumination")


def build_sheet(path: str, colors: list[MarkerColor], diameter_mm: float, ring_mm: float,
                count: int, paper: str, label: str, cut_guides: bool) -> int:
    """Write the PDF. Returns the number of markers actually placed."""
    page_w, page_h = PAPER_SIZES[paper]
    c = pdf_canvas.Canvas(path, pagesize=(page_w, page_h))
    c.setTitle("exo-skeleton-pose color marker sheet")

    left = MARGIN_MM * mm
    usable_w = page_w - 2 * MARGIN_MM * mm
    cursor = page_h - MARGIN_MM * mm

    # --- header ---
    stamp = datetime.date.today().isoformat()
    c.setFillColorRGB(0, 0, 0)
    c.setFont("Helvetica-Bold", 12)
    cursor -= 4.5 * mm
    c.drawString(left, cursor, "exo-skeleton-pose  color marker sheet")

    c.setFont("Helvetica", 8)
    cursor -= 5.0 * mm
    spec = (f"disc {diameter_mm:g} mm   ring {ring_mm:g} mm   "
            f"colors {', '.join(cc.name for cc in colors)}   {stamp}")
    if label:
        spec = f"[{label}]   " + spec
    c.drawString(left, cursor, spec)

    c.setFont("Helvetica-Bold", 8.5)
    cursor -= 5.0 * mm
    c.setFillColorRGB(0.75, 0.0, 0.0)
    c.drawString(left, cursor, "PRINT AT 100% / ACTUAL SIZE. Do not use 'fit to page'. "
                               "Matte paper only, no gloss.")
    c.setFillColorRGB(0, 0, 0)

    # --- scale bar ---
    cursor -= 9.0 * mm
    cursor -= _draw_scale_bar(c, left, cursor)

    # --- color patches ---
    cursor -= 12.0 * mm          # clear the scale bar
    cursor -= PATCH_MM * mm      # squares are drawn upward from this baseline
    _draw_color_patches(c, left, cursor, colors)
    cursor -= 8.0 * mm           # the two label lines sit below the baseline
    cursor -= 8.0 * mm

    # --- marker grid ---
    outer = (diameter_mm + 2 * ring_mm) * mm
    pitch = outer + GAP_MM * mm
    cols = max(1, int((usable_w + GAP_MM * mm) // pitch))

    placed = 0
    for color in colors:
        c.setFont("Helvetica-Bold", 8)
        c.setFillColorRGB(0, 0, 0)
        cursor -= 4.0 * mm
        c.drawString(left, cursor, f"{color.name}  ({color.hex})")
        cursor -= 3.0 * mm

        remaining = count
        while remaining > 0:
            row_n = min(remaining, cols)
            cursor -= outer / 2.0 + 1.0 * mm
            if cursor - outer / 2.0 < MARGIN_MM * mm:
                c.showPage()
                cursor = page_h - MARGIN_MM * mm - outer / 2.0
            for i in range(row_n):
                cx = left + outer / 2.0 + i * pitch
                _draw_marker(c, cx, cursor, diameter_mm, ring_mm, color, cut_guides)
            placed += row_n
            remaining -= row_n
            cursor -= outer / 2.0 + GAP_MM * mm

        cursor -= 3.0 * mm

    c.showPage()
    c.save()
    return placed


# ---------------------------------------------------------------------------
# cli
# ---------------------------------------------------------------------------

def resolve_colors(spec: str) -> list[MarkerColor]:
    """Accepts preset names, `#RRGGBB` literals, `name=#RRGGBB` pairs, or `all`."""
    if spec.strip().lower() == "all":
        return [MarkerColor(n, h) for n, h in PRESETS.items()]

    out: list[MarkerColor] = []
    for i, token in enumerate(t.strip() for t in spec.split(",") if t.strip()):
        if "=" in token:
            name, _, value = token.partition("=")
            out.append(MarkerColor(name.strip(), value.strip()))
        elif token.startswith("#"):
            out.append(MarkerColor(f"custom{i + 1}", token))
        elif token.lower() in PRESETS:
            out.append(MarkerColor(token.lower(), PRESETS[token.lower()]))
        else:
            raise ValueError(f"unknown color '{token}'. presets: {', '.join(PRESETS)}")
    if not out:
        raise ValueError("no colors given")
    return out


def report_separation(colors: list[MarkerColor]) -> None:
    """Print each color's a*b* coordinates and the pairwise separation.

    Classification runs on the a*b* plane, so this is the number that decides whether two classes
    can be told apart. Print `--colors all`, measure the patches through the camera, and keep the
    pair that sits furthest from the background's own cloud.
    """
    print("color        hex        L*      a*      b*")
    for color in colors:
        light, a_star, b_star = color.lab
        print(f"{color.name:<12} {color.hex}  {light:6.1f}  {a_star:+6.1f}  {b_star:+6.1f}")

    if len(colors) < 2:
        return
    print("\na*b* separation (larger is easier to separate)")
    for a, b in itertools.combinations(colors, 2):
        print(f"  {a.name:>10} <-> {b.name:<10} {ab_distance(a, b):6.1f}")


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description="Generate a printable color-marker sheet (exact mm, vector PDF).")
    p.add_argument("--out", default="markers.pdf", help="output PDF path")
    p.add_argument("--diameter", type=float, default=18.0,
                   help="colored disc diameter [mm] (default: 18)")
    p.add_argument("--ring", type=float, default=1.5,
                   help="black ring width around the disc [mm] (default: 1.5)")
    p.add_argument("--colors", default=",".join(DEFAULT_COLORS),
                   help=f"comma separated; presets ({', '.join(PRESETS)}), '#RRGGBB', "
                        f"'name=#RRGGBB', or 'all' (default: {','.join(DEFAULT_COLORS)})")
    p.add_argument("--count", type=int, default=12, help="markers per color (default: 12)")
    p.add_argument("--paper", choices=sorted(PAPER_SIZES), default="a4")
    p.add_argument("--label", default="", help="site or profile name, printed on the sheet")
    p.add_argument("--no-cut-guides", action="store_true", help="omit the dashed cutting circles")
    p.add_argument("--report-only", action="store_true",
                   help="print the color separation report and exit without writing a PDF")
    args = p.parse_args(argv)

    try:
        colors = resolve_colors(args.colors)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2

    if args.diameter <= 0 or args.ring < 0 or args.count < 1:
        print("error: --diameter must be > 0, --ring >= 0, --count >= 1", file=sys.stderr)
        return 2

    report_separation(colors)

    if args.report_only:
        return 0

    placed = build_sheet(
        path=args.out,
        colors=colors,
        diameter_mm=args.diameter,
        ring_mm=args.ring,
        count=args.count,
        paper=args.paper,
        label=args.label,
        cut_guides=not args.no_cut_guides,
    )
    print(f"\nwrote {args.out}: {placed} markers, disc {args.diameter:g} mm, ring {args.ring:g} mm")
    print("Print at 100% and check the scale bar reads 100 mm before cutting.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
