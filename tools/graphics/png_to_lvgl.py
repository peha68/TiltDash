#!/usr/bin/env python3
"""Convert a PNG into an LVGL 8.x LV_IMG_CF_TRUE_COLOR_ALPHA C asset pair
(.c + .h), matching the exact byte format already used by this project's
existing generated assets (verified byte-for-byte against
include/background.png -> src/tiltdash_bg.c):

  - RGB565, truncating (not rounding) each 8-bit channel: R>>3, G>>2, B>>3.
  - Packed as (R5<<11)|(G6<<5)|B5, stored little-endian (low byte first).
  - Followed by the raw 8-bit alpha byte -> 3 bytes/pixel, row-major,
    top-to-bottom / left-to-right.

Usage:
    .venv/bin/python3 png_to_lvgl.py <input.png> <asset_name> [--src-dir DIR] [--include-dir DIR]

Example:
    .venv/bin/python3 png_to_lvgl.py ~/Desktop/gauge.png tiltdash_gauge

Writes src/<asset_name>.c and include/<asset_name>.h (paths relative to the
project root by default), declaring `extern const lv_img_dsc_t <asset_name>;`
Add `#include "<asset_name>.h"` and call `lv_img_set_src(obj, &<asset_name>);`
to use it - see the "Graphics pipeline" section in README.md.
"""
import argparse
import pathlib
import sys

try:
    from PIL import Image
except ImportError:
    sys.exit("Pillow is required: run 'pip install Pillow' in this venv first "
              "(see tools/graphics/.venv or create your own).")

PIXELS_PER_LINE = 8  # matches the formatting of existing generated files


def convert(png_path: pathlib.Path, name: str, src_dir: pathlib.Path, include_dir: pathlib.Path):
    img = Image.open(png_path).convert("RGBA")
    w, h = img.size
    pixels = list(img.getdata())

    data_bytes = []
    for (r, g, b, a) in pixels:
        rgb565 = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
        data_bytes.append(rgb565 & 0xFF)         # low byte first (little-endian)
        data_bytes.append((rgb565 >> 8) & 0xFF)  # high byte
        data_bytes.append(a)

    data_size = w * h * 3
    assert len(data_bytes) == data_size

    src_dir.mkdir(parents=True, exist_ok=True)
    include_dir.mkdir(parents=True, exist_ok=True)
    c_path = src_dir / f"{name}.c"
    h_path = include_dir / f"{name}.h"

    lines = []
    for i in range(0, len(data_bytes), PIXELS_PER_LINE * 3):
        chunk = data_bytes[i:i + PIXELS_PER_LINE * 3]
        lines.append("  " + ", ".join(f"0x{b:02X}" for b in chunk) + ",")

    c_content = (
        '#include "lvgl.h"\n'
        f"// Auto-generated from {png_path.name} ({w}x{h}) by tools/graphics/png_to_lvgl.py\n"
        f"static const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_LARGE_CONST uint8_t {name}_map[] = {{\n"
        + "\n".join(lines) + "\n"
        "};\n\n"
        f"const lv_img_dsc_t {name} = {{\n"
        "  .header = {\n"
        "    .always_zero = 0,\n"
        f"    .w = {w},\n"
        f"    .h = {h},\n"
        "    .cf = LV_IMG_CF_TRUE_COLOR_ALPHA,\n"
        "  },\n"
        f"  .data_size = {data_size},\n"
        f"  .data = {name}_map,\n"
        "};\n"
    )
    h_content = (
        "#pragma once\n"
        '#include "lvgl.h"\n\n'
        f"extern const lv_img_dsc_t {name};\n"
    )

    c_path.write_text(c_content)
    h_path.write_text(h_content)
    return c_path, h_path, w, h, data_size


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("png", type=pathlib.Path, help="Source PNG (RGBA recommended)")
    ap.add_argument("name", help="Asset name, e.g. tiltdash_gauge (used for the C symbol and file names)")
    root = pathlib.Path(__file__).resolve().parents[2]  # tools/graphics/ -> project root
    ap.add_argument("--src-dir", type=pathlib.Path, default=root / "src")
    ap.add_argument("--include-dir", type=pathlib.Path, default=root / "include")
    args = ap.parse_args()

    if not args.png.exists():
        sys.exit(f"Not found: {args.png}")

    c_path, h_path, w, h, data_size = convert(args.png, args.name, args.src_dir, args.include_dir)
    print(f"OK: {w}x{h} -> {data_size} bytes (flash-resident C array, not RAM)")
    print(f"  {c_path}")
    print(f"  {h_path}")
    print(f'Add: #include "{args.name}.h"  and  lv_img_set_src(obj, &{args.name});')


if __name__ == "__main__":
    main()
