# PNG -> LVGL converter

Converts a PNG into the `LV_IMG_CF_TRUE_COLOR_ALPHA` C asset pair (`.c`/`.h`)
this project uses, replacing the manual online-converter step in the main
README's "Graphics pipeline" section. Output format was verified byte-for-byte
against the existing `include/background.png` -> `src/tiltdash_bg.c` pair.

## Setup (once)

```bash
cd tools/graphics
python3 -m venv .venv
.venv/bin/pip install Pillow
```

## Usage

```bash
tools/graphics/.venv/bin/python3 tools/graphics/png_to_lvgl.py <input.png> <asset_name>
```

Example:

```bash
tools/graphics/.venv/bin/python3 tools/graphics/png_to_lvgl.py ~/Desktop/gauge.png tiltdash_gauge
```

Writes `src/tiltdash_gauge.c` and `include/tiltdash_gauge.h`. Then in your code:

```cpp
#include "tiltdash_gauge.h"
lv_img_set_src(obj, &tiltdash_gauge);
```

Source PNG requirements (same as the manual pipeline): PNG-24 with a real
alpha channel, not indexed/palette color. See the required dimensions table
in the main README if you're replacing an existing asset (background, side,
back images) rather than adding a new one.
