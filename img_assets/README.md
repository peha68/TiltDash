# Vehicle graphics presets

Ready-made alternative graphics for the tilt display, one folder per vehicle/trailer.
Each preset provides the `.c` files that get compiled into the firmware - **not** the
`.h` headers, which don't need to change (they only declare the symbol name, independent
of which vehicle's image data is behind it - see `include/tiltdash_bg.h` and
`include/tiltdash_images_105_alpha.h`).

## Using a preset

1. Copy the `.c` file(s) from a preset folder into `src/`, overwriting the files already
   there (they share the same names on purpose - `tiltdash_side_105_alpha.c`,
   `tiltdash_back_105_alpha.c`, and optionally `tiltdash_bg.c`).
2. Rebuild and reflash (`pio run -t upload`). No other code changes needed.

A preset that only includes `tiltdash_side_105_alpha.c`/`tiltdash_back_105_alpha.c` (no
`tiltdash_bg.c`) keeps whatever background is currently in `src/` - only the vehicle
silhouette images change.

## Available presets

| Folder | Contents | Notes |
|---|---|---|
| `vw-t4/` | background + side + back | The project's original default graphics (VW T4 van) - kept here as a way back after switching to another preset. |
| `niewadow-n126nt/` | side + back (+ source PNGs) | Trailer graphics. Includes the source `.png` files (not compiled, kept for reference/re-editing). |

## Adding a new preset

Generate the `.c` files with `tools/graphics/png_to_lvgl.py` (see that tool's own
README) using these exact names so they drop straight into `src/`:
`tiltdash_side_105_alpha`, `tiltdash_back_105_alpha`, and `tiltdash_bg` if you're also
replacing the background. See the "Graphics pipeline" section in the main README for
the required source image dimensions and format.
