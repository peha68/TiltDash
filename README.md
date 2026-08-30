# TiltDash – ESP32-S3 AMOLED Vehicle Leveler

A project based on a Waveshare ESP32-S3 1.91" AMOLED (RM67162) + QMI8658 IMU + FT3168 touch controller.
LVGL UI: vehicle leveling (pitch/roll), calibration. Built with a van in mind, but not tied to any
specific make or model - the leveling math and calibration work for any vehicle the enclosure is
mounted in.

## Hardware
- Waveshare ESP32-S3 Touch AMOLED 1.91"
- IMU: QMI8658
- Touch: FT3168

## Build
PlatformIO:
- Board: `esp32-s3-devkitc-1`
- Framework: Arduino

## Notes
- Graphics: RGB565 (`swap16`) as `lv_img_dsc_t`
- Zero calibration: stored in `Preferences`, computed as a full 3D rotation (not a simple angle
  offset) so it correctly compensates for a compound mounting tilt (e.g. a landscape screen plus a
  tilted-back enclosure) - see `buildCalRotation()` in `src/imu.cpp`.

## WiFi setup
No SSID/password is baked into the firmware. On first boot (or when re-entering the SETUP screen -
swipe left or right from the clock screen), the device starts its own hotspot with a captive portal:
- Scan the on-screen QR code, or manually join the `TiltDash-XXXX` WiFi network.
- A page listing nearby networks opens automatically (or open `http://<shown IP>` manually);
  tap "Rescan" if the list comes up empty the first time.
- Pick your network and save (its own form/button) - the device restarts and connects as a client.
- Timezone is a separate form/button on the same page, so it can be changed without re-entering
  WiFi credentials. It's a fixed UTC offset, not automatic DST - switch it by hand twice a year.

See `include/wifi_portal.h` / `src/wifi_portal.cpp`.

## Screen navigation
Two independent horizontal loops, linked vertically:
- **Level 1**: CLOCK <-> SETUP (swipe left/right from either one)
- **Level 2**: MAIN (tilt readout) <-> CALIBRATION (swipe left/right from either one)
- **Vertical link**: CLOCK <-> MAIN (swipe down on CLOCK, swipe up on MAIN)

## Graphics pipeline (PNG -> LVGL C assets)

This project uses LVGL 8.4 on ESP32-S3 with RM67162 AMOLED display. Images are converted to LVGL C arrays and compiled into firmware.

### 0. Required image dimensions

The display runs at 536x240 (landscape, `ROTATION 1` in `src/main.cpp` - see `LCD_HOR_RES`/`LCD_VER_RES`).
If you want to personalize the artwork, replace these three source PNGs and regenerate the
corresponding `.c`/`.h` pair for each (see steps 1-4 below), keeping the exact pixel dimensions -
LVGL doesn't scale images at runtime, and the layout code positions/pivots them assuming these
sizes:

| Asset | Used for | Required size (px) | Generated files |
|---|---|---|---|
| Background | Full-screen background on every screen | 536 x 240 | `tiltdash_bg.c` / `tiltdash_bg.h` |
| Side image | Pitch/tilt indicator (splash + main screen, left zone) | 276 x 105 | `tiltdash_side_105_alpha.c` / `.h` |
| Back image | Roll/tilt indicator (main screen, right zone) | 115 x 105 | `tiltdash_back_105_alpha.c` / `.h` |

### 1. Source Format Requirements (Photoshop)
- **Format**: PNG-24 (truecolor) with transparency (alpha channel), not indexed color.
- **Color space**: sRGB, 8-bit per channel.
- **Alpha**: Keep alpha clean; avoid premultiplied alpha artifacts if possible.
- **Export settings**: Use PNG-24 with alpha. Avoid indexed PNG, palette modes, or GIF (no alpha support).
- **Avoid**: Indexed color, palette-based exports, or formats without proper alpha.

### 2. Conversion

**Option A - local script (recommended)**: `tools/graphics/png_to_lvgl.py` converts a PNG
directly to the exact format below - see `tools/graphics/README.md` for setup/usage. Its output
was verified byte-for-byte against the existing `background.png` -> `tiltdash_bg.c` pair.

**Option B - LVGL's own tools**: the online converter (https://lvgl.io/tools/imageconverter) or
`lv_img_conv` CLI:
```bash
lv_img_conv input.png --format LV_IMG_CF_TRUE_COLOR_ALPHA --output output.c
```

Either way, the target format is:
- **Output format**: `LV_IMG_CF_TRUE_COLOR_ALPHA` - 3 bytes/pixel: RGB565 (2 bytes) + alpha (1 byte).
- **RGB565 packing**: truncate each 8-bit channel (`R>>3`, `G>>2`, `B>>3`), pack as `(R5<<11)|(G6<<5)|B5`.
- **Byte order**: that 16-bit value stored **little-endian** (low byte first) - this project builds
  with `LV_COLOR_16_SWAP=0` in effect (`include/lv_conf.h` overrides the `=1` build flag in
  `platformio.ini` - the header's `#define` wins since it's processed after the command-line one).
  `convert565()` in `src/main.cpp` is unrelated to asset byte order - it only handles the small
  `flush_tmp` scratch buffer path in the display flush callback.

### 3. File Placement and Naming Convention
- Place generated `.c` files in `src/` and `.h` files in `include/` (PlatformIO structure).
- Naming pattern: `tiltdash_[component]_[variant].c/h`, e.g., `tiltdash_side_105_alpha.c/h`.

### 4. Integration into Code
- Include the header: `#include "tiltdash_side_105_alpha.h"`
- Set image source: `lv_img_set_src(img, &tiltdash_side_105_alpha);`
- **Reminder**: If colors appear wrong, ensure consistent swap16 handling—either pre-swap in assets or runtime swap, not both.

### 5. Troubleshooting Checklist
- **Rainbow colors**: Indicates wrong endianness, double swap, or incorrect CF mode.
- **Black background around image**: Alpha channel missing or wrong color key.
- **Build fails**: Check for duplicate files or invalid C array formatting.

## License

This project's own code is licensed under the MIT License - see [LICENSE](LICENSE).
Third-party libraries vendored under `lib/` (ArduinoJson, SensorLib) keep their own
licenses in their respective folders.
