#pragma once
#include "lvgl.h"

// Reference example: opaque (LV_IMG_CF_TRUE_COLOR, no alpha channel)
// variants of the side/back assets, kept as a simpler starting point for
// anyone customizing the graphics - see the "Graphics pipeline" section
// in README.md. Not included/used by the app itself, which uses the
// alpha-channel versions in tiltdash_images_105_alpha.h instead.
extern const lv_img_dsc_t tiltdash_side_105;
extern const lv_img_dsc_t tiltdash_back_105;
