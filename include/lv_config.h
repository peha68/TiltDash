#pragma once

/* Minimal LVGL config for RGB565 displays */

#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0   /* If colors are weird (red<->blue), set to 1 */

#define LV_USE_LOG 0

/* Memory */
#define LV_MEM_CUSTOM 0

/* Enable basic widgets */
#define LV_USE_LABEL 1

/* Fonts */
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_DEFAULT &lv_font_montserrat_14
