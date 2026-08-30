#include <Arduino.h>
#include <math.h>

#include "rm67162.h"

#define LV_CONF_INCLUDE_SIMPLE 1
#include "lv_conf.h"
#include <lvgl.h>

#include "esp_timer.h"
#include <Wire.h>
#include <Preferences.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

#include "SensorQMI8658.hpp"
#include "FT3168.h"
#include "imu.h"
#include "wifi_portal.h"
#include "qrcode.h"
#include <esp_heap_caps.h>

#include "tiltdash_images_105_alpha.h"   // 105px assets with transparent background (TRUE_COLOR_ALPHA)
#include "tiltdash_bg.h"                 // background with alpha channel support (TRUE_COLOR_ALPHA)

// ================== SETTINGS ==================
#define ROTATION 1
#define INVERT_PITCH 0
#define INVERT_ROLL  0

#define TOUCH_SWAP_XY   1
#define TOUCH_INVERT_X  0
#define TOUCH_INVERT_Y  0

// WiFi / NTP
// SSID/password are NOT baked into the firmware - see wifi_portal.h/.cpp:
// loaded from NVS, set via the configuration portal (hotspot + web page),
// so changing networks doesn't require a recompile.

// Timezone offset is NOT fixed at compile time - it's loaded from NVS
// (set via the portal's "Timezone" field, see wifi_portal.cpp) and applied
// to timeClient in setup() via setTimeOffset(). This constant is only the
// value the NTPClient object below is constructed with before that NVS
// value is available; it's immediately overridden.
static constexpr int32_t  TZ_OFFSET_SEC_DEFAULT = 3600;   // UTC+1
static constexpr uint32_t WIFI_RETRY_MS     = 15000;  // retry WiFi connect every 15s
static constexpr uint32_t NTP_RETRY_MS      = 5000;   // try NTP update every 5s while connected

// WiFi icon blink
static constexpr uint32_t WIFI_ICON_BLINK_MS = 350;

// Splash side image animation
static constexpr int      SPLASH_SIDE_ZOOM          = 250;
static constexpr uint32_t SPLASH_ENTER_TIME_MS      = 900;
static constexpr uint32_t SPLASH_EXIT_TIME_MS       = 650;
static constexpr int      SPLASH_BOTTOM_MARGIN_PX   = 10;
static constexpr int      SPLASH_ENTER_OFFSCREEN_PX = 20;
static constexpr int      SPLASH_EXIT_OFFSCREEN_PX  = 40;

// --- BOUNCE (nice-to-have flourish) ---
static constexpr int      SPLASH_BOUNCE_PX          = 6;
static constexpr uint32_t SPLASH_BOUNCE_BACK_MS     = 160;

// Flourish: parallax + fade
static constexpr int      SPLASH_BG_PARALLAX_ENTER_PX = 14;
static constexpr int      SPLASH_BG_PARALLAX_EXIT_PX  = 22;
static constexpr uint32_t SPLASH_FADE_IN_MS           = 650;
static constexpr uint32_t SPLASH_FADE_OUT_MS          = 450;

// Clock fade-in after screen switch (helps avoid stutter)
static constexpr uint32_t CLOCK_FADE_DELAY_MS         = 150;

// Display angle range for car images
static constexpr float PITCH_MAX_DEG = 90.0f;
static constexpr float ROLL_MAX_DEG  = 90.0f;

// Image rotation direction
static constexpr int PITCH_SIGN = -1;
static constexpr int ROLL_SIGN  = +1;

// Dynamic accel indicators direction (tune if needed)
static constexpr int LONG_SIGN  = +1;
static constexpr int LAT_SIGN   = +1;

static constexpr int IMG_ZOOM_SIDE = 250;
static constexpr int IMG_ZOOM_BACK = 250;

// Layout split (left/right images)
static constexpr float SPLIT_RATIO = 0.70f;

static inline lv_color_t ral7037() { return lv_color_make(123, 125, 125); }

static constexpr int IMG_Y_OFFSET         = 0;
static constexpr int IMG_RIGHT_X_OFFSET   = 12;
static constexpr int LABEL_Y_PAD          = 10;

static constexpr int LABEL_LEFT_X_OFFSET  = 15;
static constexpr int LABEL_RIGHT_X_OFFSET = 15;

// UI update
static constexpr uint32_t UI_PERIOD_MS  = 100;
static constexpr uint32_t IMU_PERIOD_MS = 10;
// Each unit here costs LCD_HOR_RES*2 bytes THREE times over (buf1, buf2,
// and the flush_tmp color-conversion scratch buffer all scale with it) -
// at the previous value of 50 (536px-wide landscape display) that was
// 160800 bytes, ~49% of all internal RAM, before WiFi/LVGL's own pool/
// anything else got a byte. That starved heap is what caused the whole
// chain of WiFi AP/scan/portal failures earlier - not the WiFi code
// itself. 16 trades a bit of flush-call overhead (more, smaller chunks)
// for ~86KB back.
static constexpr int DRAWBUF_LINES = 16;

// --- G-meter indicators ---
static constexpr float GMETER_FS_G = 0.40f;

static constexpr float IND_W_RATIO = 0.80f;
static constexpr int   IND_H       = 6;
static constexpr int   IND_GAP_Y   = 8;
static constexpr int   IND_PAD_B   = 10;
// =============================================

#if ROTATION == 0
static constexpr int LCD_HOR_RES = 240;
static constexpr int LCD_VER_RES = 536;
#elif ROTATION == 1
static constexpr int LCD_HOR_RES = 536;
static constexpr int LCD_VER_RES = 240;
#else
#error "Use ROTATION 0 or 1"
#endif

static constexpr int I2C_SDA_PIN = 40;
static constexpr int I2C_SCL_PIN = 39;

// ====== LVGL tick (1ms) ======
static void lv_tick_cb(void*) { lv_tick_inc(1); }

static void lvgl_init_tick() {
  const esp_timer_create_args_t args = {
    .callback = &lv_tick_cb,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "lv_tick"
  };
  esp_timer_handle_t timer;
  esp_timer_create(&args, &timer);
  esp_timer_start_periodic(timer, 1000);
}

// ====== Preferences ======
static Preferences prefs;

// ====== WiFi and NTP ======
static WiFiUDP ntpUDP;
static NTPClient timeClient(ntpUDP, "pool.ntp.org", TZ_OFFSET_SEC_DEFAULT, 60000);

static bool     g_wifi_ok = false;
static bool     g_ntp_ok  = false;
static uint32_t g_last_wifi_try_ms = 0;
static uint32_t g_last_ntp_try_ms  = 0;

// --- splash timeout for offline mode ---
static uint32_t g_boot_ms = 0;
static bool     g_splash_forced_exit = false;
static constexpr uint32_t SPLASH_MAX_WAIT_MS = 8000; // 8s offline splash timeout

// ====== LCD color mode (runtime) ======
static bool g_byteSwap = true;
static bool g_bgrMode  = false;

static inline uint16_t swap_bytes_16(uint16_t v) { return (uint16_t)((v << 8) | (v >> 8)); }

static inline uint16_t rgb565_to_bgr565(uint16_t v) {
  uint16_t r = (v >> 11) & 0x1F;
  uint16_t g = (v >> 5)  & 0x3F;
  uint16_t b = (v >> 0)  & 0x1F;
  return (uint16_t)((b << 11) | (g << 5) | (r << 0));
}

static inline uint16_t convert565(uint16_t v)
{
  if (g_bgrMode)  v = rgb565_to_bgr565(v);
  if (g_byteSwap) v = swap_bytes_16(v);
  return v;
}

static void loadColorMode()
{
  prefs.begin("lcdmode", true);
  g_byteSwap = prefs.getBool("bs", true);
  g_bgrMode  = prefs.getBool("bgr", false);
  prefs.end();
}

// ====== Display flush ======
static uint16_t flush_tmp[LCD_HOR_RES * DRAWBUF_LINES];

static void my_flush_cb(lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p)
{
  const int32_t w = (area->x2 - area->x1 + 1);
  const int32_t h = (area->y2 - area->y1 + 1);
  const uint32_t n = (uint32_t)(w * h);

  if (n > (uint32_t)(LCD_HOR_RES * DRAWBUF_LINES)) {
    lcd_PushColors(area->x1, area->y1, w, h, (uint16_t*)color_p);
    lv_disp_flush_ready(disp_drv);
    return;
  }

  uint16_t* src = (uint16_t*)color_p;
  for (uint32_t i = 0; i < n; i++) flush_tmp[i] = convert565(src[i]);

  lcd_PushColors(area->x1, area->y1, w, h, flush_tmp);
  lv_disp_flush_ready(disp_drv);
}

// ====== Helpers (general) ======
static float clampf(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

// ====== Touch (FT3168) ======
static FT3168 tp(I2C_SDA_PIN, I2C_SCL_PIN, -1, 41);

static void map_touch(uint16_t rx, uint16_t ry, lv_coord_t &sx, lv_coord_t &sy)
{
  int x = (int)rx;
  int y = (int)ry;

#if TOUCH_SWAP_XY
  int tt = x; x = y; y = tt;
#endif
#if TOUCH_INVERT_X
  x = LCD_HOR_RES - 1 - x;
#endif
#if TOUCH_INVERT_Y
  y = LCD_VER_RES - 1 - y;
#endif

  if (x < 0) x = 0;
  if (y < 0) y = 0;
  if (x >= LCD_HOR_RES) x = LCD_HOR_RES - 1;
  if (y >= LCD_VER_RES) y = LCD_VER_RES - 1;

  sx = (lv_coord_t)x;
  sy = (lv_coord_t)y;
}

static void touch_read_cb(lv_indev_drv_t*, lv_indev_data_t* data)
{
  uint16_t x = 0, y = 0;
  uint8_t g = 0;

  bool pressed = tp.getTouch(&x, &y, &g);
  if (pressed) {
    lv_coord_t sx, sy;
    map_touch(x, y, sx, sy);
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = sx;
    data->point.y = sy;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

// ====== UI: screens ======
static lv_obj_t* scr_splash = nullptr;
static lv_obj_t* scr_clock  = nullptr;
static lv_obj_t* scr_main   = nullptr;
static lv_obj_t* scr_cal    = nullptr;

// Splash objects
static lv_obj_t* splash_bg       = nullptr;
static lv_obj_t* splash_wifi     = nullptr;
static lv_obj_t* splash_side_img = nullptr;
static lv_obj_t* splash_offline  = nullptr;

// Clock objects
static lv_obj_t* clock_bg     = nullptr;
static lv_obj_t* clock_time   = nullptr;
static lv_obj_t* clock_date   = nullptr;
static lv_obj_t* clock_wifi   = nullptr;
static lv_obj_t* clock_offline = nullptr;

// Gyro objects
static lv_obj_t* gyro_bg      = nullptr;
static lv_obj_t* gyro_wifi    = nullptr;
static lv_obj_t* gyro_offline = nullptr;

static lv_obj_t* zoneLeft  = nullptr;
static lv_obj_t* zoneRight = nullptr;

static lv_obj_t* imgPitch = nullptr;
static lv_obj_t* imgRoll  = nullptr;
static lv_obj_t* lblPitch = nullptr;
static lv_obj_t* lblRoll  = nullptr;

static lv_obj_t* accBarCont = nullptr;
static lv_obj_t* latBarCont = nullptr;
static lv_obj_t* accBarFill = nullptr;
static lv_obj_t* latBarFill = nullptr;

// Calibration objects
static lv_obj_t* cal_bg      = nullptr;
static lv_obj_t* calInfo     = nullptr;
static lv_obj_t* cal_wifi    = nullptr;
static lv_obj_t* cal_offline = nullptr;

// Setup (WiFi portal) objects
static lv_obj_t* scr_setup      = nullptr;
static lv_obj_t* setup_ssid_lbl = nullptr;
static lv_obj_t* setup_ip_lbl   = nullptr;

// WiFi blinking (opacity)
static lv_timer_t* g_wifi_blink_timer = nullptr;
static bool g_wifi_blink_on = true;

// Splash animation state
static bool g_splash_enter_started = false;
static bool g_splash_exit_started  = false;

// Parallax base X
static int g_splash_bg_base_x = 0;

// Side target geometry
static int g_splash_side_target_x = 0;
static int g_splash_side_target_y = 0;
static int g_splash_side_w        = 0;

// Clock flow
static bool g_clock_shown_after_ntp = false;
static bool g_clock_faded_in        = false;

enum Screen { SCR_SPLASH, SCR_CLOCK, SCR_MAIN, SCR_CAL, SCR_SETUP };
static Screen current_screen = SCR_SPLASH;

// ====== Active IMU policy ======
static inline bool imu_should_run()
{
  return (current_screen == SCR_MAIN) || (current_screen == SCR_CAL);
}

// Defined further down (WiFi/NTP section and the SETUP screen) - forward
// declarations so switch_screen() can call them when entering/leaving the
// SETUP screen.
static void wifi_begin_nonblocking();
static void setup_screen_on_enter();
static void setup_screen_on_leave();

// ====== Helpers ======
static void switch_screen(Screen new_screen) {
  lv_obj_t* new_scr = nullptr;
  switch (new_screen) {
    case SCR_SPLASH: new_scr = scr_splash; break;
    case SCR_CLOCK:  new_scr = scr_clock;  break;
    case SCR_MAIN:   new_scr = scr_main;   break;
    case SCR_CAL:    new_scr = scr_cal;    break;
    case SCR_SETUP:  new_scr = scr_setup;  break;
  }
  if (!new_scr) return;

  if (current_screen == SCR_SETUP && new_screen != SCR_SETUP) {
    setup_screen_on_leave();
  }

  lv_scr_load(new_scr);
  current_screen = new_screen;

  if (new_screen == SCR_SETUP) {
    setup_screen_on_enter();
  }
}

static void ui_wifi_icons_set_opa(lv_opa_t opa)
{
  lv_obj_t* icons[] = {splash_wifi, clock_wifi, gyro_wifi, cal_wifi};
  for (auto* ic : icons) {
    if (!ic) continue;
    lv_obj_set_style_opa(ic, opa, 0);
  }
}

static void ui_offline_labels_set_opa(lv_opa_t opa)
{
  lv_obj_t* labels[] = {splash_offline, clock_offline, gyro_offline, cal_offline};
  for (auto* lb : labels) {
    if (!lb) continue;
    lv_obj_set_style_opa(lb, opa, 0);
  }
}

static void wifi_blink_cb(lv_timer_t* t)
{
  (void)t;

  if (g_wifi_ok) {
    // Connected: wifi icon solid, offline labels hidden
    ui_wifi_icons_set_opa(LV_OPA_COVER);
    ui_offline_labels_set_opa(LV_OPA_0);
    g_wifi_blink_on = true;
    return;
  }

  // Offline: blink Wi-Fi, offline labels are handled in wifi_ntp_update_state()
  g_wifi_blink_on = !g_wifi_blink_on;
  ui_wifi_icons_set_opa(g_wifi_blink_on ? LV_OPA_COVER : LV_OPA_0);
}

static void update_time_labels(const char* time_str, const char* date_str)
{
  if (clock_time)  lv_label_set_text(clock_time,  time_str);
  if (clock_date)  lv_label_set_text(clock_date,  date_str);
}

static void update_time_from_ntp()
{
  char time_str[6]  = {0};
  char date_str[11] = {0};

  time_t epoch = (time_t)timeClient.getEpochTime();
  struct tm t;
  gmtime_r(&epoch, &t);

  snprintf(time_str, sizeof(time_str), "%02d:%02d", t.tm_hour, t.tm_min);
  snprintf(date_str, sizeof(date_str), "%04d-%02d-%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);

  update_time_labels(time_str, date_str);
}

static lv_obj_t* make_zone(lv_obj_t* parent, int x, int y, int w, int h)
{
  lv_obj_t* z = lv_obj_create(parent);
  lv_obj_set_pos(z, x, y);
  lv_obj_set_size(z, w, h);

  lv_obj_set_style_bg_opa(z, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(z, 0, 0);
  lv_obj_set_style_radius(z, 0, 0);
  lv_obj_set_style_pad_all(z, 0, 0);
  lv_obj_set_style_clip_corner(z, true, 0);

  lv_obj_set_style_outline_width(z, 0, 0);
  lv_obj_set_style_shadow_width(z, 0, 0);
  lv_obj_set_style_line_width(z, 0, 0);

  return z;
}

static void make_indicator(lv_obj_t* parent, int y, lv_obj_t** outCont, lv_obj_t** outFill)
{
  const int barW = (int)lroundf((float)LCD_HOR_RES * IND_W_RATIO);
  const int barX = (LCD_HOR_RES - barW) / 2;

  lv_obj_t* cont = lv_obj_create(parent);
  lv_obj_set_pos(cont, barX, y);
  lv_obj_set_size(cont, barW, IND_H);
  lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(cont, 0, 0);
  lv_obj_set_style_pad_all(cont, 0, 0);
  lv_obj_set_style_outline_width(cont, 0, 0);
  lv_obj_set_style_shadow_width(cont, 0, 0);

  lv_obj_t* track = lv_obj_create(cont);
  lv_obj_set_size(track, barW, IND_H);
  lv_obj_align(track, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_radius(track, IND_H / 2, 0);
  lv_obj_set_style_bg_color(track, lv_color_make(40, 40, 40), 0);
  lv_obj_set_style_bg_opa(track, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(track, 0, 0);
  lv_obj_set_style_outline_width(track, 0, 0);
  lv_obj_set_style_shadow_width(track, 0, 0);

  lv_obj_t* zero = lv_obj_create(cont);
  lv_obj_set_size(zero, 2, IND_H + 6);
  lv_obj_set_style_bg_color(zero, ral7037(), 0);
  lv_obj_set_style_bg_opa(zero, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(zero, 0, 0);
  lv_obj_set_style_outline_width(zero, 0, 0);
  lv_obj_set_style_shadow_width(zero, 0, 0);
  lv_obj_align(zero, LV_ALIGN_CENTER, 0, 0);

  lv_obj_t* fill = lv_obj_create(cont);
  lv_obj_set_style_radius(fill, IND_H / 2, 0);
  lv_obj_set_style_bg_color(fill, ral7037(), 0);
  lv_obj_set_style_bg_opa(fill, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(fill, 0, 0);
  lv_obj_set_style_outline_width(fill, 0, 0);
  lv_obj_set_style_shadow_width(fill, 0, 0);
  lv_obj_set_size(fill, 0, IND_H);
  lv_obj_align(fill, LV_ALIGN_CENTER, 0, 0);

  *outCont = cont;
  *outFill = fill;
}

static void indicator_set_g(lv_obj_t* barCont, lv_obj_t* fill, float valG)
{
  if (!barCont || !fill) return;

  const int w = lv_obj_get_width(barCont);
  const int h = lv_obj_get_height(barCont);

  float v = clampf(valG, -GMETER_FS_G, GMETER_FS_G);
  const int half = w / 2;
  const int px = (int)lroundf((fabsf(v) / GMETER_FS_G) * (float)half);

  if (px <= 0) {
    lv_obj_set_size(fill, 0, h);
    lv_obj_align(fill, LV_ALIGN_CENTER, 0, 0);
    return;
  }

  lv_obj_set_size(fill, px, h);
  if (v >= 0.0f) lv_obj_align(fill, LV_ALIGN_CENTER,  px / 2, 0);
  else           lv_obj_align(fill, LV_ALIGN_CENTER, -px / 2, 0);
}

static void ui_update(float pitchDeg, float rollDeg, float long_g, float lat_g)
{
  if (!imgPitch || !imgRoll) return;

  pitchDeg = clampf(pitchDeg, -PITCH_MAX_DEG, PITCH_MAX_DEG);
  rollDeg  = clampf(rollDeg,  -ROLL_MAX_DEG,  ROLL_MAX_DEG);

  lv_img_set_angle(imgPitch, (int16_t)(PITCH_SIGN * pitchDeg * 10.0f));
  lv_img_set_angle(imgRoll,  (int16_t)(ROLL_SIGN  * rollDeg  * 10.0f));

  // Pitch readout sign is flipped relative to the raw value/graphic angle
  // above: nose-down/downhill reads negative, nose-up/uphill reads
  // positive. Only the displayed number - not the image rotation angle,
  // which already tilts the correct visual direction.
  const int p = (int)lroundf(-pitchDeg);
  // Roll readout is a magnitude only (always shown as positive) - the
  // side/back graphic's tilt direction is what tells you left vs right,
  // via ROLL_SIGN * rollDeg above, which stays signed.
  const int r = (int)lroundf(fabsf(rollDeg));

  static char lp[8], lr[8];
  snprintf(lp, sizeof(lp), "%d", p);
  snprintf(lr, sizeof(lr), "%d", r);

  if (lblPitch) lv_label_set_text(lblPitch, lp);
  if (lblRoll)  lv_label_set_text(lblRoll,  lr);

  indicator_set_g(accBarCont, accBarFill, long_g);
  indicator_set_g(latBarCont, latBarFill, lat_g);
}

static void cal_btn_event_cb(lv_event_t* e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

  if (!imu_calibrate_zero()) {
    if (calInfo) lv_label_set_text(calInfo, "No IMU data (try again).");
  } else {
    if (calInfo) lv_label_set_text(calInfo, "ZERO saved. Swipe to return.");
  }
}

static void gesture_event_cb(lv_event_t* e)
{
  (void)e;
  lv_indev_t* indev = lv_indev_get_act();
  if (!indev) return;

  lv_dir_t dir = lv_indev_get_gesture_dir(indev);

  // Two independent horizontal loops, linked only vertically:
  //   level 1 (top):    CLOCK <-> SETUP
  //   level 2 (bottom): MAIN  <-> CAL
  //   vertical link:    CLOCK <-> MAIN
  // Each loop has just two screens, so either horizontal direction
  // toggles between them - there's no meaningful "forward"/"backward"
  // with only two stops.
  switch (current_screen) {
    case SCR_CLOCK:
      if (dir == LV_DIR_TOP) switch_screen(SCR_MAIN);
      else if (dir == LV_DIR_LEFT || dir == LV_DIR_RIGHT) switch_screen(SCR_SETUP);
      break;

    case SCR_MAIN:
      if (dir == LV_DIR_BOTTOM) switch_screen(SCR_CLOCK);
      else if (dir == LV_DIR_LEFT || dir == LV_DIR_RIGHT) switch_screen(SCR_CAL);
      break;

    case SCR_CAL:
      if (dir == LV_DIR_LEFT || dir == LV_DIR_RIGHT) switch_screen(SCR_MAIN);
      break;

    case SCR_SETUP:
      if (dir == LV_DIR_LEFT || dir == LV_DIR_RIGHT) switch_screen(SCR_CLOCK);
      break;

    default:
      break;
  }
}

// ====== WiFi / NTP state machine ======
static void wifi_begin_nonblocking()
{
  // If the saved network is unreachable (out of range, wrong password),
  // this gets called again every WIFI_RETRY_MS while the STA is still
  // mid-negotiation from the previous attempt ("sta is connecting,
  // return error" in the log). Layering WiFi.begin() on top of that again
  // and again - rather than fully resetting first - leaves the radio in
  // a state where WiFi.scanNetworks() reliably finds 0 networks later
  // (confirmed: a scan right at boot, before this has ever run, works
  // fine; one from the setup portal after the device has been retrying
  // for a while does not). Forcing STA fully down and back up on every
  // retry keeps it in the same clean state a fresh boot starts from.
  WiFi.disconnect(true /* wifioff */, false);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(wifi_portal_get_ssid(), wifi_portal_get_pass());
  g_last_wifi_try_ms = millis();
}

static void wifi_ntp_init()
{
  wifi_begin_nonblocking();
  timeClient.begin();
  g_last_ntp_try_ms = millis();
}

static void wifi_ntp_update_state()
{
  const uint32_t now = millis();

  if (WiFi.status() == WL_CONNECTED) {
    if (!g_wifi_ok) {
      g_wifi_ok = true;
      g_ntp_ok  = false;  // time may not be set yet
      Serial.printf("[WIFI] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
      // Hide OFFLINE labels when Wi-Fi becomes OK
      ui_offline_labels_set_opa(LV_OPA_0);
    }

    if (!timeClient.isTimeSet()) {
      if ((now - g_last_ntp_try_ms) >= NTP_RETRY_MS) {
        g_last_ntp_try_ms = now;
        bool ok = timeClient.update();
        if (ok && timeClient.isTimeSet()) {
          g_ntp_ok = true;
          Serial.printf("[NTP] Time set. tz offset=%d min, formatted=%s, raw epoch(+offset)=%lu\n",
                        wifi_portal_get_tz_offset_min(), timeClient.getFormattedTime().c_str(),
                        (unsigned long)timeClient.getEpochTime());
        }
      }
    } else {
      g_ntp_ok = true;
      timeClient.update();
    }
  } else {
    if (g_wifi_ok) {
      g_wifi_ok = false;
      g_ntp_ok  = false;
      Serial.println("[WIFI] Disconnected.");
      // Show OFFLINE labels when Wi-Fi goes down
      ui_offline_labels_set_opa(LV_OPA_COVER);
    }

    if ((now - g_last_wifi_try_ms) >= WIFI_RETRY_MS) {
      Serial.println("[WIFI] Retry connect...");
      wifi_begin_nonblocking();
    }
  }
}

static lv_obj_t* make_wifi_icon(lv_obj_t* parent)
{
  lv_obj_t* ic = lv_label_create(parent);
  lv_label_set_text(ic, LV_SYMBOL_WIFI);
  lv_obj_set_style_text_color(ic, lv_color_white(), 0);
  lv_obj_set_style_text_font(ic, &lv_font_montserrat_14, 0);
  lv_obj_align(ic, LV_ALIGN_TOP_LEFT, 8, 8);
  lv_obj_set_style_opa(ic, LV_OPA_COVER, 0);
  return ic;
}

static lv_obj_t* make_offline_label(lv_obj_t* parent)
{
  lv_obj_t* lb = lv_label_create(parent);
  lv_label_set_text(lb, "OFFLINE");
  lv_obj_set_style_text_color(lb, lv_color_hex(0xFF4040), 0); // red
  lv_obj_set_style_text_font(lb, &lv_font_montserrat_14, 0);
  lv_obj_align(lb, LV_ALIGN_TOP_LEFT, 40, 8);
  lv_obj_set_style_opa(lb, LV_OPA_0, 0); // hidden by default
  return lb;
}

// ===== Splash animation helpers =====
static void anim_set_opa(void* obj, int32_t v)
{
  if (!obj) return;
  lv_obj_set_style_opa((lv_obj_t*)obj, (lv_opa_t)v, 0);
}

static void splash_side_calc_target(int& out_w, int& out_h, int& out_x_left, int& out_y_top)
{
  const int srcW = (int)tiltdash_side_105_alpha.header.w;
  const int srcH = (int)tiltdash_side_105_alpha.header.h;

  out_w = (srcW * SPLASH_SIDE_ZOOM) / 256;
  out_h = (srcH * SPLASH_SIDE_ZOOM) / 256;

  out_x_left = (LCD_HOR_RES / 2) - (out_w / 2);
  out_y_top  = (LCD_VER_RES - SPLASH_BOTTOM_MARGIN_PX) - out_h;
}

static void splash_start_bg_parallax_enter()
{
  if (!splash_bg) return;

  const int x_end = g_splash_bg_base_x;
  const int x_start = g_splash_bg_base_x + SPLASH_BG_PARALLAX_ENTER_PX;
  lv_obj_set_x(splash_bg, x_start);

  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, splash_bg);
  lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_x);
  lv_anim_set_values(&a, x_start, x_end);
  lv_anim_set_time(&a, SPLASH_ENTER_TIME_MS);
  lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
  lv_anim_start(&a);
}

static void splash_start_bg_parallax_exit()
{
  if (!splash_bg) return;

  const int x_start = lv_obj_get_x(splash_bg);
  const int x_end   = g_splash_bg_base_x - SPLASH_BG_PARALLAX_EXIT_PX;

  lv_anim_t a;
  lv_anim_init(&a);
  lv_anim_set_var(&a, splash_bg);
  lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_x);
  lv_anim_set_values(&a, x_start, x_end);
  lv_anim_set_time(&a, SPLASH_EXIT_TIME_MS);
  lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
  lv_anim_start(&a);
}

static void splash_side_bounce_back_start(lv_anim_t* a)
{
  (void)a;
  if (!splash_side_img) return;

  const int x_from = lv_obj_get_x(splash_side_img);
  const int x_to   = g_splash_side_target_x;

  lv_anim_t ax2;
  lv_anim_init(&ax2);
  lv_anim_set_var(&ax2, splash_side_img);
  lv_anim_set_exec_cb(&ax2, (lv_anim_exec_xcb_t)lv_obj_set_x);
  lv_anim_set_values(&ax2, x_from, x_to);
  lv_anim_set_time(&ax2, SPLASH_BOUNCE_BACK_MS);
  lv_anim_set_path_cb(&ax2, lv_anim_path_ease_in_out);
  lv_anim_start(&ax2);
}

static void splash_side_start_enter_anim()
{
  if (!splash_side_img) return;
  if (g_splash_enter_started) return;

  int w=0, h=0, x_end=0, y_end=0;
  splash_side_calc_target(w, h, x_end, y_end);

  g_splash_side_target_x = x_end;
  g_splash_side_target_y = y_end;
  g_splash_side_w        = w;

  const int x_start = LCD_HOR_RES + SPLASH_ENTER_OFFSCREEN_PX;
  const int x_overshoot = x_end - SPLASH_BOUNCE_PX;

  lv_obj_set_pos(splash_side_img, x_start, y_end);
  lv_obj_set_style_opa(splash_side_img, LV_OPA_0, 0);

  lv_anim_t ax;
  lv_anim_init(&ax);
  lv_anim_set_var(&ax, splash_side_img);
  lv_anim_set_exec_cb(&ax, (lv_anim_exec_xcb_t)lv_obj_set_x);
  lv_anim_set_values(&ax, x_start, x_overshoot);
  lv_anim_set_time(&ax, SPLASH_ENTER_TIME_MS);
  lv_anim_set_path_cb(&ax, lv_anim_path_ease_in_out);
  lv_anim_set_ready_cb(&ax, splash_side_bounce_back_start);
  lv_anim_start(&ax);

  lv_anim_t ao;
  lv_anim_init(&ao);
  lv_anim_set_var(&ao, splash_side_img);
  lv_anim_set_exec_cb(&ao, (lv_anim_exec_xcb_t)anim_set_opa);
  lv_anim_set_values(&ao, LV_OPA_0, LV_OPA_COVER);
  lv_anim_set_time(&ao, SPLASH_FADE_IN_MS);
  lv_anim_set_path_cb(&ao, lv_anim_path_ease_in_out);
  lv_anim_start(&ao);

  splash_start_bg_parallax_enter();

  g_splash_enter_started = true;
}

static void clock_fade_in_time_date()
{
  if (!clock_time || !clock_date) return;
  if (g_clock_faded_in) return;

  lv_obj_set_style_opa(clock_time, LV_OPA_0, 0);
  lv_obj_set_style_opa(clock_date, LV_OPA_0, 0);

  lv_obj_fade_in(clock_time, 800, 0);
  lv_obj_fade_in(clock_date, 800, 200);

  g_clock_faded_in = true;
}

static void clock_post_switch_cb(lv_timer_t* t)
{
  (void)t;

  if (timeClient.isTimeSet()) update_time_from_ntp();
  clock_fade_in_time_date();

  lv_timer_del(t);
}

static void splash_exit_anim_ready_cb(lv_anim_t* a)
{
  (void)a;

  switch_screen(SCR_CLOCK);
  g_clock_shown_after_ntp = true;

  lv_timer_t* once = lv_timer_create(clock_post_switch_cb, CLOCK_FADE_DELAY_MS, nullptr);
  (void)once;
}

static void splash_side_start_exit_anim()
{
  if (!splash_side_img) return;
  if (g_splash_exit_started) return;

  const int w = g_splash_side_w > 0 ? g_splash_side_w
                                    : (int)((tiltdash_side_105_alpha.header.w * SPLASH_SIDE_ZOOM) / 256);

  const int x_start = lv_obj_get_x(splash_side_img);
  const int x_end   = -w - SPLASH_EXIT_OFFSCREEN_PX;

  lv_anim_t ax;
  lv_anim_init(&ax);
  lv_anim_set_var(&ax, splash_side_img);
  lv_anim_set_exec_cb(&ax, (lv_anim_exec_xcb_t)lv_obj_set_x);
  lv_anim_set_values(&ax, x_start, x_end);
  lv_anim_set_time(&ax, SPLASH_EXIT_TIME_MS);
  lv_anim_set_path_cb(&ax, lv_anim_path_ease_in_out);
  lv_anim_set_ready_cb(&ax, splash_exit_anim_ready_cb);
  lv_anim_start(&ax);

  lv_anim_t ao;
  lv_anim_init(&ao);
  lv_anim_set_var(&ao, splash_side_img);
  lv_anim_set_exec_cb(&ao, (lv_anim_exec_xcb_t)anim_set_opa);
  lv_anim_set_values(&ao, LV_OPA_COVER, LV_OPA_0);
  lv_anim_set_time(&ao, SPLASH_FADE_OUT_MS);
  lv_anim_set_path_cb(&ao, lv_anim_path_ease_in_out);
  lv_anim_start(&ao);

  splash_start_bg_parallax_exit();

  g_splash_exit_started = true;
}

// ====== SETUP: QR code to join the configuration hotspot ======
static constexpr uint8_t QR_VERSION     = 3;  // 29x29 modules - up to 53B of data at ECC_LOW
static constexpr uint8_t QR_MODULES     = 21 + 4 * (QR_VERSION - 1);
static constexpr int     QR_QUIET       = 3;  // quiet zone around the code, in modules
static constexpr int     QR_PX          = 5;  // px per module
static constexpr int     QR_CANVAS_SIDE = (QR_MODULES + 2 * QR_QUIET) * QR_PX;

static lv_obj_t* setup_qr_canvas = nullptr;

// LV_IMG_CF_INDEXED_1BIT stores one bit/pixel (+ a tiny 2-color palette)
// instead of 2 bytes/pixel like TRUE_COLOR - a few KB instead of ~30KB.
// This board has no PSRAM (confirmed by ESP.getPsramSize()==0 on this
// hardware), so the QR canvas has to live in the same tight internal RAM
// the WiFi driver needs for the AP/scan; at TRUE_COLOR size it was
// starving that init, which is what caused both the AP-start crash and
// scanNetworks() failing (-2). Indexed storage sidesteps the problem
// instead of just moving the allocation around.
static inline lv_color_t qr_idx(uint8_t idx) { lv_color_t c; c.full = idx; return c; }

static void setup_qr_alloc_canvas()
{
  if (setup_qr_canvas) return;

  size_t bufSize = LV_IMG_BUF_SIZE_INDEXED_1BIT(QR_CANVAS_SIDE, QR_CANVAS_SIDE);
  uint8_t* buf = (uint8_t*)heap_caps_malloc(bufSize, MALLOC_CAP_8BIT);

  Serial.printf("[SETUP] heap free=%u, QR buffer=%uB (%s)\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)bufSize, buf ? "OK" : "FAILED");

  if (!buf) {
    Serial.println("[SETUP] QR canvas alloc failed - no QR will be shown");
    return;
  }

  setup_qr_canvas = lv_canvas_create(scr_setup);
  lv_canvas_set_buffer(setup_qr_canvas, buf, QR_CANVAS_SIDE, QR_CANVAS_SIDE, LV_IMG_CF_INDEXED_1BIT);
  lv_canvas_set_palette(setup_qr_canvas, 0, lv_color_white());
  lv_canvas_set_palette(setup_qr_canvas, 1, lv_color_black());
  lv_obj_align(setup_qr_canvas, LV_ALIGN_LEFT_MID, 24, 0);

  Serial.printf("[SETUP] QR canvas ready: %dx%d px (module=%dpx)\n",
                QR_CANVAS_SIDE, QR_CANVAS_SIDE, QR_PX);
}

static void setup_qr_render(const char* ssid)
{
  if (!setup_qr_canvas) return;

  char payload[96];
  // Standard QR format for joining a WiFi network (our hotspot is open,
  // no password - hence T:nopass).
  snprintf(payload, sizeof(payload), "WIFI:T:nopass;S:%s;;", ssid);

  QRCode qr;
  static uint8_t qrData[128]; // enough for QR_VERSION=3 (qrcode_getBufferSize(3)=106B)
  if (qrcode_initText(&qr, qrData, QR_VERSION, ECC_LOW, payload) != 0) {
    Serial.println("[SETUP] QR encode failed");
    return;
  }

  lv_canvas_fill_bg(setup_qr_canvas, qr_idx(0), LV_OPA_COVER); // index 0 = white

  for (uint8_t y = 0; y < qr.size; y++) {
    for (uint8_t x = 0; x < qr.size; x++) {
      if (!qrcode_getModule(&qr, x, y)) continue;
      const int px0 = (QR_QUIET + x) * QR_PX;
      const int py0 = (QR_QUIET + y) * QR_PX;
      for (int dy = 0; dy < QR_PX; dy++)
        for (int dx = 0; dx < QR_PX; dx++)
          lv_canvas_set_px_color(setup_qr_canvas, px0 + dx, py0 + dy, qr_idx(1)); // index 1 = black
    }
  }
}

// Entering the SETUP screen: starts the hotspot + portal (if needed),
// refreshes the labels and renders the QR code. Called from switch_screen().
static void setup_screen_on_enter()
{
  // Start the AP first, while the heap is at its roomiest (nothing else
  // in the app has claimed memory for the setup screen yet). The QR
  // canvas buffer (~22KB in internal RAM - PSRAM isn't available on this
  // board) used to be allocated once at boot in ui_init(), permanently
  // taking that memory away from the WiFi driver's own init; that starved
  // allocation is what caused the LoadProhibited crash inside
  // wifi_softap_start. Allocating the canvas only now, after the AP is
  // already up, means it can never compete with that init again.
  wifi_portal_start_ap();

  const char* ssid = wifi_portal_ap_ssid();
  if (setup_ssid_lbl) lv_label_set_text(setup_ssid_lbl, ssid);
  if (setup_ip_lbl)   lv_label_set_text(setup_ip_lbl, wifi_portal_ap_ip().toString().c_str());

  setup_qr_alloc_canvas(); // no-op if already allocated from a previous visit
  setup_qr_render(ssid);
}

// Leaving the SETUP screen: stops the hotspot and - if we have a saved
// network - resumes normal connecting as a client (STA).
static void setup_screen_on_leave()
{
  wifi_portal_stop_ap();

  if (wifi_portal_has_credentials()) {
    wifi_begin_nonblocking();
  }
}

// ====== UI init ======
static void ui_init()
{
  scr_splash = lv_obj_create(NULL);
  scr_clock  = lv_obj_create(NULL);
  scr_main   = lv_obj_create(NULL);
  scr_cal    = lv_obj_create(NULL);
  scr_setup  = lv_obj_create(NULL);

  lv_obj_t* screens[] = {scr_splash, scr_clock, scr_main, scr_cal, scr_setup};
  for (auto* scr : screens) {
    lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_add_event_cb(scr, gesture_event_cb, LV_EVENT_GESTURE, nullptr);
  }

  // ===== SPLASH =====
  splash_bg = lv_img_create(scr_splash);
  lv_img_set_src(splash_bg, &tiltdash_bg);
  lv_obj_center(splash_bg);
  g_splash_bg_base_x = lv_obj_get_x(splash_bg);
  lv_obj_set_style_opa(splash_bg, LV_OPA_0, 0);

  splash_wifi    = make_wifi_icon(scr_splash);
  splash_offline = make_offline_label(scr_splash);

  // start with OFFLINE visible until a successful connection
  ui_offline_labels_set_opa(LV_OPA_COVER);

  splash_side_img = lv_img_create(scr_splash);
  lv_img_set_src(splash_side_img, &tiltdash_side_105_alpha);
  lv_img_set_zoom(splash_side_img, SPLASH_SIDE_ZOOM);

  int w=0, h=0, x_end=0, y_end=0;
  splash_side_calc_target(w, h, x_end, y_end);

  g_splash_side_target_x = x_end;
  g_splash_side_target_y = y_end;
  g_splash_side_w        = w;

  lv_obj_set_size(splash_side_img, w, h);
  lv_obj_set_pos(splash_side_img, LCD_HOR_RES + SPLASH_ENTER_OFFSCREEN_PX, y_end);
  lv_obj_set_style_opa(splash_side_img, LV_OPA_0, 0);

  // ===== CLOCK =====
  clock_bg = lv_img_create(scr_clock);
  lv_img_set_src(clock_bg, &tiltdash_bg);
  lv_obj_center(clock_bg);

  clock_wifi    = make_wifi_icon(scr_clock);
  clock_offline = make_offline_label(scr_clock);

  clock_time = lv_label_create(scr_clock);
  lv_obj_set_style_text_color(clock_time, lv_color_white(), 0);
  lv_obj_set_style_text_font(clock_time, &lv_font_montserrat_48, 0);
  lv_label_set_text(clock_time, "--:--");
  lv_obj_align(clock_time, LV_ALIGN_TOP_RIGHT, -20, 32);
  lv_obj_set_style_opa(clock_time, LV_OPA_0, 0);

  clock_date = lv_label_create(scr_clock);
  lv_obj_set_style_text_color(clock_date, lv_color_white(), 0);
  lv_obj_set_style_text_font(clock_date, &lv_font_montserrat_22, 0);
  lv_label_set_text(clock_date, "---- -- --");
  lv_obj_align(clock_date, LV_ALIGN_TOP_RIGHT, -20, 82);
  lv_obj_set_style_opa(clock_date, LV_OPA_0, 0);

  // ===== GYRO (MAIN) =====
  gyro_bg = lv_img_create(scr_main);
  lv_img_set_src(gyro_bg, &tiltdash_bg);
  lv_obj_center(gyro_bg);
  lv_obj_move_background(gyro_bg);

  gyro_wifi    = make_wifi_icon(scr_main);
  gyro_offline = make_offline_label(scr_main);

  const int splitX = (int)lroundf(LCD_HOR_RES * SPLIT_RATIO);

  zoneLeft  = make_zone(scr_main, 0, 0, splitX, LCD_VER_RES);
  zoneRight = make_zone(scr_main, splitX, 0, LCD_HOR_RES - splitX, LCD_VER_RES);

  imgPitch = lv_img_create(zoneLeft);
  lv_img_set_src(imgPitch, &tiltdash_side_105_alpha);
  lv_img_set_zoom(imgPitch, IMG_ZOOM_SIDE);
  lv_img_set_pivot(imgPitch, tiltdash_side_105_alpha.header.w / 2, tiltdash_side_105_alpha.header.h / 2);
  lv_obj_align(imgPitch, LV_ALIGN_CENTER, 0, IMG_Y_OFFSET);

  lblPitch = lv_label_create(zoneLeft);
  lv_obj_set_style_text_color(lblPitch, ral7037(), 0);
  lv_label_set_text(lblPitch, "0");
  lv_obj_align(lblPitch, LV_ALIGN_TOP_MID, LABEL_LEFT_X_OFFSET, LABEL_Y_PAD);

  imgRoll = lv_img_create(zoneRight);
  lv_img_set_src(imgRoll, &tiltdash_back_105_alpha);
  lv_img_set_zoom(imgRoll, IMG_ZOOM_BACK);
  lv_img_set_pivot(imgRoll, tiltdash_back_105_alpha.header.w / 2, tiltdash_back_105_alpha.header.h / 2);
  lv_obj_align(imgRoll, LV_ALIGN_CENTER, IMG_RIGHT_X_OFFSET, IMG_Y_OFFSET);

  lblRoll = lv_label_create(zoneRight);
  lv_obj_set_style_text_color(lblRoll, ral7037(), 0);
  lv_label_set_text(lblRoll, "0");
  lv_obj_align(lblRoll, LV_ALIGN_TOP_MID, LABEL_RIGHT_X_OFFSET, LABEL_Y_PAD);

  const int indY2 = LCD_VER_RES - IND_PAD_B - IND_H;
  const int indY1 = indY2 - IND_GAP_Y - IND_H;

  make_indicator(scr_main, indY1, &accBarCont, &accBarFill);
  make_indicator(scr_main, indY2, &latBarCont, &latBarFill);

  // ===== CAL =====
  cal_bg = lv_img_create(scr_cal);
  lv_img_set_src(cal_bg, &tiltdash_bg);
  lv_obj_center(cal_bg);
  lv_obj_move_background(cal_bg);

  cal_wifi    = make_wifi_icon(scr_cal);
  cal_offline = make_offline_label(scr_cal);

  lv_obj_t* title2 = lv_label_create(scr_cal);
  lv_obj_set_style_text_color(title2, lv_color_white(), 0);
  lv_label_set_text(title2, "CALIBRATION (ZERO)");
  lv_obj_align(title2, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t* btnCal = lv_btn_create(scr_cal);
  lv_obj_set_size(btnCal, 240, 70);
  lv_obj_align(btnCal, LV_ALIGN_CENTER, 0, -10);
  lv_obj_add_event_cb(btnCal, cal_btn_event_cb, LV_EVENT_ALL, nullptr);

  lv_obj_t* tCal = lv_label_create(btnCal);
  lv_label_set_text(tCal, "SET POSITION\nAND TAP");
  lv_obj_center(tCal);

  calInfo = lv_label_create(scr_cal);
  lv_obj_set_style_text_color(calInfo, ral7037(), 0);
  lv_label_set_text(calInfo, "Tap to save zero.");
  lv_obj_align(calInfo, LV_ALIGN_CENTER, 0, 55);

  lv_obj_t* hint2 = lv_label_create(scr_cal);
  lv_obj_set_style_text_color(hint2, ral7037(), 0);
  lv_label_set_text(hint2, "Swipe: back to tilts");
  lv_obj_align(hint2, LV_ALIGN_BOTTOM_RIGHT, -10, -6);

  // ===== SETUP (portal WiFi) =====
  lv_obj_t* setupTitle = lv_label_create(scr_setup);
  lv_obj_set_style_text_color(setupTitle, lv_color_white(), 0);
  lv_label_set_text(setupTitle, "WIFI SETUP");
  lv_obj_align(setupTitle, LV_ALIGN_TOP_MID, 0, 10);

  // QR canvas is allocated lazily from setup_screen_on_enter() instead of
  // here - see the comment there for why (it used to permanently pin
  // ~22KB of internal RAM from boot onward, which starved the WiFi
  // driver's own init and crashed it when the AP was started).

  lv_obj_t* setupHint1 = lv_label_create(scr_setup);
  lv_obj_set_style_text_color(setupHint1, ral7037(), 0);
  lv_label_set_text(setupHint1, "Scan to join\nthe hotspot:");
  lv_obj_align(setupHint1, LV_ALIGN_RIGHT_MID, -20, -55);

  setup_ssid_lbl = lv_label_create(scr_setup);
  lv_obj_set_style_text_color(setup_ssid_lbl, lv_color_white(), 0);
  lv_label_set_text(setup_ssid_lbl, "---");
  lv_obj_align(setup_ssid_lbl, LV_ALIGN_RIGHT_MID, -20, -18);

  lv_obj_t* setupHint2 = lv_label_create(scr_setup);
  lv_obj_set_style_text_color(setupHint2, ral7037(), 0);
  lv_label_set_text(setupHint2, "or in a browser:");
  lv_obj_align(setupHint2, LV_ALIGN_RIGHT_MID, -20, 14);

  setup_ip_lbl = lv_label_create(scr_setup);
  lv_obj_set_style_text_color(setup_ip_lbl, lv_color_white(), 0);
  lv_label_set_text(setup_ip_lbl, "---");
  lv_obj_align(setup_ip_lbl, LV_ALIGN_RIGHT_MID, -20, 40);

  lv_obj_t* setupHint3 = lv_label_create(scr_setup);
  lv_obj_set_style_text_color(setupHint3, ral7037(), 0);
  lv_label_set_text(setupHint3, "Swipe: back to clock");
  lv_obj_align(setupHint3, LV_ALIGN_BOTTOM_RIGHT, -10, -6);

  g_wifi_blink_timer = lv_timer_create(wifi_blink_cb, WIFI_ICON_BLINK_MS, nullptr);
}

void setup()
{
  Serial.begin(115200);
  delay(150);

  // remember boot time (for the splash timeout)
  g_boot_ms = millis();

  loadColorMode();

  rm67162_init();
  lcd_setRotation(ROTATION);
  lcd_brightness(0xD0);

  lv_init();
  lvgl_init_tick();

  static lv_color_t buf1[LCD_HOR_RES * DRAWBUF_LINES];
  static lv_color_t buf2[LCD_HOR_RES * DRAWBUF_LINES];
  static lv_disp_draw_buf_t draw_buf;
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LCD_HOR_RES * DRAWBUF_LINES);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res  = LCD_HOR_RES;
  disp_drv.ver_res  = LCD_VER_RES;
  disp_drv.flush_cb = my_flush_cb;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(10);

  tp.begin();
  Wire.setClock(400000);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touch_read_cb;
  lv_indev_drv_register(&indev_drv);

  ui_init();

  wifi_portal_init();
  Serial.printf("[BOOT] Applying saved tz offset: %d min\n", wifi_portal_get_tz_offset_min());
  timeClient.setTimeOffset(wifi_portal_get_tz_offset_min() * 60);

  switch_screen(SCR_SPLASH);
  lv_obj_fade_in(splash_bg, 800, 120);
  splash_side_start_enter_anim();

  if (!wifi_portal_has_credentials()) {
    // No saved network - go straight into setup mode instead of waiting
    // on WiFi/NTP (there's nothing to connect to anyway). switch_screen()
    // starts the AP and fills in the labels itself (setup_screen_on_enter).
    switch_screen(SCR_SETUP);
  } else {
    wifi_ntp_init();
  }

  imu_init();   // IMU module

  Serial.println("Ready.");
}

void loop()
{
  static uint32_t lastLv  = 0;
  static uint32_t lastUi  = 0;
  static uint32_t lastImu = 0;

  const uint32_t now = millis();

  // LVGL
  if ((uint32_t)(now - lastLv) >= 10) {
    lastLv = now;
    lv_timer_handler();
  }

  if (wifi_portal_is_active()) {
    // WiFi configuration mode (AP + captive portal) - the normal
    // WiFi/NTP/splash state machine is irrelevant here (and would
    // conflict with AP mode), so we skip it entirely.
    wifi_portal_loop();
  } else {
    // WiFi/NTP
    wifi_ntp_update_state();

    // Decide when to leave splash screen:
    // 1) normal path: WiFi + NTP ready
    // 2) offline path: timeout after SPLASH_MAX_WAIT_MS since boot
    if (current_screen == SCR_SPLASH && !g_splash_exit_started) {

      // case 1: online, time is set
      if (g_wifi_ok && g_ntp_ok && timeClient.isTimeSet()) {
        splash_side_start_exit_anim();
      }
      // case 2: offline fallback - no network / no time, but we don't want to block UI
      else if (!g_splash_forced_exit && (now - g_boot_ms) >= SPLASH_MAX_WAIT_MS) {
        Serial.println("[SPLASH] No network / NTP, continuing in offline mode.");
        g_splash_forced_exit = true;
        splash_side_start_exit_anim();
      }
    }
  }

  // IMU update ONLY when needed (MAIN or CAL)
  if (imu_should_run() && (uint32_t)(now - lastImu) >= IMU_PERIOD_MS) {
    lastImu = now;
    imu_update();
  }

  // UI updates
  if ((uint32_t)(now - lastUi) >= UI_PERIOD_MS) {
    lastUi = now;

    // Keep clock updated only when visible and time is set
    if (current_screen == SCR_CLOCK && timeClient.isTimeSet() && g_clock_shown_after_ntp) {
      update_time_from_ntp();
    }

    // Main screen - update from last IMU sample
    if (current_screen == SCR_MAIN) {
      ImuSample s = imu_get_sample();
      if (s.valid) {
        float pitchDeg = s.pitchDeg;
        float rollDeg  = s.rollDeg;

#if INVERT_PITCH
        pitchDeg = -pitchDeg;
#endif
#if INVERT_ROLL
        rollDeg = -rollDeg;
#endif

        ui_update(pitchDeg, rollDeg, s.long_g, s.lat_g);
      }
    }
  }

  delay(1);
}
