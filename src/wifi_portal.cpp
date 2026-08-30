#include "wifi_portal.h"
#include "imu.h"
#include "tiltdash_images_105_alpha.h" // whatever vehicle graphics are currently compiled in - see img_assets/

#include <ctype.h>
#include <math.h>
#include <string.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>

// ====== NVS ======
static Preferences prefsWifi;
static constexpr const char* NVS_NAMESPACE = "wifinet";

static String g_ssid;
static String g_pass;
static int    g_tzOffsetMin = 60; // default UTC+1, overridden by NVS if set
static String g_deviceName;       // owner-chosen name, empty = use the MAC-suffixed default
// Vehicle dimensions (cm) for converting pitch/roll to how much a corner
// needs raising: dh = dimension * sin(angle). Defaults are a rough van/
// small-camper ballpark - not accurate for any specific vehicle until the
// owner sets their own via the setup page's "Vehicle dimensions" card.
static float g_wheelbaseCm = 350.0f; // front-back axle spacing, used with pitch
static float g_trackCm     = 180.0f; // left-right wheel spacing, used with roll

// Curated list of real-world UTC offsets (minutes) for the setup page's
// timezone <select> - deliberately not every 15-minute step from -12:00
// to +14:00, since most of those don't correspond to an actual timezone.
struct TzOption { int16_t minutes; const char* label; };
static const TzOption TZ_OPTIONS[] = {
    { -720, "UTC-12:00" },
    { -660, "UTC-11:00" },
    { -600, "UTC-10:00 (Hawaii)" },
    { -540, "UTC-9:00 (Alaska)" },
    { -480, "UTC-8:00 (US Pacific)" },
    { -420, "UTC-7:00 (US Mountain)" },
    { -360, "UTC-6:00 (US Central)" },
    { -300, "UTC-5:00 (US Eastern)" },
    { -240, "UTC-4:00 (Atlantic)" },
    { -210, "UTC-3:30 (Newfoundland)" },
    { -180, "UTC-3:00 (Brazil/Argentina)" },
    { -120, "UTC-2:00" },
    {  -60, "UTC-1:00" },
    {    0, "UTC+0:00 (London winter)" },
    {   60, "UTC+1:00 (CET - e.g. Poland winter)" },
    {  120, "UTC+2:00 (CEST - e.g. Poland summer, EET)" },
    {  180, "UTC+3:00 (Moscow)" },
    {  210, "UTC+3:30 (Iran)" },
    {  240, "UTC+4:00" },
    {  270, "UTC+4:30 (Afghanistan)" },
    {  300, "UTC+5:00" },
    {  330, "UTC+5:30 (India)" },
    {  345, "UTC+5:45 (Nepal)" },
    {  360, "UTC+6:00" },
    {  390, "UTC+6:30 (Myanmar)" },
    {  420, "UTC+7:00" },
    {  480, "UTC+8:00 (China)" },
    {  540, "UTC+9:00 (Japan/Korea)" },
    {  570, "UTC+9:30 (Australia Central)" },
    {  600, "UTC+10:00 (Australia East)" },
    {  630, "UTC+10:30 (Lord Howe)" },
    {  660, "UTC+11:00" },
    {  720, "UTC+12:00 (New Zealand)" },
    {  765, "UTC+12:45 (Chatham)" },
    {  780, "UTC+13:00" },
    {  840, "UTC+14:00 (Kiribati)" },
};
static constexpr size_t TZ_OPTIONS_COUNT = sizeof(TZ_OPTIONS) / sizeof(TZ_OPTIONS[0]);

// ====== AP / portal ======
static WebServer   server(80);
static DNSServer   dnsServer;
static bool        g_apActive = false;
static String      g_apSsid;
static IPAddress   g_apIp;
static const uint32_t DNS_PORT = 53;
static bool        g_httpHandlersRegistered = false;

// ====== Remote monitor ======
static bool   g_monitorMode      = false; // true: handleRoot() serves the live-data page, not the setup form
static bool   g_monitorServing   = false; // server.begin() called directly on an existing STA connection (no AP)
static bool   g_monitorUsingOwnAp = false;
static String g_mdnsHostname;
static String g_monitorTimeStr = "--:--";
static String g_monitorDateStr = "----------";

// ====== Helpers ======

static String defaultDeviceTag()
{
    // MAC-suffixed fallback identity, used for both the AP SSID and mDNS
    // hostname whenever the owner hasn't picked a name of their own -
    // guarantees multiple units nearby don't collide by default.
    uint64_t mac = ESP.getEfuseMac();
    char buf[16];
    snprintf(buf, sizeof(buf), "%04X", (unsigned)(mac & 0xFFFF));
    return String(buf);
}

// DNS labels (mDNS hostnames) only allow letters/digits/hyphens, no
// leading/trailing/duplicate hyphens - anything else in the owner's
// chosen name gets folded into a single hyphen so "My Camper!" becomes
// "my-camper" instead of silently breaking <name>.local resolution.
static String sanitizeForHostname(const String& raw)
{
    String out;
    out.reserve(raw.length());
    bool lastWasHyphen = true; // true so we never start with a hyphen
    for (size_t i = 0; i < raw.length(); i++) {
        char c = raw[i];
        if (isalnum((unsigned char)c)) {
            out += (char)tolower((unsigned char)c);
            lastWasHyphen = false;
        } else if (!lastWasHyphen) {
            out += '-';
            lastWasHyphen = true;
        }
    }
    while (out.length() && out[out.length() - 1] == '-') out.remove(out.length() - 1);
    return out;
}

static String buildApSsid()
{
    // WiFi SSIDs tolerate spaces/punctuation fine, so the owner's chosen
    // name is used as-is (just length-capped - SSIDs are limited to 32
    // bytes).
    if (g_deviceName.length() > 0) {
        String ssid = g_deviceName;
        if (ssid.length() > 32) ssid = ssid.substring(0, 32);
        return ssid;
    }
    return "TiltDash-" + defaultDeviceTag();
}

static String buildMdnsHostname()
{
    if (g_deviceName.length() > 0) {
        String host = sanitizeForHostname(g_deviceName);
        if (host.length() > 0) return host; // e.g. all-punctuation input falls through to the default below
    }
    String tag = defaultDeviceTag();
    tag.toLowerCase(); // toLowerCase() mutates in place (returns void), can't be chained
    return "tiltdash-" + tag;
}

// Used wherever the owner-chosen device name (arbitrary user input) gets
// embedded into HTML - the setup/monitor page titles and the name
// input's value attribute.
static String htmlEscape(const String& s)
{
    String out;
    out.reserve(s.length());
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        switch (c) {
            case '&':  out += "&amp;";  break;
            case '<':  out += "&lt;";   break;
            case '>':  out += "&gt;";   break;
            case '"':  out += "&quot;"; break;
            case '\'': out += "&#39;";  break;
            default:   out += c;        break;
        }
    }
    return out;
}

static String jsonEscape(const String& s)
{
    String out;
    out.reserve(s.length());
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if ((uint8_t)c >= 0x20) out += c; // drop control chars
    }
    return out;
}

// Handles GET /scan: runs a fresh scan and returns it as JSON. Called
// on demand from the page's own JS (see handleRoot()) instead of once
// up front, so it can be retried without restarting the device.
//
// The radio sits in AP-only mode at rest (see wifi_portal_start_ap()) to
// keep steady-state memory usage as low as possible on this board (no
// PSRAM) - a full-time WIFI_AP_STA was enough to starve lwIP and make the
// portal page itself unreachable. STA is added back just for the
// duration of this one scan and dropped again right after; the AP bit is
// never touched, so its beacon/netif isn't disturbed.
static void handleScan()
{
    Serial.printf("[WIFI-PORTAL] /scan requested (heap free=%u, mode=%d, status=%d)\n",
                  (unsigned)ESP.getFreeHeap(), (int)WiFi.getMode(), (int)WiFi.status());

    WiFi.mode(WIFI_AP_STA);
    delay(300); // give the STA side (freshly added on top of the AP) time to settle
    WiFi.scanDelete();

    // Explicit params (active scan, 500ms/channel, all channels, include
    // hidden) rather than relying on scanNetworks()'s defaults, so this
    // matches exactly what's being tested when comparing results.
    int n = WiFi.scanNetworks(false /* async */, true /* show_hidden */,
                               false /* passive */, 500 /* max_ms_per_chan */,
                               0 /* all channels */);
    Serial.printf("[WIFI-PORTAL] scanNetworks() returned %d, mode=%d, status=%d\n",
                  n, (int)WiFi.getMode(), (int)WiFi.status());

    // A hard error (<0) or a suspiciously empty result on the first try
    // both get one retry with more settling time - the earlier scan
    // failures elsewhere in this flow all traced back to not giving the
    // radio enough time after a mode change, not to an actual empty
    // environment.
    for (int attempt = 0; n <= 0 && attempt < 2; attempt++) {
        delay(500);
        n = WiFi.scanNetworks(false, true, false, 500, 0);
        Serial.printf("[WIFI-PORTAL] scanNetworks() retry %d returned %d\n", attempt + 1, n);
    }
    if (n < 0) n = 0; // report "no networks" rather than emit malformed JSON

    for (int i = 0; i < n; i++) {
        Serial.printf("[WIFI-PORTAL]   [%d] SSID='%s' RSSI=%d chan=%d enc=%d hidden=%d\n",
                      i, WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i),
                      (int)WiFi.encryptionType(i), WiFi.SSID(i).length() == 0);
    }

    String json;
    json.reserve(64 + (size_t)n * 40);
    json += "[";
    bool first = true;
    for (int i = 0; i < n; i++) {
        String ssid = WiFi.SSID(i);
        if (ssid.length() == 0) continue; // skip hidden SSIDs in the list
        if (!first) json += ",";
        first = false;
        json += "{\"ssid\":\"" + jsonEscape(ssid) + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
    }
    json += "]";
    Serial.printf("[WIFI-PORTAL] /scan responding with %d entries\n", n);

    WiFi.scanDelete();
    WiFi.mode(WIFI_AP); // back to the low-memory steady state

    server.send(200, "application/json", json);
}

// Must match PITCH_SIGN/ROLL_SIGN in main.cpp - only used here to rotate
// the monitor page's icons the same visual direction as the on-device
// side/back images (lv_img_set_angle). Small enough to duplicate rather
// than pull main.cpp's UI constants into this module.
static constexpr int MONITOR_PITCH_SIGN = -1;
static constexpr int MONITOR_ROLL_SIGN  = +1;

// Handles GET /live: current pitch/roll as JSON, polled from the monitor
// page's own JS. Cheap - just reads the last IMU sample, no scanning or
// mode changes involved.
static void handleLive()
{
    ImuSample s = imu_get_display_sample();
    // pitch/roll: same sign/magnitude convention as the on-device MAIN/
    // MONITOR number readouts (see ui_update() in main.cpp) - pitch
    // negated (nose-down/downhill reads negative), roll a positive
    // magnitude only. pitchAngle/rollAngle: raw signed degrees, for
    // rotating the page's icons the same direction as the real images.
    // Rough "how much to raise this end/side" estimate: dh = dimension *
    // sin(angle). Magnitude only, same reasoning as the roll degrees -
    // the icon's rotation direction already conveys which way, so this
    // isn't duplicated as an invented "front/rear"/"left/right" label
    // that could get the direction backwards for someone's mounting.
    constexpr float DEG2RAD = 0.017453292519943295f; // matches imu.cpp's own constant
    float pitchCm = fabsf(g_wheelbaseCm * sinf(s.pitchDeg * DEG2RAD));
    float rollCm  = fabsf(g_trackCm     * sinf(s.rollDeg  * DEG2RAD));

    // rssi: 0 is a sentinel for "not applicable" (using our own hotspot,
    // so there's no STA link to measure) rather than a real reading.
    int rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
    float tempC = imu_get_temperature_c();
    if (isnan(tempC)) tempC = 0.0f; // NAN would print as "nan" and break JSON parsing client-side

    char json[280];
    snprintf(json, sizeof(json),
             "{\"pitch\":%.1f,\"roll\":%.1f,\"pitchAngle\":%.1f,\"rollAngle\":%.1f,"
             "\"pitchCm\":%.1f,\"rollCm\":%.1f,\"temp\":%.1f,\"rssi\":%d,\"valid\":%s,"
             "\"time\":\"%s\",\"date\":\"%s\"}",
             -s.pitchDeg, fabsf(s.rollDeg),
             MONITOR_PITCH_SIGN * s.pitchDeg, MONITOR_ROLL_SIGN * s.rollDeg,
             pitchCm, rollCm, tempC, rssi,
             s.valid ? "true" : "false",
             g_monitorTimeStr.c_str(), g_monitorDateStr.c_str());
    server.send(200, "application/json", json);
}

static inline void put16LE(uint8_t* p, uint16_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }
static inline void put32LE(uint8_t* p, uint32_t v) {
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}

// Renders a TRUE_COLOR_ALPHA lv_img_dsc_t (the same vehicle graphics used
// on-device - see tiltdash_images_105_alpha.h, whichever preset from
// img_assets/ is currently compiled in) as a 24-bit BMP for the monitor
// page's <img> tags. Plain BMP has no alpha channel, so transparent
// pixels are alpha-blended onto a fixed dark background matching the
// page's .card color instead. Built fresh per request and freed right
// after - these are small (at most 276x105 -> under 90KB) and this
// endpoint is only hit once per page load, not worth caching.
static void sendLvImageAsBmp(const lv_img_dsc_t& img)
{
    const int w = img.header.w;
    const int h = img.header.h;
    const int rowBytes = ((w * 3 + 3) / 4) * 4; // BMP rows pad to a 4-byte boundary
    const size_t pixelDataSize = (size_t)rowBytes * h;
    const size_t fileSize = 54 + pixelDataSize; // 14B file header + 40B DIB header

    // Streamed one row at a time rather than built in a single fileSize
    // buffer: the side image alone needs ~87KB contiguous, which can fail
    // under heap fragmentation even when total free heap looks plenty -
    // that's exactly what made it not load while the smaller back image
    // (~36KB) worked fine. A row buffer only ever needs to hold the
    // widest image's single row (well under 1KB).
    uint8_t header[54] = {0};
    header[0] = 'B'; header[1] = 'M';
    put32LE(header + 2,  (uint32_t)fileSize);
    put32LE(header + 10, 54);              // pixel data offset
    put32LE(header + 14, 40);              // DIB header size
    put32LE(header + 18, (uint32_t)w);
    put32LE(header + 22, (uint32_t)h);     // positive height = bottom-up row order
    put16LE(header + 26, 1);               // color planes
    put16LE(header + 28, 24);              // bits per pixel
    put32LE(header + 34, (uint32_t)pixelDataSize);

    // Background to blend transparent pixels against - matches the
    // monitor page's .card background (#222222).
    const uint8_t bgR = 0x22, bgG = 0x22, bgB = 0x22;

    server.setContentLength(fileSize);
    server.send(200, "image/bmp", "");
    server.sendContent((const char*)header, sizeof(header));

    uint8_t rowBuf[900]; // >= widest supported image (276px * 3B, padded)
    const uint8_t* src = img.data; // per pixel: RGB565 (2 bytes, little-endian) + alpha (1 byte)
    for (int y = h - 1; y >= 0; y--) { // BMP rows are bottom-up
        const uint8_t* srcRow = src + (size_t)y * w * 3;
        for (int x = 0; x < w; x++) {
            uint16_t rgb565 = srcRow[x * 3] | (srcRow[x * 3 + 1] << 8);
            uint8_t a  = srcRow[x * 3 + 2];
            uint8_t r5 = (rgb565 >> 11) & 0x1F;
            uint8_t g6 = (rgb565 >> 5)  & 0x3F;
            uint8_t b5 = rgb565 & 0x1F;
            uint8_t r8 = (r5 << 3) | (r5 >> 2);
            uint8_t g8 = (g6 << 2) | (g6 >> 4);
            uint8_t b8 = (b5 << 3) | (b5 >> 2);
            rowBuf[x * 3 + 0] = (uint8_t)((b8 * a + bgB * (255 - a)) / 255); // BMP is BGR
            rowBuf[x * 3 + 1] = (uint8_t)((g8 * a + bgG * (255 - a)) / 255);
            rowBuf[x * 3 + 2] = (uint8_t)((r8 * a + bgR * (255 - a)) / 255);
        }
        for (int pad = w * 3; pad < rowBytes; pad++) rowBuf[pad] = 0; // row padding bytes
        server.sendContent((const char*)rowBuf, rowBytes);
    }
}

static void handleSideImg() { sendLvImageAsBmp(tiltdash_side_105_alpha); }
static void handleBackImg() { sendLvImageAsBmp(tiltdash_back_105_alpha); }

// The remote-monitor page: shows live pitch/roll, auto-refreshed via
// /live, so a phone can watch it from outside the vehicle while
// levelling. Rendered by handleRoot() when g_monitorMode is set - see
// wifi_monitor_start().
static void handleMonitorRoot()
{
    String displayName = g_deviceName.length() > 0 ? htmlEscape(g_deviceName) : "TiltDash";

    String page;
    page.reserve(3600);

    page += "<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width, initial-scale=1'>"
            "<title>" + displayName + " - Monitor</title>"
            "<style>"
            "*{box-sizing:border-box}"
            "body{font-family:-apple-system,'Segoe UI',Roboto,sans-serif;background:#111;color:#eee;"
            "padding:24px 16px;max-width:420px;margin:auto;text-align:center}"
            "header{margin-bottom:28px}"
            "h1{font-size:19px;font-weight:600;margin:0;letter-spacing:.3px}"
            ".sub{display:flex;align-items:center;justify-content:center;gap:6px;"
            "margin-top:6px;font-size:12px;color:#7b7d7d;text-transform:uppercase;letter-spacing:1px}"
            ".dot{width:7px;height:7px;border-radius:50%;background:#5ad16b;"
            "animation:pulse 1.6s ease-in-out infinite}"
            "@keyframes pulse{0%,100%{opacity:1}50%{opacity:.35}}"
            ".row{display:flex;gap:14px}"
            ".card{flex:1;background:#1c1c1c;border:1px solid #2a2a2a;border-radius:14px;"
            "padding:18px 8px 16px;box-shadow:0 4px 14px rgba(0,0,0,.35)}"
            ".icon-wrap{height:64px;display:flex;align-items:center;justify-content:center}"
            ".icon{transition:transform 0.2s ease-out}"
            ".label{font-size:12px;color:#7b7d7d;letter-spacing:1.5px;margin-top:14px}"
            ".val{font-size:44px;font-weight:700;margin-top:2px;font-variant-numeric:tabular-nums}"
            ".cm{font-size:12px;color:#7b7d7d;margin-top:4px}"
            ".meta{margin-top:18px;font-size:12px;color:#666}"
            ".deg{font-size:20px;color:#7b7d7d;font-weight:400;vertical-align:14px;line-height:1}"
            ".badge{display:inline-block;padding:8px 22px;border-radius:20px;font-weight:700;"
            "font-size:13px;letter-spacing:1.5px;margin-top:2px;"
            "background:#2a2a2a;color:#888;transition:background-color .3s,color .3s}"
            ".badge.level{background:#1f4d2e;color:#5ad16b}"
            ".clock{margin:16px 0 22px}"
            ".clock #time{font-size:30px;font-weight:600;font-variant-numeric:tabular-nums}"
            ".clock #date{display:block;font-size:12px;color:#7b7d7d;margin-top:2px}"
            "</style></head><body>"
            "<header><h1>" + displayName + "</h1>"
            "<div class='sub'><span class='dot'></span>Live monitor</div></header>"
            "<div class='badge' id='levelBadge'>--</div>"
            "<div class='clock'><span id='time'>--:--</span><span id='date'>----------</span></div>"
            "<div class='row'>"
            "<div class='card'>"
            // Same vehicle graphics as the on-device display, served as
            // BMP (see /side.bmp - sendLvImageAsBmp() in this file) - one
            // static image, rotated live via CSS transform exactly like
            // lv_img_set_angle() does on-screen. Both icons sit in a
            // fixed-height wrapper so the differing image aspect ratios
            // (side is wide, back is tall) don't push the labels below
            // them out of alignment between the two cards.
            "<div class='icon-wrap'><img class='icon' id='pitchIcon' src='/side.bmp' width='130'></div>"
            "<div class='label'>PITCH</div><div class='val' id='p'>--<span class='deg'>&deg;</span></div>"
            "<div class='cm' id='pCm'>&asymp; -- cm</div></div>"
            "<div class='card'>"
            "<div class='icon-wrap'><img class='icon' id='rollIcon' src='/back.bmp' width='62'></div>"
            "<div class='label'>ROLL</div><div class='val' id='r'>--<span class='deg'>&deg;</span></div>"
            "<div class='cm' id='rCm'>&asymp; -- cm</div></div>"
            "</div>"
            "<div class='meta'>Sensor <span id='temp'>--</span>&deg;C &nbsp;&middot;&nbsp; "
            "WiFi <span id='rssi'>--</span></div>"
            "<script>"
            // toFixed(0) on a small negative float (e.g. -0.3) returns the
            // string "-0" even though it's numerically zero - round trips
            // through Math.round() first so -0 becomes a real 0.
            "function fmt0(n){var v=Math.round(n);return v===0?0:v;}"
            "var LEVEL_TOL=1;" // degrees - matches the badge's "close enough" threshold
            "function poll(){"
            "fetch('/live').then(function(r){return r.json();}).then(function(d){"
            "document.getElementById('p').innerHTML=(d.valid?fmt0(d.pitch):'--')+\"<span class='deg'>&deg;</span>\";"
            "document.getElementById('r').innerHTML=(d.valid?fmt0(d.roll):'--')+\"<span class='deg'>&deg;</span>\";"
            "document.getElementById('pCm').textContent='\\u2248 '+(d.valid?d.pitchCm.toFixed(1):'--')+' cm';"
            "document.getElementById('rCm').textContent='\\u2248 '+(d.valid?d.rollCm.toFixed(1):'--')+' cm';"
            "document.getElementById('temp').textContent=d.valid?d.temp.toFixed(1):'--';"
            "document.getElementById('rssi').textContent=d.rssi?d.rssi+' dBm':'(own hotspot)';"
            "document.getElementById('pitchIcon').style.transform="
            "'rotate('+(d.valid?d.pitchAngle:0)+'deg)';"
            "document.getElementById('rollIcon').style.transform="
            "'rotate('+(d.valid?d.rollAngle:0)+'deg)';"
            "document.getElementById('time').textContent=d.time;"
            "document.getElementById('date').textContent=d.date;"
            "var badge=document.getElementById('levelBadge');"
            "if(!d.valid){badge.textContent='--';badge.className='badge';}"
            "else if(Math.abs(d.pitch)<=LEVEL_TOL&&Math.abs(d.roll)<=LEVEL_TOL){"
            "badge.textContent='\\u2713 LEVEL';badge.className='badge level';"
            "}else{badge.textContent='ADJUSTING';badge.className='badge';}"
            "});"
            "}"
            "poll();setInterval(poll,400);"
            "</script>"
            "</body></html>";

    server.send(200, "text/html", page);
}

static void handleRoot()
{
    if (g_monitorMode) { handleMonitorRoot(); return; }

    Serial.printf("[WIFI-PORTAL] GET %s from %s (heap free=%u)\n",
                  server.uri().c_str(), server.client().remoteIP().toString().c_str(),
                  (unsigned)ESP.getFreeHeap());

    String displayName = g_deviceName.length() > 0 ? htmlEscape(g_deviceName) : "TiltDash";

    String page;
    page.reserve(5500); // room for the timezone <option> list

    page += "<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width, initial-scale=1'>"
            "<title>" + displayName + " - Setup</title>"
            "<style>"
            "*{box-sizing:border-box}"
            "body{font-family:-apple-system,'Segoe UI',Roboto,sans-serif;background:#111;color:#eee;"
            "padding:24px 16px;max-width:420px;margin:auto;text-align:center}"
            "header{margin-bottom:24px}"
            "h1{font-size:19px;font-weight:600;margin:0;letter-spacing:.3px}"
            ".sub{margin-top:6px;font-size:12px;color:#7b7d7d;text-transform:uppercase;letter-spacing:1px}"
            ".card{background:#1c1c1c;border:1px solid #2a2a2a;border-radius:14px;"
            "padding:18px 16px;box-shadow:0 4px 14px rgba(0,0,0,.35);margin-bottom:16px;text-align:left}"
            ".card h2{font-size:13px;font-weight:600;color:#7b7d7d;text-transform:uppercase;"
            "letter-spacing:1px;margin:0 0 14px}"
            "label{display:block;margin-top:14px;font-size:13px;color:#999}"
            "label:first-of-type{margin-top:0}"
            "select,input{width:100%;padding:11px;margin-top:5px;"
            "background:#111;color:#eee;border:1px solid #333;border-radius:8px;font-size:15px}"
            "select:focus,input:focus{outline:none;border-color:#7b7d7d}"
            "button{width:100%;padding:13px;margin-top:16px;background:#7b7d7d;color:#111;"
            "border:none;border-radius:8px;font-size:15px;font-weight:600}"
            "button.secondary{background:transparent;color:#ccc;border:1px solid #333;margin-top:8px}"
            "</style></head><body>"
            "<header><h1>" + displayName + "</h1><div class='sub'>Setup</div></header>"
            "<div class='card'>"
            "<h2>WiFi network</h2>"
            "<form action='/save' method='POST'>"
            "<label>Detected networks</label>"
            "<select id='ssid_scan' onchange=\"document.getElementById('ssid').value=this.value\">"
            "<option value=''>-- scanning... --</option>"
            "</select>"
            "<button type='button' class='secondary' onclick='scanNow()'>Rescan</button>"
            "<label>SSID (or type manually, e.g. hidden network)</label>"
            "<input type='text' name='ssid' id='ssid' maxlength='63'>"
            "<label>Password</label>"
            "<input type='password' name='pass' maxlength='63'>"
            "<button type='submit'>Save and connect</button>"
            "</form>"
            "</div>"
            "<div class='card'>"
            "<h2>Timezone</h2>"
            "<form action='/save_tz' method='POST'>"
            "<label>Independent of WiFi - saving this does not require a network</label>"
            "<select name='tz'>";
    for (size_t i = 0; i < TZ_OPTIONS_COUNT; i++) {
        page += "<option value='";
        page += String(TZ_OPTIONS[i].minutes);
        page += "'";
        if (TZ_OPTIONS[i].minutes == g_tzOffsetMin) page += " selected";
        page += ">";
        page += TZ_OPTIONS[i].label;
        page += "</option>";
    }
    page += "</select>"
            "<button type='submit'>Save timezone</button>"
            "</form>"
            "</div>"
            "<div class='card'>"
            "<h2>Device name</h2>"
            "<form action='/save_name' method='POST'>"
            "<label>Used for the WiFi hotspot name and the monitor's .local address - "
            "keep it unique if you have more than one TiltDash</label>"
            "<input type='text' name='name' maxlength='32' placeholder='e.g. My Camper' value='"
            + htmlEscape(g_deviceName) + "'>"
            "<button type='submit'>Save name</button>"
            "</form>"
            "</div>"
            "<div class='card'>"
            "<h2>Vehicle dimensions</h2>"
            "<form action='/save_dims' method='POST'>"
            "<label>Wheelbase, cm (front-back axle spacing - used for the pitch estimate)</label>"
            "<input type='number' name='wheelbase' min='1' step='1' value='"
            + String(g_wheelbaseCm, 0) + "'>"
            "<label>Track width, cm (left-right wheel spacing - used for the roll estimate)</label>"
            "<input type='number' name='track' min='1' step='1' value='"
            + String(g_trackCm, 0) + "'>"
            "<button type='submit'>Save dimensions</button>"
            "</form>"
            "</div>"
            "<script>"
            "function scanNow(){"
            "var sel=document.getElementById('ssid_scan');"
            "sel.innerHTML=\"<option value=''>-- scanning... --</option>\";"
            "fetch('/scan').then(function(r){return r.json();}).then(function(list){"
            "sel.innerHTML=\"<option value=''>-- choose (or type below) --</option>\";"
            "list.forEach(function(n){"
            "var o=document.createElement('option');"
            "o.value=n.ssid;o.textContent=n.ssid+' ('+n.rssi+' dBm)';"
            "sel.appendChild(o);"
            "});"
            "if(list.length===0){"
            "var o=document.createElement('option');"
            "o.value='';o.textContent='-- none found, tap Rescan --';"
            "sel.appendChild(o);"
            "}"
            "}).catch(function(){"
            "sel.innerHTML=\"<option value=''>-- scan failed, tap Rescan --</option>\";"
            "});"
            "}"
            "scanNow();"
            "</script>"
            "</body></html>";

    server.send(200, "text/html", page);
}

static void handleSave()
{
    String ssid = server.arg("ssid");
    String pass = server.arg("pass");

    if (ssid.length() == 0) {
        server.send(400, "text/plain", "Missing SSID.");
        return;
    }

    prefsWifi.begin(NVS_NAMESPACE, false);
    prefsWifi.putString("ssid", ssid);
    prefsWifi.putString("pass", pass);
    prefsWifi.end();

    server.send(200, "text/html",
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'></head>"
        "<body style='font-family:sans-serif;background:#111;color:#eee;padding:20px'>"
        "<h2>Saved.</h2><p>Restarting and connecting to the network...</p>"
        "</body></html>");

    Serial.printf("[WIFI-PORTAL] Saved SSID='%s'. Restarting...\n", ssid.c_str());
    delay(1200);
    ESP.restart();
}

// Deliberately separate from handleSave(): that one requires a non-empty
// SSID to submit at all, so a combined form made it impossible to change
// only the timezone without also re-entering (or blanking out) the WiFi
// network. This endpoint only ever touches "tzmin".
static void handleSaveTz()
{
    int tzMin = server.arg("tz").toInt();

    prefsWifi.begin(NVS_NAMESPACE, false);
    prefsWifi.putInt("tzmin", tzMin);
    prefsWifi.end();
    g_tzOffsetMin = tzMin;

    server.send(200, "text/html",
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'></head>"
        "<body style='font-family:sans-serif;background:#111;color:#eee;padding:20px'>"
        "<h2>Saved.</h2><p>Restarting...</p>"
        "</body></html>");

    Serial.printf("[WIFI-PORTAL] Saved tz=%d min (raw arg='%s'). Restarting...\n",
                  tzMin, server.arg("tz").c_str());
    delay(1200);
    ESP.restart();
}

// Independent of WiFi/timezone, same reasoning as handleSaveTz(). No
// restart needed - the AP SSID and mDNS hostname are only (re)built the
// next time wifi_portal_start_ap()/wifi_monitor_start() runs, i.e. the
// next time the setup or monitor screen is entered.
static void handleSaveName()
{
    String name = server.arg("name");
    name.trim();
    if (name.length() > 32) name = name.substring(0, 32);

    prefsWifi.begin(NVS_NAMESPACE, false);
    prefsWifi.putString("devname", name);
    prefsWifi.end();
    g_deviceName = name;

    server.send(200, "text/html",
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'></head>"
        "<body style='font-family:sans-serif;background:#111;color:#eee;padding:20px'>"
        "<h2>Saved.</h2><p>Takes effect next time you open the setup or monitor screen.</p>"
        "</body></html>");

    Serial.printf("[WIFI-PORTAL] Saved device name='%s'\n", name.c_str());
}

// Independent form, same reasoning as handleSaveName()/handleSaveTz() -
// takes effect on the monitor page's next /live poll, no restart needed.
static void handleSaveDims()
{
    float wheelbase = server.arg("wheelbase").toFloat();
    float track     = server.arg("track").toFloat();
    if (wheelbase <= 0) wheelbase = 350.0f;
    if (track <= 0)     track     = 180.0f;

    prefsWifi.begin(NVS_NAMESPACE, false);
    prefsWifi.putFloat("wheelbase", wheelbase);
    prefsWifi.putFloat("track", track);
    prefsWifi.end();
    g_wheelbaseCm = wheelbase;
    g_trackCm     = track;

    server.send(200, "text/html",
        "<!DOCTYPE html><html><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'></head>"
        "<body style='font-family:sans-serif;background:#111;color:#eee;padding:20px'>"
        "<h2>Saved.</h2><p>Used for the cm estimate on the monitor page.</p>"
        "</body></html>");

    Serial.printf("[WIFI-PORTAL] Saved dimensions: wheelbase=%.0fcm track=%.0fcm\n", wheelbase, track);
}

// Registers every HTTP route once - shared between the setup portal (its
// own AP) and the remote monitor (either that same AP as a fallback, or
// directly over an existing STA connection). server.stop() doesn't clear
// the handler list, so calling this more than once would just duplicate
// entries.
static void ensureHttpHandlersRegistered()
{
    if (g_httpHandlersRegistered) return;
    server.on("/", HTTP_GET, handleRoot);
    server.on("/scan", HTTP_GET, handleScan);
    server.on("/save", HTTP_POST, handleSave);
    server.on("/save_tz", HTTP_POST, handleSaveTz);
    server.on("/save_name", HTTP_POST, handleSaveName);
    server.on("/save_dims", HTTP_POST, handleSaveDims);
    server.on("/live", HTTP_GET, handleLive);
    server.on("/side.bmp", HTTP_GET, handleSideImg);
    server.on("/back.bmp", HTTP_GET, handleBackImg);
    // Captive portal: any other path also serves the current page, so the
    // phone/OS detects the portal and pops the sign-in prompt itself.
    server.onNotFound(handleRoot);
    g_httpHandlersRegistered = true;
}

// ====== API ======

void wifi_portal_init()
{
    prefsWifi.begin(NVS_NAMESPACE, true);
    g_ssid = prefsWifi.getString("ssid", "");
    g_pass = prefsWifi.getString("pass", "");
    g_tzOffsetMin = prefsWifi.getInt("tzmin", 60);
    g_deviceName = prefsWifi.getString("devname", "");
    g_wheelbaseCm = prefsWifi.getFloat("wheelbase", 350.0f);
    g_trackCm     = prefsWifi.getFloat("track", 180.0f);
    prefsWifi.end();
}

const char* wifi_portal_get_device_name() { return g_deviceName.c_str(); }

int wifi_portal_get_tz_offset_min() { return g_tzOffsetMin; }

bool wifi_portal_has_credentials()
{
    return g_ssid.length() > 0;
}

const char* wifi_portal_get_ssid() { return g_ssid.c_str(); }
const char* wifi_portal_get_pass() { return g_pass.c_str(); }

void wifi_portal_start_ap()
{
    if (g_apActive) return;

    Serial.println("[WIFI-PORTAL] Starting setup AP...");

    // Registered once - lets us see in the serial log whether a phone
    // actually joins the hotspot at all, independent of whether it goes
    // on to make any HTTP request.
    static bool eventHandlerRegistered = false;
    if (!eventHandlerRegistered) {
        WiFi.onEvent([](arduino_event_id_t event, arduino_event_info_t info) {
            if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) {
                Serial.println("[WIFI-PORTAL] Phone/station joined the AP");
            } else if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) {
                Serial.println("[WIFI-PORTAL] Phone/station left the AP");
            }
        });
        eventHandlerRegistered = true;
    }

    WiFi.persistent(false); // don't wear out flash with mode churn

    // The portal takes full ownership of the radio for the whole setup
    // session: stop the core's own reconnect logic so it can't fight our
    // mode/scan sequencing from the background.
    WiFi.setAutoReconnect(false);

    // If wifi_ntp_update_state() has a saved network it can't reach (out
    // of range, wrong password, ...), the STA sits permanently in a
    // "connecting" state - "sta is connecting, return error" in the
    // serial log. Plain WiFi.disconnect() is a no-op in that state: this
    // core's STAClass::disconnect() only calls esp_wifi_disconnect() when
    // connected() is already true, so a stuck connect attempt is never
    // actually cancelled by it. wifioff=true forces the STA interface
    // down regardless of connection state, so the AP_STA brought up below
    // starts from a clean slate instead of inheriting a wedged STA.
    // eraseap=true also clears the STA config esp_wifi keeps in its own
    // internal (non-Arduino) NVS blob - separate from our own "wifinet"
    // Preferences namespace, so the saved credentials used elsewhere in
    // the app are untouched - which stops the driver from silently trying
    // to auto-rejoin that old network on its own.
    WiFi.disconnect(true /* wifioff */, true /* eraseap */);
    delay(500);

    g_apSsid = buildApSsid();

    // Bring STA up on its own first, then add AP on top with a second
    // mode() call, rather than jumping straight from the disconnect(true)
    // above (which tears the interface down to WIFI_OFF) into a combined
    // mode. Going directly from WIFI_OFF into a mode that then starts the
    // AP is what caused a netif-recreation crash in wifi_softap_start
    // (Guru Meditation / LoadProhibited, EXCVADDR near 0) - see git log
    // for that debugging session. This staged sequence avoids it.
    WiFi.mode(WIFI_STA);
    delay(200);

    WiFi.mode(WIFI_AP_STA);
    delay(100);

    Serial.printf("[WIFI-PORTAL] Free heap before softAP(): %u\n", (unsigned)ESP.getFreeHeap());

    // Open hotspot (no password) - this is only the setup mode, meant to
    // be trivially reachable from a phone; it stays up until the user
    // either saves credentials (which triggers a restart) or leaves the
    // setup screen.
    bool apOk = WiFi.softAP(g_apSsid.c_str());
    if (!apOk) {
        Serial.println("[WIFI-PORTAL] WiFi.softAP() FAILED - retrying once");
        delay(200);
        apOk = WiFi.softAP(g_apSsid.c_str());
        Serial.printf("[WIFI-PORTAL] retry %s\n", apOk ? "OK" : "FAILED");
    }
    g_apIp = WiFi.softAPIP();

    // Drop STA back off now that the AP is up: keeping it alive full-time
    // (WIFI_AP_STA) costs its own connection buffers/queues on top of the
    // AP + WebServer + DNSServer + QR canvas, and on this board (no
    // PSRAM, ~84% RAM already static) that was enough to starve lwIP -
    // the phone could join the hotspot (L2/DHCP) but the HTTP request for
    // the portal page never completed (free heap measured ~10KB with STA
    // left on, vs comfortably more with it off). handleScan() re-adds STA
    // only for the few seconds an actual scan runs. The AP bit is never
    // removed here, so its beacon/netif stays up throughout.
    WiFi.mode(WIFI_AP);

    dnsServer.start(DNS_PORT, "*", g_apIp);

    ensureHttpHandlersRegistered();
    server.begin();

    g_apActive = true;

    Serial.printf("[WIFI-PORTAL] AP '%s' ready at %s\n",
                  g_apSsid.c_str(), g_apIp.toString().c_str());
}

void wifi_portal_stop_ap()
{
    if (!g_apActive) return;

    server.stop();
    dnsServer.stop();
    WiFi.softAPdisconnect(true);

    g_apActive = false;

    Serial.println("[WIFI-PORTAL] AP stopped.");
}

bool wifi_portal_is_active() { return g_apActive; }

void wifi_portal_loop()
{
    if (!g_apActive) return;
    dnsServer.processNextRequest();
    server.handleClient();
}

const char* wifi_portal_ap_ssid() { return g_apSsid.c_str(); }
IPAddress wifi_portal_ap_ip() { return g_apIp; }

// ====== Remote monitor ======

void wifi_monitor_start()
{
    g_monitorMode = true;

    if (WiFi.status() == WL_CONNECTED) {
        // Already on a real network - serve straight from that, no AP
        // needed. wifi_portal_start_ap() is NOT involved here at all, so
        // this path is cheap: just the WebServer, no mode churn.
        g_monitorUsingOwnAp = false;
        ensureHttpHandlersRegistered();
        server.begin();
        g_monitorServing = true;

        g_mdnsHostname = buildMdnsHostname();
        if (MDNS.begin(g_mdnsHostname.c_str())) {
            MDNS.addService("http", "tcp", 80);
            Serial.printf("[MONITOR] Serving on existing WiFi at %s / %s.local\n",
                          WiFi.localIP().toString().c_str(), g_mdnsHostname.c_str());
        } else {
            Serial.println("[MONITOR] mDNS start failed - IP address still works");
        }
    } else {
        // No network to piggyback on - fall back to the same hotspot the
        // setup screen uses. handleRoot() already checks g_monitorMode
        // (set above) and will serve the live-data page instead of the
        // WiFi setup form even though this is the same AP machinery.
        g_monitorUsingOwnAp = true;
        wifi_portal_start_ap();
        Serial.printf("[MONITOR] No STA connection - serving from own AP '%s' at %s\n",
                      g_apSsid.c_str(), g_apIp.toString().c_str());
    }
}

void wifi_monitor_stop()
{
    g_monitorMode = false;

    if (g_monitorUsingOwnAp) {
        wifi_portal_stop_ap();
    } else if (g_monitorServing) {
        server.stop();
        MDNS.end();
        g_monitorServing = false;
    }
    g_monitorUsingOwnAp = false;

    Serial.println("[MONITOR] Stopped.");
}

bool wifi_monitor_is_active()
{
    return g_monitorServing || (g_monitorUsingOwnAp && g_apActive);
}

void wifi_monitor_service()
{
    // The AP case is already serviced by wifi_portal_loop() (DNS + the
    // same server.handleClient()) - this only covers the STA-direct case.
    if (g_monitorServing) server.handleClient();
}

bool wifi_monitor_using_own_ap() { return g_monitorUsingOwnAp; }

IPAddress wifi_monitor_ip()
{
    return g_monitorUsingOwnAp ? g_apIp : WiFi.localIP();
}

const char* wifi_monitor_hostname() { return g_mdnsHostname.c_str(); }

void wifi_monitor_set_time(const char* time_str, const char* date_str)
{
    g_monitorTimeStr = time_str;
    g_monitorDateStr = date_str;
}
