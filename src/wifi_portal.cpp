#include "wifi_portal.h"

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

// ====== NVS ======
static Preferences prefsWifi;
static constexpr const char* NVS_NAMESPACE = "wifinet";

static String g_ssid;
static String g_pass;

// ====== AP / portal ======
static WebServer   server(80);
static DNSServer   dnsServer;
static bool        g_apActive = false;
static String      g_apSsid;
static IPAddress   g_apIp;
static const uint32_t DNS_PORT = 53;

// ====== Helpers ======

static String buildApSsid()
{
    // Unique hotspot name (MAC suffix) so multiple devices nearby don't
    // collide on the same SSID.
    uint64_t mac = ESP.getEfuseMac();
    char buf[32];
    snprintf(buf, sizeof(buf), "TiltDash-%04X", (unsigned)(mac & 0xFFFF));
    return String(buf);
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

static void handleRoot()
{
    Serial.printf("[WIFI-PORTAL] GET %s from %s (heap free=%u)\n",
                  server.uri().c_str(), server.client().remoteIP().toString().c_str(),
                  (unsigned)ESP.getFreeHeap());

    String page;
    page.reserve(1600);

    page += "<!DOCTYPE html><html><head><meta charset='utf-8'>"
            "<meta name='viewport' content='width=device-width, initial-scale=1'>"
            "<title>TiltDash - WiFi setup</title>"
            "<style>"
            "body{font-family:sans-serif;background:#111;color:#eee;padding:20px;max-width:420px;margin:auto}"
            "h1{font-size:20px}"
            "label{display:block;margin-top:14px;font-size:14px;color:#aaa}"
            "select,input{width:100%;padding:10px;margin-top:4px;box-sizing:border-box;"
            "background:#222;color:#eee;border:1px solid #444;border-radius:6px;font-size:16px}"
            "button{width:100%;padding:12px;margin-top:20px;background:#7b7d7d;color:#111;"
            "border:none;border-radius:6px;font-size:16px;font-weight:bold}"
            "button.secondary{background:#333;color:#eee;margin-top:8px}"
            "</style></head><body>"
            "<h1>TiltDash &ndash; WiFi setup</h1>"
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

// ====== API ======

void wifi_portal_init()
{
    prefsWifi.begin(NVS_NAMESPACE, true);
    g_ssid = prefsWifi.getString("ssid", "");
    g_pass = prefsWifi.getString("pass", "");
    prefsWifi.end();
}

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

    // DIAGNOSTIC: every scan attempted so far through /scan (STA added
    // back on top of an already-running AP) has come back with exactly 0
    // networks, heap and timing both fine - which on ESP32 can mean a
    // scan while AP+STA are concurrent gets limited to something less
    // than a real full-channel sweep. This one-shot scan runs in plain
    // STA mode with no AP up at all yet, to tell whether that's really
    // AP_STA-specific or the radio finds nothing regardless of mode.
    int probeScan = WiFi.scanNetworks(false, true, false, 500, 0);
    Serial.printf("[WIFI-PORTAL] DIAGNOSTIC pre-AP probe scan (STA only, no AP) returned %d, mode=%d, status=%d\n",
                  probeScan, (int)WiFi.getMode(), (int)WiFi.status());
    for (int i = 0; i < probeScan; i++) {
        Serial.printf("[WIFI-PORTAL]   probe [%d] SSID='%s' RSSI=%d\n", i, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    }
    WiFi.scanDelete();

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

    // Handlers are registered only once - the setup screen can be entered
    // and left multiple times per session, and server.stop() doesn't
    // clear the handler list, so re-registering would just duplicate them.
    static bool handlersRegistered = false;
    if (!handlersRegistered) {
        server.on("/", HTTP_GET, handleRoot);
        server.on("/scan", HTTP_GET, handleScan);
        server.on("/save", HTTP_POST, handleSave);
        // Captive portal: any other path also serves the form, so the
        // phone/OS detects the portal and pops the sign-in prompt itself.
        server.onNotFound(handleRoot);
        handlersRegistered = true;
    }
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
