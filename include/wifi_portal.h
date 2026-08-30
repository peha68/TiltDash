#pragma once

#include <Arduino.h>
#include <IPAddress.h>

// ====== WiFi configuration portal ======
// Instead of baking SSID/password into the firmware (wifi_credentials.h,
// requiring a recompile for every network change), credentials are stored
// in NVS (Preferences). When they're missing - or on the user's request -
// the device starts its own hotspot (AP) with a captive portal: a phone
// connects to that hotspot, gets a page listing nearby networks, and once
// saved the device restarts and connects normally as a client (STA).

// Loads saved WiFi credentials from NVS (if any). Call once in setup().
void wifi_portal_init();

// Whether an SSID is saved in NVS (i.e. there's something to try to
// connect to).
bool wifi_portal_has_credentials();

// Saved credentials - valid only after wifi_portal_init(), when
// wifi_portal_has_credentials() returns true.
const char* wifi_portal_get_ssid();
const char* wifi_portal_get_pass();

// Saved UTC offset in minutes (e.g. 120 for UTC+2 / CEST) - configurable
// from the same setup page as the WiFi credentials, since a fixed offset
// baked into firmware means a recompile every DST transition. Valid
// after wifi_portal_init(); defaults to +60 (UTC+1) if never set.
int wifi_portal_get_tz_offset_min();

// Starts setup mode: AP + DNS (captive portal) + web server with the
// config form. Blocking for a couple of seconds (WiFi scan) - see the
// comment on scanNetworksIntoHtml() in wifi_portal.cpp. Safe to call
// repeatedly (no-op if the AP is already active).
void wifi_portal_start_ap();

// Stops the AP/portal (web server, DNS, hotspot). Call when leaving the
// setup screen if the user didn't save anything.
void wifi_portal_stop_ap();

// Whether AP/portal mode is currently active.
bool wifi_portal_is_active();

// Call on every loop() iteration while wifi_portal_is_active() == true.
void wifi_portal_loop();

// Hotspot name and IP address - for showing on the setup screen.
const char* wifi_portal_ap_ssid();
IPAddress wifi_portal_ap_ip();
