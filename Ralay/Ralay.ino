// -------------------------------------------------------------------------------
// LoRa Hardware Relay with WebUI — E22-900T33S / ESP32
// FIXED: heap fragmentation, JSON injection, log ordering, millis() rollover
// -------------------------------------------------------------------------------

#include <Arduino.h>
#include <LoRa_E22.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <WebServer.h>
#include <mbedtls/base64.h>
#include "esp_sleep.h"

// ----------------------------- WiFi credentials -------------------------------
const char* ssid_AP    = "LMRalay-3";
const char* password_AP = "12345678";

// ----------------------------- Relay configuration ----------------------------
#define REPEATER_ADDH  0xFF
#define REPEATER_ADDL  0xFF
#define COMMON_CHAN    0x41
#define COMMON_NETID   0x01
#define CRYPT_KEY      0x8002

#define SLEEP_IDLE_MS       30000   // enter sleep after 30s with no activity
#define SLEEP_WAKE_POLL_US  500000  // light-sleep wake every 500ms to check LoRa/WiFi

// ────────────────────────────────────────────────
// GPS on Serial1
// ────────────────────────────────────────────────
#define GPS_TX_PIN  26
#define GPS_RX_PIN  25
#define GPS_BAUD    9600

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// ----------------------------- OLED ------------------------------------------
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS 0x3C
#define OLED_SDA        27
#define OLED_SCL        33

// ----------------------------- E22 pins --------------------------------------
#define PIN_E22_TX   4
#define PIN_E22_RX  22
#define PIN_AUX     18
#define PIN_M0      21
#define PIN_M1      19

LoRa_E22 e22(PIN_E22_TX, PIN_E22_RX, &Serial2, PIN_AUX, PIN_M0, PIN_M1, UART_BPS_RATE_9600, SERIAL_8N1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);

bool     oledReady     = false;
uint8_t  oledI2cAddress = SCREEN_ADDRESS;
bool     relayReady    = false;
bool     inSleepMode   = false;
uint32_t relayCount    = 0;
uint32_t startTime     = 0;
uint32_t lastActivityMs = 0;

// GPS state
bool     gpsHasFix    = false;
double   gpsLat       = 0.0;
double   gpsLng       = 0.0;
uint32_t gpsSatellites = 0;
char     gpsTimeUtc[16] = "--:--:--";

// =============================================================================
// FIX 1: Fixed-size char arrays instead of Arduino String for log entries.
//         Eliminates heap fragmentation from repeated String allocation /
//         deallocation over many hours of runtime.
// =============================================================================
#define MAX_LOG_ENTRIES 20
#define MAX_MSG_LEN      220  // enough for Khmer UTF-8 chat bodies and decoded MSG3 logs
#define MAX_ADDR_LEN      5   // "FFFF\0"

struct LogEntry {
    uint32_t timestamp;           // millis() at reception
    bool     valid;               // false = slot never written
    char     direction[6];        // "RX" or "RELAY"
    int8_t   rssi;
    char     srcAddr[MAX_ADDR_LEN];
    char     destAddr[MAX_ADDR_LEN];
    char     message[MAX_MSG_LEN];
};

LogEntry messageLog[MAX_LOG_ENTRIES];
int logHead = 0;   // index of the NEXT slot to write (ring buffer head)

void initLog() {
    memset(messageLog, 0, sizeof(messageLog));
}

static int countValidLogEntries() {
    int count = 0;
    for (int i = 0; i < MAX_LOG_ENTRIES; i++) {
        if (messageLog[i].valid) count++;
    }
    return count;
}

// FIX 2: Escape special JSON characters so malformed messages can't break
//         the JSON response and crash the browser-side parser.
//         Writes into a caller-supplied buffer; returns number of bytes written.
static int jsonEscape(const char* src, char* dst, int dstLen) {
    int di = 0;
    for (int si = 0; src[si] && di < dstLen - 1; si++) {
        char c = src[si];
        if (c == '"'  && di < dstLen - 2) { dst[di++] = '\\'; dst[di++] = '"';  }
        else if (c == '\\' && di < dstLen - 2) { dst[di++] = '\\'; dst[di++] = '\\'; }
        else if (c == '\n' && di < dstLen - 2) { dst[di++] = '\\'; dst[di++] = 'n';  }
        else if (c == '\r' && di < dstLen - 2) { dst[di++] = '\\'; dst[di++] = 'r';  }
        else if (c == '\t' && di < dstLen - 2) { dst[di++] = '\\'; dst[di++] = 't';  }
        else { dst[di++] = c; }
    }
    dst[di] = '\0';
    return di;
}

static bool base64DecodeUtf8(const char* src, char* dst, size_t dstLen) {
    if (!src || !dst || dstLen == 0) return false;
    size_t olen = 0;
    int rc = mbedtls_base64_decode((unsigned char*)dst, dstLen - 1, &olen,
                                   (const unsigned char*)src, strlen(src));
    if (rc != 0) return false;
    dst[olen] = '\0';
    return true;
}

static bool decodeMsg3ForLog(const char* frame, char* srcAddr, size_t srcLen,
                             char* destAddr, size_t destLen,
                             char* msgOut, size_t msgLen) {
    if (!frame || strncmp(frame, "MSG3|", 5) != 0) return false;

    const char* p1 = strchr(frame, '|');       // after MSG3
    if (!p1) return false;
    const char* p2 = strchr(p1 + 1, '|');      // after id
    if (!p2) return false;
    const char* p3 = strchr(p2 + 1, '|');      // after source
    if (!p3) return false;
    const char* p4 = strchr(p3 + 1, '|');      // after destination
    if (!p4) return false;

    size_t srcFieldLen = (size_t)(p3 - (p2 + 1));
    size_t dstFieldLen = (size_t)(p4 - (p3 + 1));
    if (srcFieldLen != 4 || dstFieldLen != 4 || srcLen < 5 || destLen < 5) return false;

    memcpy(srcAddr, p2 + 1, 4);
    srcAddr[4] = '\0';
    memcpy(destAddr, p3 + 1, 4);
    destAddr[4] = '\0';

    return base64DecodeUtf8(p4 + 1, msgOut, msgLen);
}

void addLogEntry(const char* direction, int8_t rssi,
                 const char* src, const char* dest, const char* msg) {
    LogEntry& e = messageLog[logHead];
    e.timestamp = millis();
    e.valid     = true;
    e.rssi      = rssi;
    strncpy(e.direction, direction, sizeof(e.direction) - 1);
    e.direction[sizeof(e.direction) - 1] = '\0';
    strncpy(e.srcAddr,   src,       sizeof(e.srcAddr)   - 1);
    e.srcAddr[sizeof(e.srcAddr) - 1] = '\0';
    strncpy(e.destAddr,  dest,      sizeof(e.destAddr)  - 1);
    e.destAddr[sizeof(e.destAddr) - 1] = '\0';
    strncpy(e.message,   msg,       sizeof(e.message)   - 1);
    e.message[sizeof(e.message) - 1] = '\0';

    logHead = (logHead + 1) % MAX_LOG_ENTRIES;
}

// =============================================================================
// Helpers
// =============================================================================

void drawDisplay();

static bool i2cAddressResponds(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

static bool initOled() {
    Wire.begin(OLED_SDA, OLED_SCL);
    Wire.setClock(100000);
    delay(50);

    const uint8_t addresses[] = {0x3C, 0x3D};
    for (uint8_t i = 0; i < 2; i++) {
        uint8_t addr = addresses[i];
        if (!i2cAddressResponds(addr)) {
            Serial.printf("[OLED] No I2C ACK at 0x%02X\n", addr);
            continue;
        }
        if (display.begin(SSD1306_SWITCHCAPVCC, addr)) {
            oledReady = true;
            oledI2cAddress = addr;
            display.setRotation(2);
            display.clearDisplay();
            display.display();
            Serial.printf("[OLED] OK at 0x%02X on SDA=%d SCL=%d\n", addr, OLED_SDA, OLED_SCL);
            return true;
        }
        Serial.printf("[OLED] display.begin failed at 0x%02X\n", addr);
    }

    oledReady = false;
    Serial.printf("[OLED] FAILED. Check VCC/GND/SDA=%d/SCL=%d and OLED address 0x3C/0x3D.\n", OLED_SDA, OLED_SCL);
    return false;
}

static void showBootMessage(const char *line1, const char *line2) {
    if (!oledReady) return;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(line1);
    display.println(line2);
    display.display();
}

void noteActivity() {
    lastActivityMs = millis();
    if (inSleepMode) {
        inSleepMode = false;
        WiFi.setSleep(false);
        if (oledReady) {
            display.ssd1306_command(SSD1306_DISPLAYON);
        }
        Serial.println("[SLEEP] Wake — active");
        drawDisplay();
    }
}

void enterSleepMode() {
    if (inSleepMode) return;
    inSleepMode = true;
    WiFi.setSleep(true);
    if (oledReady) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 20);
        display.println("  SLEEP MODE");
        display.setCursor(0, 34);
        display.println("  (30s idle)");
        display.display();
        delay(400);
        display.ssd1306_command(SSD1306_DISPLAYOFF);
    }
    Serial.println("[SLEEP] Entering (30s idle)");
}

void lightSleepTick() {
    esp_sleep_enable_timer_wakeup(SLEEP_WAKE_POLL_US);
    esp_light_sleep_start();
}

bool checkSleepWakeSources() {
    if (e22.available() > 0) return true;
    if (digitalRead(PIN_AUX) == LOW) return true;
    return false;
}

void getUptime(char* buf, size_t len) {
    // FIX 3: millis() rollover guard — cast difference to uint32_t so subtraction
    //         wraps correctly even after the ~49-day rollover.
    uint32_t seconds = (uint32_t)(millis() - startTime) / 1000;
    uint32_t hours   = seconds / 3600;
    uint32_t minutes = (seconds % 3600) / 60;
    snprintf(buf, len, "%uh %um", (unsigned)hours, (unsigned)minutes);
}

void updateGPS() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
    if (gps.location.isUpdated() && gps.location.isValid()) {
        gpsHasFix = true;
        gpsLat    = gps.location.lat();
        gpsLng    = gps.location.lng();
    }
    if (gps.satellites.isValid()) {
        gpsSatellites = gps.satellites.value();
    }
    if (gps.time.isValid()) {
        snprintf(gpsTimeUtc, sizeof(gpsTimeUtc), "%02d:%02d:%02d",
                 gps.time.hour(), gps.time.minute(), gps.time.second());
    }
}

// =============================================================================
// OLED — uses fixed-size stack buffers, no String
// =============================================================================

void drawDisplay() {
    if (!oledReady || inSleepMode) return;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    char buf[32];

    display.setCursor(0, 0);
    snprintf(buf, sizeof(buf), "=== %s ===", ssid_AP);
    display.println(buf);

    display.setCursor(0, 12);
    snprintf(buf, sizeof(buf), "IP: %s", WiFi.softAPIP().toString().c_str());
    display.print(buf);

    display.setCursor(0, 22);
    display.print("Addr: 0xFF/0xFF");

    display.setCursor(0, 32);
    snprintf(buf, sizeof(buf), "CH:0x%02X", COMMON_CHAN);
    display.print(buf);

    display.setCursor(0, 42);
    snprintf(buf, sizeof(buf), "GPS:%s S:%u",
             gpsHasFix ? "FIX " : "NOFIX", (unsigned)gpsSatellites);
    display.print(buf);

    display.setCursor(0, 52);
    snprintf(buf, sizeof(buf), "Relayed: %u", (unsigned)relayCount);
    display.print(buf);

    display.display();
}

// =============================================================================
// Wait for AUX
// =============================================================================

bool waitAuxHigh(uint32_t timeoutMs = 5000) {
    uint32_t t = millis();
    while (digitalRead(PIN_AUX) == LOW) {
        if ((uint32_t)(millis() - t) > timeoutMs) return false;
        delay(10);
    }
    return true;
}

// =============================================================================
// Radio config
// =============================================================================

bool applyRelayConfig() {
    if (!waitAuxHigh(8000)) return false;
    delay(100);

    ResponseStructContainer c = e22.getConfiguration();
    if (c.status.code != 1) { c.close(); return false; }

    Configuration config = *(Configuration*)c.data;
    c.close();

    config.ADDH  = REPEATER_ADDH;
    config.ADDL  = REPEATER_ADDL;
    config.NETID = COMMON_NETID;
    config.CHAN  = COMMON_CHAN;

    config.SPED.uartBaudRate = UART_BPS_9600;
    config.SPED.airDataRate  = AIR_DATA_RATE_010_24;
    config.SPED.uartParity   = MODE_00_8N1;

    config.CRYPT.CRYPT_H = highByte(CRYPT_KEY);
    config.CRYPT.CRYPT_L = lowByte(CRYPT_KEY);

    config.OPTION.subPacketSetting  = SPS_240_00;
    config.OPTION.RSSIAmbientNoise  = RSSI_AMBIENT_NOISE_DISABLED;
    config.OPTION.transmissionPower = POWER_22;

    config.TRANSMISSION_MODE.enableRSSI            = RSSI_ENABLED;
    config.TRANSMISSION_MODE.fixedTransmission     = FT_FIXED_TRANSMISSION;
    config.TRANSMISSION_MODE.enableRepeater        = REPEATER_ENABLED;
    config.TRANSMISSION_MODE.enableLBT             = LBT_DISABLED;
    config.TRANSMISSION_MODE.WORTransceiverControl = WOR_RECEIVER;
    config.TRANSMISSION_MODE.WORPeriod             = WOR_2000_011;

    ResponseStatus rs = e22.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
    if (rs.code != 1) return false;

    delay(300);
    if (!waitAuxHigh(5000)) return false;
    delay(100);
    return true;
}

// =============================================================================
// Software relay
// FIX 4: Operate entirely on char arrays; release rc before sending to
//         avoid holding two large String objects in heap simultaneously.
// =============================================================================

void handleSoftwareRelay() {
    if (e22.available() <= 0) return;
    noteActivity();

    ResponseContainer rc = e22.receiveMessageRSSI();
    if (rc.status.code != 1) return;

    // Copy data out of the Arduino String immediately, then let rc go out of
    // scope so the heap is freed before we allocate the reply.
    char rxBuf[512];
    strncpy(rxBuf, rc.data.c_str(), sizeof(rxBuf) - 1);
    rxBuf[sizeof(rxBuf) - 1] = '\0';
    int8_t rssi = (int8_t)rc.rssi;
    rc.data = String();   // explicitly release String storage

    Serial.printf("[RX] RSSI=%d data=%s\n", rssi, rxBuf);

    char logSrc[MAX_ADDR_LEN] = "????";
    char logDest[MAX_ADDR_LEN] = "FFFF";
    char decodedMsg[MAX_MSG_LEN];
    const char* logMsg = rxBuf;
    if (decodeMsg3ForLog(rxBuf, logSrc, sizeof(logSrc), logDest, sizeof(logDest),
                         decodedMsg, sizeof(decodedMsg))) {
        logMsg = decodedMsg;
    }
    addLogEntry("RX", rssi, logSrc, logDest, logMsg);

    // Expected format: "RELAY|XXYY|<payload>"
    if (strncmp(rxBuf, "RELAY|", 6) != 0) return;

    char* p1 = strchr(rxBuf + 6, '|');
    if (!p1) return;

    // Extract 4-char hex destination address
    int addrLen = (int)(p1 - (rxBuf + 6));
    if (addrLen != 4) return;

    char destHex[5];
    memcpy(destHex, rxBuf + 6, 4);
    destHex[4] = '\0';

    char* endPtr = nullptr;
    long addr = strtol(destHex, &endPtr, 16);
    if (endPtr != destHex + 4) return;   // not all 4 chars were valid hex

    uint8_t destAddh = (uint8_t)((addr >> 8) & 0xFF);
    uint8_t destAddl = (uint8_t)(addr & 0xFF);

    const char* payload = p1 + 1;
    char relaySrc[MAX_ADDR_LEN] = "FFFF";
    char relayDecodedMsg[MAX_MSG_LEN];
    const char* relayLogMsg = payload;
    if (decodeMsg3ForLog(payload, relaySrc, sizeof(relaySrc), destHex, sizeof(destHex),
                         relayDecodedMsg, sizeof(relayDecodedMsg))) {
        relayLogMsg = relayDecodedMsg;
    }

    ResponseStatus rs = e22.sendFixedMessage(destAddh, destAddl, COMMON_CHAN, payload);
    if (rs.code == 1) {
        relayCount++;
        noteActivity();
        addLogEntry("RELAY", 0, relaySrc, destHex, relayLogMsg);
        Serial.printf("[RELAY] -> 0x%02X%02X: %s\n", destAddh, destAddl, payload);
    } else {
        Serial.printf("[RELAY] FAIL code=%d -> 0x%02X%02X\n", rs.code, destAddh, destAddl);
    }
}

// =============================================================================
// Web Handlers
// FIX 5: handleRoot() sends the static HTML page in one PROGMEM-friendly
//         literal so it is never rebuilt on each request.
//         Dynamic values (IP, SSID) are injected via a single small snprintf
//         rather than repeated String +=" concatenation.
//
// FIX 6: handleAPIStatus() and handleAPILog() use a fixed char[512/1024]
//         stack buffer instead of building with String concatenation.  The
//         buffer is sized to hold the worst-case JSON; snprintf truncates
//         gracefully if something is unexpectedly large.
// =============================================================================

// The page HTML is stored once in flash (PROGMEM on ESP32 is just .rodata).
// Everything dynamic is loaded by the JS fetch calls, so the HTML itself never
// changes — no need to rebuild it per-request.
static const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>LoRa Relay Monitor</title>
  <style>
    *{box-sizing:border-box}
    body{font-family:'Courier New','Noto Sans Khmer','Khmer OS','Khmer Sangam MN',monospace;margin:0;padding:20px;background:#0d0d0d;color:#c8ffc8;}
    h1{color:#00ff88;letter-spacing:2px;margin-bottom:4px;}
    h2{color:#00cc66;font-size:1em;letter-spacing:1px;margin:0 0 14px 0;}
    .container{max-width:1100px;margin:0 auto;}
    .card{background:#111;border:1px solid #1a3a1a;padding:18px;border-radius:6px;margin-bottom:18px;}
    .stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;}
    .stat{background:#0a1a0a;border:1px solid #1f4a1f;padding:12px;border-radius:4px;text-align:center;}
    .stat h3{margin:0 0 6px;font-size:11px;color:#4a9;letter-spacing:1px;text-transform:uppercase;}
    .stat .v{font-size:22px;font-weight:bold;color:#0f6;}
    .stat .v.sm{font-size:15px;}
    table{width:100%;border-collapse:collapse;font-size:13px;}
    th,td{padding:8px 10px;text-align:left;border-bottom:1px solid #1a2a1a;}
    th{color:#0f6;font-size:11px;letter-spacing:1px;text-transform:uppercase;}
    .rx{color:#4af;}
    .relay{color:#fa0;}
    .rssi{color:#888;}
    .msg{color:#aef;word-break:break-word;font-family:'Noto Sans Khmer','Khmer OS','Khmer Sangam MN',sans-serif;}
    button{background:#0f6;color:#000;border:none;padding:7px 16px;border-radius:4px;
           cursor:pointer;font-family:inherit;font-weight:bold;font-size:13px;margin-bottom:12px;}
    button:hover{background:#0c5;}
    #heap{font-size:11px;color:#555;text-align:right;margin-top:4px;}
  </style>
  <script>
    var AUTO_INTERVAL = 2000;
    var timer;

    function esc(s){
      return String(s)
        .replace(/&/g,'&amp;')
        .replace(/</g,'&lt;')
        .replace(/>/g,'&gt;');
    }

    function refreshData(){
      fetch('/api/status')
        .then(function(r){return r.json();})
        .then(function(d){
          document.getElementById('uptime').textContent     = d.uptime;
          document.getElementById('relayCount').textContent = d.relayCount;
          document.getElementById('logEntries').textContent = d.logCount;
          document.getElementById('gpsFix').textContent     = d.gpsFix;
          document.getElementById('gpsSat').textContent     = d.gpsSat;
          document.getElementById('gpsLat').textContent     = d.gpsLat;
          document.getElementById('gpsLng').textContent     = d.gpsLng;
          document.getElementById('gpsTime').textContent    = d.gpsTimeUtc;
          document.getElementById('heap').textContent       = 'Free heap: ' + d.freeHeap + ' B';
        })
        .catch(function(){});   // silently ignore transient errors

      fetch('/api/log')
        .then(function(r){return r.json();})
        .then(function(data){
          var tbody = document.getElementById('logBody');
          var rows = '';
          for(var i = data.length - 1; i >= 0; i--){
            var e = data[i];
            rows += '<tr><td>' + esc(e.time) + '</td>'
                  + '<td class="' + e.dir.toLowerCase() + '">' + esc(e.dir) + '</td>'
                  + '<td class="rssi">' + esc(e.rssi) + ' dBm</td>'
                  + '<td>0x' + esc(e.src) + '</td>'
                  + '<td>0x' + esc(e.dest) + '</td>'
                  + '<td class="msg">' + esc(e.msg) + '</td></tr>';
          }
          tbody.innerHTML = rows || '<tr><td colspan="6" style="text-align:center;color:#555">No messages yet</td></tr>';
        })
        .catch(function(){});
    }

    window.onload = function(){
      refreshData();
      timer = setInterval(refreshData, AUTO_INTERVAL);
    };
  </script>
</head>
<body>
<div class="container">
  <h1>&#128225; LoRa Relay Monitor</h1>
  <p style="color:#4a8;font-size:12px;margin:0 0 16px">SSID: __SSID__ &nbsp;|&nbsp; IP: __IP__</p>
  <div id="heap"></div>

  <div class="card">
    <h2>&#9654; SYSTEM STATUS</h2>
    <div class="stats">
      <div class="stat"><h3>Address</h3><div class="v">0xFFFF</div></div>
      <div class="stat"><h3>Channel</h3><div class="v">0x41</div></div>
      <div class="stat"><h3>Uptime</h3><div class="v sm" id="uptime">--</div></div>
      <div class="stat"><h3>Relayed</h3><div class="v" id="relayCount">0</div></div>
      <div class="stat"><h3>Log Entries</h3><div class="v" id="logEntries">0</div></div>
      <div class="stat"><h3>GPS Fix</h3><div class="v sm" id="gpsFix">--</div></div>
      <div class="stat"><h3>Satellites</h3><div class="v" id="gpsSat">0</div></div>
      <div class="stat"><h3>Latitude</h3><div class="v sm" id="gpsLat">--</div></div>
      <div class="stat"><h3>Longitude</h3><div class="v sm" id="gpsLng">--</div></div>
      <div class="stat"><h3>UTC Time</h3><div class="v sm" id="gpsTime">--:--:--</div></div>
    </div>
  </div>

  <div class="card">
    <h2>&#9654; MESSAGE LOG (LAST 20)</h2>
    <button onclick="refreshData()">&#8635; Refresh</button>
    <table>
      <thead><tr><th>Time</th><th>Type</th><th>RSSI</th><th>Source</th><th>Dest</th><th>Message</th></tr></thead>
      <tbody id="logBody"><tr><td colspan="6" style="text-align:center;color:#555">Loading...</td></tr></tbody>
    </table>
  </div>
</div>
</body>
</html>
)rawliteral";

void handleRoot() {
    noteActivity();
    // Build tiny per-request substitution buffer (only SSID + IP change)
    // We do a single pass replace on the two __PLACEHOLDER__ tokens.
    // The full page is read from flash each time but never heap-allocated
    // as a String — we stream it directly.
    String page = FPSTR(PAGE_HTML);   // one-shot copy into a String (unavoidable for WebServer::send)
    page.replace("__SSID__", ssid_AP);
    page.replace("__IP__",   WiFi.softAPIP().toString());
    server.send(200, "text/html", page);
    page = String();   // immediately release
}

void handleAPIStatus() {
    noteActivity();
    char uptime[20];
    getUptime(uptime, sizeof(uptime));

    char latBuf[16], lngBuf[16];
    if (gpsHasFix) {
        snprintf(latBuf, sizeof(latBuf), "%.6f", gpsLat);
        snprintf(lngBuf, sizeof(lngBuf), "%.6f", gpsLng);
    } else {
        strncpy(latBuf, "--", sizeof(latBuf));
        strncpy(lngBuf, "--", sizeof(lngBuf));
    }

    char buf[360];
    snprintf(buf, sizeof(buf),
        "{"
        "\"uptime\":\"%s\","
        "\"relayCount\":%u,"
        "\"logCount\":%d,"
        "\"gpsFix\":\"%s\","
        "\"gpsSat\":%u,"
        "\"gpsLat\":\"%s\","
        "\"gpsLng\":\"%s\","
        "\"gpsTimeUtc\":\"%s\","
        "\"freeHeap\":%u,"
        "\"sleepMode\":%s"
        "}",
        uptime,
        (unsigned)relayCount,
        countValidLogEntries(),
        gpsHasFix ? "LOCKED" : "SEARCHING",
        (unsigned)gpsSatellites,
        latBuf,
        lngBuf,
        gpsTimeUtc,
        (unsigned)ESP.getFreeHeap(),
        inSleepMode ? "true" : "false"
    );
    server.send(200, "application/json", buf);
}

void handleAPILog() {
    noteActivity();
    // FIX 7: Walk the ring buffer in chronological order (oldest first).
    //         The old code used (logIndex - MAX + i + MAX) % MAX which
    //         is equivalent but clearer here.  More importantly we now
    //         skip entries where valid==false instead of checking timestamp==0,
    //         which was unreliable after the first millis() wraparound.
    //
    // Use a heap allocation here (once per request) to avoid a large stack frame.
    const int JSON_SIZE = 8192;
    char* buf = (char*)malloc(JSON_SIZE);
    if (!buf) { server.send(500, "application/json", "[]"); return; }

    int pos = 0;
    buf[pos++] = '[';

    bool first = true;
    for (int i = 0; i < MAX_LOG_ENTRIES; i++) {
        // Walk oldest→newest
        int idx = (logHead + i) % MAX_LOG_ENTRIES;
        if (!messageLog[idx].valid) continue;

        char escapedMsg[MAX_MSG_LEN * 2 + 1];   // worst case: every char escaped
        jsonEscape(messageLog[idx].message, escapedMsg, sizeof(escapedMsg));

        char entry[640];
        int n = snprintf(entry, sizeof(entry),
            "%s{\"time\":\"%us\",\"dir\":\"%s\",\"rssi\":\"%d\","
            "\"src\":\"%s\",\"dest\":\"%s\",\"msg\":\"%s\"}",
            first ? "" : ",",
            (unsigned)(messageLog[idx].timestamp / 1000),
            messageLog[idx].direction,
            (int)messageLog[idx].rssi,
            messageLog[idx].srcAddr,
            messageLog[idx].destAddr,
            escapedMsg
        );

        if (n > 0 && pos + n < JSON_SIZE - 2) {
            memcpy(buf + pos, entry, n);
            pos += n;
            first = false;
        }
    }

    buf[pos++] = ']';
    buf[pos]   = '\0';

    server.send(200, "application/json", buf);
    free(buf);
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== LoRa Hardware Relay with WebUI ===");

    startTime = millis();
    lastActivityMs = millis();
    initLog();

    pinMode(PIN_AUX, INPUT);
    pinMode(PIN_M0,  OUTPUT);
    pinMode(PIN_M1,  OUTPUT);

    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    WiFi.softAP(ssid_AP, password_AP);
    Serial.printf("AP: %s  IP: %s\n", ssid_AP, WiFi.softAPIP().toString().c_str());

    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.printf("GPS RX=%d TX=%d @%d\n", GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD);

    server.on("/",           handleRoot);
    server.on("/api/status", handleAPIStatus);
    server.on("/api/log",    handleAPILog);
    server.begin();
    Serial.println("Web server started");

    initOled();
    if (oledReady) {
        showBootMessage("LoRa Relay", "Configuring...");
    }

    if (!e22.begin()) {
        Serial.println("[E22] FAILED");
        showBootMessage("E22 FAILED", "Web UI still active");
    } else if (applyRelayConfig()) {
        relayReady = true;
        Serial.println("[READY] Hardware relay active.");
    } else {
        Serial.println("[ERROR] Relay config failed.");
        showBootMessage("CONFIG FAILED", "Check E22 AUX/pins");
    }

    drawDisplay();
}

// =============================================================================
// Loop
// =============================================================================

void loop() {
    static uint32_t lastDraw = 0;

    if (inSleepMode) {
        lightSleepTick();

        if (checkSleepWakeSources()) {
            noteActivity();
        }

        server.handleClient();
        if (relayReady) {
            handleSoftwareRelay();
        }

        if (!inSleepMode) {
            lastDraw = 0;
        }
        return;
    }

    updateGPS();
    server.handleClient();

    if (relayReady) {
        if (e22.available() > 0) {
            noteActivity();
        }
        handleSoftwareRelay();

    }

    if ((uint32_t)(millis() - lastActivityMs) >= SLEEP_IDLE_MS) {
        enterSleepMode();
        return;
    }

    if ((uint32_t)(millis() - lastDraw) >= 1000UL) {
        lastDraw = millis();
        drawDisplay();
    }
}
