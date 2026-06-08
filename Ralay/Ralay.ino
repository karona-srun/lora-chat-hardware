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
#include <Preferences.h>
#include <mbedtls/base64.h>
#include <esp_system.h>
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
#define RELAY_LIGHT_SLEEP_ENABLED 0 // Keep USB serial/debug operation awake by default.
#define USB_SAFE_E22_TX_POWER POWER_22

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
Preferences preferences;

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

// New Pin button
#define BTN_LEFT_PIN    5
#define BTN_RIGHT_PIN   13
#define BTN_UP_PIN      23
#define BTN_DOWN_PIN    32 
#define BTN_SELECT_PIN  14

// OLED pages and persisted display settings
#define NUM_SCREENS 3
uint8_t currentScreen = 0;      // 0=status, 1=GPS, 2=settings
uint8_t oledSleepMode = 3;      // 0=10s, 1=40s, 2=1min, 3=never
uint8_t oledBrightPct = 90;     // 0..100
bool oledDisplayOff = false;
uint32_t lastDisplayActivityMs = 0;
uint8_t settingsField = 0;      // 0=sleep, 1=brightness
bool settingsAdjusting = false;
bool settingsDirty = false;
uint32_t settingsSavedUntilMs = 0;

const uint32_t BTN_DEBOUNCE_MS = 35;
const uint32_t BTN_SETTINGS_HOLD_SAVE_MS = 700;
const uint32_t SETTINGS_SAVED_NOTICE_MS = 1500;

// =============================================================================
// FIX 1: Fixed-size char arrays instead of Arduino String for log entries.
//         Eliminates heap fragmentation from repeated String allocation /
//         deallocation over many hours of runtime.
// =============================================================================
#define MAX_LOG_ENTRIES 20
#define MAX_MSG_LEN      220  // enough for Khmer UTF-8 chat bodies and decoded logs
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
        unsigned char c = (unsigned char)src[si];
        if (c == '"'  && di < dstLen - 2) { dst[di++] = '\\'; dst[di++] = '"';  }
        else if (c == '\\' && di < dstLen - 2) { dst[di++] = '\\'; dst[di++] = '\\'; }
        else if (c == '\n' && di < dstLen - 2) { dst[di++] = '\\'; dst[di++] = 'n';  }
        else if (c == '\r' && di < dstLen - 2) { dst[di++] = '\\'; dst[di++] = 'r';  }
        else if (c == '\t' && di < dstLen - 2) { dst[di++] = '\\'; dst[di++] = 't';  }
        else if (c < 0x20 && di < dstLen - 7) {
            int n = snprintf(dst + di, dstLen - di, "\\u%04X", c);
            if (n <= 0) break;
            di += n;
        }
        else { dst[di++] = (char)c; }
    }
    dst[di] = '\0';
    return di;
}

static size_t utf8CharLen(unsigned char c) {
    if ((c & 0x80) == 0x00) return 1;
    if ((c & 0xE0) == 0xC0) return 2;
    if ((c & 0xF0) == 0xE0) return 3;
    if ((c & 0xF8) == 0xF0) return 4;
    return 1;
}

static void copyUtf8Safe(char* dst, size_t dstLen, const char* src) {
    if (!dst || dstLen == 0) return;
    if (!src) {
        dst[0] = '\0';
        return;
    }

    size_t si = 0;
    size_t di = 0;
    while (src[si] && di < dstLen - 1) {
        size_t count = utf8CharLen((unsigned char)src[si]);
        if (di + count >= dstLen) break;
        bool complete = true;
        for (size_t j = 1; j < count; j++) {
            if (!src[si + j] || (((unsigned char)src[si + j] & 0xC0) != 0x80)) {
                complete = false;
                break;
            }
        }
        if (!complete) break;
        for (size_t j = 0; j < count; j++) dst[di++] = src[si++];
    }
    dst[di] = '\0';
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

static bool decodeRawMsgForLog(const char* frame, char* srcAddr, size_t srcLen,
                               const char* relayDest, char* destAddr, size_t destLen,
                               char* msgOut, size_t msgLen) {
    if (!frame || strncmp(frame, "MSG|", 4) != 0 || !relayDest) return false;

    const char* p1 = strchr(frame, '|');       // after MSG
    if (!p1) return false;
    const char* p2 = strchr(p1 + 1, '|');      // after id
    if (!p2) return false;
    const char* p3 = strchr(p2 + 1, '|');      // after source
    if (!p3) return false;

    size_t srcFieldLen = (size_t)(p3 - (p2 + 1));
    if (srcFieldLen != 4 || srcLen < 5 || destLen < 5 || strlen(relayDest) != 4) return false;

    memcpy(srcAddr, p2 + 1, 4);
    srcAddr[4] = '\0';
    memcpy(destAddr, relayDest, 4);
    destAddr[4] = '\0';
    copyUtf8Safe(msgOut, msgLen, p3 + 1);
    return true;
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
    copyUtf8Safe(e.message, sizeof(e.message), msg);

    logHead = (logHead + 1) % MAX_LOG_ENTRIES;
}

// =============================================================================
// Helpers
// =============================================================================

void drawDisplay();
static void applyOledBrightness();
static void noteDisplayActivity();
static void wakeOledIfNeeded();

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
            applyOledBrightness();
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

static const char* oledSleepLabel(uint8_t mode) {
    switch (mode) {
        case 0: return "10s";
        case 1: return "40s";
        case 2: return "1m";
        default: return "Never";
    }
}

static uint32_t oledSleepTimeoutMs() {
    switch (oledSleepMode) {
        case 0: return 10000UL;
        case 1: return 40000UL;
        case 2: return 60000UL;
        default: return 0;
    }
}

static void loadRelayUiSettings() {
    preferences.begin("relay-ui", true);
    oledSleepMode = preferences.getUChar("oled_sleep", oledSleepMode);
    oledBrightPct = preferences.getUChar("oled_bright", oledBrightPct);
    preferences.end();
    if (oledSleepMode > 3) oledSleepMode = 3;
    if (oledBrightPct > 100) oledBrightPct = 100;
}

static bool saveRelayUiSettings() {
    preferences.begin("relay-ui", false);
    bool saved = preferences.putUChar("oled_sleep", oledSleepMode) == sizeof(oledSleepMode);
    saved = preferences.putUChar("oled_bright", oledBrightPct) == sizeof(oledBrightPct) && saved;
    preferences.end();
    if (saved) {
        settingsDirty = false;
        settingsSavedUntilMs = millis() + SETTINGS_SAVED_NOTICE_MS;
        Serial.println("[OLED] Display settings saved.");
    } else {
        Serial.println("[OLED] Display settings save failed.");
    }
    return saved;
}

static void applyOledBrightness() {
    if (!oledReady) return;
    uint8_t contrast = (uint8_t)((uint32_t)oledBrightPct * 255UL / 100UL);
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(contrast);
}

static void noteDisplayActivity() {
    lastDisplayActivityMs = millis();
}

static void wakeOledIfNeeded() {
    if (!oledDisplayOff || !oledReady) return;
    display.ssd1306_command(SSD1306_DISPLAYON);
    oledDisplayOff = false;
    applyOledBrightness();
    drawDisplay();
}

static void checkOledSleepTimeout() {
    if (!oledReady || oledDisplayOff || settingsAdjusting) return;
    uint32_t timeoutMs = oledSleepTimeoutMs();
    if (timeoutMs != 0 && (uint32_t)(millis() - lastDisplayActivityMs) >= timeoutMs) {
        display.ssd1306_command(SSD1306_DISPLAYOFF);
        oledDisplayOff = true;
    }
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


// ────────────────────────────────────────────────
// Draw header with rounded corners background (radius ~2px)
// ────────────────────────────────────────────────
void drawHeader() {
  // Draw filled rectangle with rounded corners (radius 1px)
  // Clear full top band to avoid old pixels after long uptime
  display.fillRect(0, 0, SCREEN_WIDTH, 13, SSD1306_BLACK);
  // Then filled rounded area across full width
  display.fillRoundRect(0, 0, SCREEN_WIDTH, 13, 3, SSD1306_WHITE);
  // Optional: white outline on top (makes it look sharper)
  display.drawRoundRect(0, 0, SCREEN_WIDTH, 13, 3, SSD1306_WHITE);
  
  // Draw text in inverse (black on white)
  display.setTextSize(1);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);

  // Relay identifies its access point in the header instead of showing battery state.
  display.setCursor(6, 3);
  display.print(ssid_AP);

  // GPS time on the right; displays placeholders until GPS time is valid.
  display.setCursor(76, 3);
  display.print(gpsTimeUtc);
  
  // Reset text color for rest of screen
  display.setTextColor(SSD1306_WHITE);
}

// =============================================================================
// OLED — uses fixed-size stack buffers, no String
// =============================================================================

static void drawScreenDots() {
    const int spacing = 10;
    const int startX = (SCREEN_WIDTH - (NUM_SCREENS - 1) * spacing) / 2;
    for (uint8_t i = 0; i < NUM_SCREENS; i++) {
        int x = startX + i * spacing;
        if (i == currentScreen) {
            display.fillCircle(x, 61, 2, SSD1306_WHITE);
        } else {
            display.drawCircle(x, 61, 1, SSD1306_WHITE);
        }
    }
}

static void drawStatusScreen() {
    display.clearDisplay();
    drawHeader();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    char buf[32];

    display.setCursor(0, 16);
    snprintf(buf, sizeof(buf), "IP: %s", WiFi.softAPIP().toString().c_str());
    display.print(buf);

    display.setCursor(0, 26);
    display.print("Addr: 0xFF/0xFF");

    display.setCursor(0, 36);
    snprintf(buf, sizeof(buf), "CH:0x%02X  NET:0x%02X", COMMON_CHAN, COMMON_NETID);
    display.print(buf);

    display.setCursor(0, 46);
    snprintf(buf, sizeof(buf), "Relayed: %u", (unsigned)relayCount);
    display.print(buf);

    drawScreenDots();
    display.display();
}

static void drawGpsScreen() {
    display.clearDisplay();
    drawHeader();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);

    char buf[32];
    display.setCursor(0, 16);
    snprintf(buf, sizeof(buf), "GPS: %s  SAT:%u",
             gpsHasFix ? "FIX" : "SEARCH", (unsigned)gpsSatellites);
    display.print(buf);

    display.setCursor(0, 27);
    if (gpsHasFix) {
        snprintf(buf, sizeof(buf), "Lat: %.5f", gpsLat);
        display.print(buf);
        display.setCursor(0, 38);
        snprintf(buf, sizeof(buf), "Lng: %.5f", gpsLng);
        display.print(buf);
    } else {
        display.print("Waiting location...");
        display.setCursor(0, 38);
        display.print("Move outdoors for fix");
    }

    display.setCursor(0, 49);
    snprintf(buf, sizeof(buf), "UTC: %s", gpsTimeUtc);
    display.print(buf);
    drawScreenDots();
    display.display();
}

static void drawSettingsScreen() {
    display.clearDisplay();
    drawHeader();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(4, 16);
    display.print("Display");
    if (settingsDirty) display.fillCircle(116, 19, 2, SSD1306_WHITE);

    const int rowX = 6;
    const int rowW = SCREEN_WIDTH - 12;
    const int rowH = 12;
    const int row1Y = 27;
    const int row2Y = 40;
    char value[10];

    snprintf(value, sizeof(value), "%s", oledSleepLabel(oledSleepMode));
    if (settingsAdjusting && settingsField == 0) {
        display.fillRoundRect(rowX, row1Y, rowW, rowH, 3, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
        display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(rowX + 5, row1Y + 2);
    display.print(settingsField == 0 ? "* " : " ");
    display.print("Sleep");
    display.setCursor(84, row1Y + 2);
    display.print(value);

    snprintf(value, sizeof(value), "%u%%", (unsigned)oledBrightPct);
    if (settingsAdjusting && settingsField == 1) {
        display.fillRoundRect(rowX, row2Y, rowW, rowH, 3, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
        display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(rowX + 5, row2Y + 2);
    display.print(settingsField == 1 ? "* " : " ");
    display.print("Brightness");
    display.setCursor(92, row2Y + 2);
    display.print(value);

    display.setTextColor(SSD1306_WHITE);
    drawScreenDots();
    if ((int32_t)(settingsSavedUntilMs - millis()) > 0) {
        const char* text = "SAVED !";
        const int x = 31;
        const int y = 27;
        display.fillRoundRect(x, y, 66, 25, 4, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
        display.setCursor(x + 12, y + 9);
        display.print(text);
    }
    display.display();
}

void drawDisplay() {
    if (!oledReady || inSleepMode || oledDisplayOff) return;
    if (currentScreen == 0) {
        drawStatusScreen();
    } else if (currentScreen == 1) {
        drawGpsScreen();
    } else {
        drawSettingsScreen();
    }
}

static void changeSettingValue(int direction) {
    if (settingsField == 0) {
        int mode = (int)oledSleepMode + direction;
        if (mode < 0) mode = 3;
        if (mode > 3) mode = 0;
        oledSleepMode = (uint8_t)mode;
    } else {
        int brightness = (int)oledBrightPct + direction * 5;
        oledBrightPct = (uint8_t)constrain(brightness, 0, 100);
        applyOledBrightness();
    }
    settingsDirty = true;
}

static void processDisplayButtons() {
    static bool previousLeft = false;
    static bool previousRight = false;
    static bool previousUp = false;
    static bool previousDown = false;
    static bool previousSelect = false;
    static bool waitForRelease = false;
    static bool selectLongDone = false;
    static uint32_t selectStartedAt = 0;
    static uint32_t lastPressAt = 0;

    bool left = digitalRead(BTN_LEFT_PIN) == LOW;
    bool right = digitalRead(BTN_RIGHT_PIN) == LOW;
    bool up = digitalRead(BTN_UP_PIN) == LOW;
    bool down = digitalRead(BTN_DOWN_PIN) == LOW;
    bool select = digitalRead(BTN_SELECT_PIN) == LOW;
    bool anyPressed = left || right || up || down || select;
    uint32_t now = millis();

    if (oledDisplayOff && anyPressed) {
        wakeOledIfNeeded();
        noteDisplayActivity();
        waitForRelease = true;
    }
    if (waitForRelease) {
        if (!anyPressed) waitForRelease = false;
        previousLeft = left;
        previousRight = right;
        previousUp = up;
        previousDown = down;
        previousSelect = select;
        return;
    }

    bool canPress = (uint32_t)(now - lastPressAt) >= BTN_DEBOUNCE_MS;
    if (canPress && left && !previousLeft) {
        lastPressAt = now;
        noteDisplayActivity();
        lastActivityMs = now;
        if (currentScreen == 2 && settingsAdjusting) {
            changeSettingValue(-1);
        } else {
            if (currentScreen == 2 && settingsDirty) saveRelayUiSettings();
            currentScreen = (currentScreen + NUM_SCREENS - 1) % NUM_SCREENS;
            settingsAdjusting = false;
        }
        drawDisplay();
    }
    if (canPress && right && !previousRight) {
        lastPressAt = now;
        noteDisplayActivity();
        lastActivityMs = now;
        if (currentScreen == 2 && settingsAdjusting) {
            changeSettingValue(1);
        } else {
            if (currentScreen == 2 && settingsDirty) saveRelayUiSettings();
            currentScreen = (currentScreen + 1) % NUM_SCREENS;
            settingsAdjusting = false;
        }
        drawDisplay();
    }
    if (canPress && up && !previousUp && currentScreen == 2 && !settingsAdjusting) {
        lastPressAt = now;
        noteDisplayActivity();
        lastActivityMs = now;
        settingsField = settingsField == 0 ? 1 : 0;
        drawDisplay();
    }
    if (canPress && down && !previousDown && currentScreen == 2 && !settingsAdjusting) {
        lastPressAt = now;
        noteDisplayActivity();
        lastActivityMs = now;
        settingsField = settingsField == 0 ? 1 : 0;
        drawDisplay();
    }

    if (select && !previousSelect) {
        selectStartedAt = now;
        selectLongDone = false;
        noteDisplayActivity();
        lastActivityMs = now;
    } else if (select && !selectLongDone && currentScreen == 2 &&
               (uint32_t)(now - selectStartedAt) >= BTN_SETTINGS_HOLD_SAVE_MS) {
        selectLongDone = true;
        settingsAdjusting = false;
        saveRelayUiSettings();
        noteDisplayActivity();
        drawDisplay();
    } else if (!select && previousSelect && !selectLongDone &&
               (uint32_t)(now - selectStartedAt) >= BTN_DEBOUNCE_MS) {
        noteDisplayActivity();
        lastActivityMs = now;
        if (currentScreen == 2) {
            if (settingsAdjusting) {
                settingsAdjusting = false;
                if (settingsDirty) saveRelayUiSettings();
            } else {
                settingsAdjusting = true;
            }
        } else {
            currentScreen = 2;
            settingsAdjusting = false;
        }
        drawDisplay();
    }

    previousLeft = left;
    previousRight = right;
    previousUp = up;
    previousDown = down;
    previousSelect = select;
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

    // The ESP32 handles forwarding in handleSoftwareRelay(); leaving the
    // module repeater enabled can cause a second, uncontrolled RF transmit.
    bool changed =
        config.ADDH != REPEATER_ADDH ||
        config.ADDL != REPEATER_ADDL ||
        config.NETID != COMMON_NETID ||
        config.CHAN != COMMON_CHAN ||
        config.CRYPT.CRYPT_H != highByte(CRYPT_KEY) ||
        config.CRYPT.CRYPT_L != lowByte(CRYPT_KEY) ||
        (uint8_t)config.SPED.uartBaudRate != UART_BPS_9600 ||
        (uint8_t)config.SPED.airDataRate != AIR_DATA_RATE_010_24 ||
        (uint8_t)config.SPED.uartParity != MODE_00_8N1 ||
        (uint8_t)config.OPTION.subPacketSetting != SPS_240_00 ||
        (uint8_t)config.OPTION.RSSIAmbientNoise != RSSI_AMBIENT_NOISE_DISABLED ||
        (uint8_t)config.OPTION.transmissionPower != USB_SAFE_E22_TX_POWER ||
        (uint8_t)config.TRANSMISSION_MODE.enableRSSI != RSSI_ENABLED ||
        (uint8_t)config.TRANSMISSION_MODE.fixedTransmission != FT_FIXED_TRANSMISSION ||
        (uint8_t)config.TRANSMISSION_MODE.enableRepeater != REPEATER_DISABLED ||
        (uint8_t)config.TRANSMISSION_MODE.enableLBT != LBT_DISABLED ||
        (uint8_t)config.TRANSMISSION_MODE.WORTransceiverControl != WOR_RECEIVER ||
        (uint8_t)config.TRANSMISSION_MODE.WORPeriod != WOR_2000_011;

    if (!changed) {
        Serial.println("[CONFIG] E22 already configured; skipped flash write");
        return true;
    }

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
    config.OPTION.transmissionPower = USB_SAFE_E22_TX_POWER;

    config.TRANSMISSION_MODE.enableRSSI            = RSSI_ENABLED;
    config.TRANSMISSION_MODE.fixedTransmission     = FT_FIXED_TRANSMISSION;
    config.TRANSMISSION_MODE.enableRepeater        = REPEATER_DISABLED;
    config.TRANSMISSION_MODE.enableLBT             = LBT_DISABLED;
    config.TRANSMISSION_MODE.WORTransceiverControl = WOR_RECEIVER;
    config.TRANSMISSION_MODE.WORPeriod             = WOR_2000_011;

    ResponseStatus rs = e22.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
    if (rs.code != 1) return false;

    delay(300);
    if (!waitAuxHigh(5000)) return false;
    delay(100);
    Serial.println("[CONFIG] E22 software-relay mode, TX power ~33 dBm");
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

    // Expected format: "RELAY|XXYY|<payload>"
    if (strncmp(rxBuf, "RELAY|", 6) != 0) {
        char logSrc[MAX_ADDR_LEN] = "????";
        char logDest[MAX_ADDR_LEN] = "FFFF";
        char decodedMsg[MAX_MSG_LEN];
        const char* logMsg = rxBuf;
        if (decodeMsg3ForLog(rxBuf, logSrc, sizeof(logSrc), logDest, sizeof(logDest),
                             decodedMsg, sizeof(decodedMsg))) {
            logMsg = decodedMsg;
        }
        addLogEntry("RX", rssi, logSrc, logDest, logMsg);
        return;
    }

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
    } else if (decodeRawMsgForLog(payload, relaySrc, sizeof(relaySrc), destHex,
                                  destHex, sizeof(destHex), relayDecodedMsg,
                                  sizeof(relayDecodedMsg))) {
        relayLogMsg = relayDecodedMsg;
    }

    ResponseStatus rs = e22.sendFixedMessage(destAddh, destAddl, COMMON_CHAN, payload);
    if (rs.code == 1) {
        relayCount++;
        noteActivity();
        addLogEntry("RELAY", rssi, relaySrc, destHex, relayLogMsg);
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
    server.send(200, "text/html; charset=utf-8", page);
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
    server.send(200, "application/json; charset=utf-8", buf);
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
    if (!buf) { server.send(500, "application/json; charset=utf-8", "[]"); return; }

    int pos = 0;
    buf[pos++] = '[';

    bool first = true;
    for (int i = 0; i < MAX_LOG_ENTRIES; i++) {
        // Walk oldest→newest
        int idx = (logHead + i) % MAX_LOG_ENTRIES;
        if (!messageLog[idx].valid) continue;

        char escapedMsg[MAX_MSG_LEN * 6 + 1];   // worst case: every byte JSON-escaped
        jsonEscape(messageLog[idx].message, escapedMsg, sizeof(escapedMsg));

        char entry[1600];
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

    server.send(200, "application/json; charset=utf-8", buf);
    free(buf);
}

// =============================================================================
// Setup
// =============================================================================

static const char* resetReasonText(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXTERNAL_RESET";
        case ESP_RST_SW:        return "SOFTWARE_RESET";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INTERRUPT_WATCHDOG";
        case ESP_RST_TASK_WDT:  return "TASK_WATCHDOG";
        case ESP_RST_WDT:       return "OTHER_WATCHDOG";
        case ESP_RST_DEEPSLEEP: return "DEEP_SLEEP_WAKE";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}


// Full splash adaptation for a 128x64 monochrome OLED.
static const uint8_t LOMHOR_SPLASH_BITMAP[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x80, 0xf8, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x7f, 0x3c, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0xff, 0xc8, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x00, 0x03, 0x80, 0xe0, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xb8, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xb8, 0x00, 0x00, 0x00, 0xe3, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf1, 0xc3, 0x81, 0xfc, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0xc7, 0xc7, 0xfe, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xcf, 0xef, 0xfe, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0x03, 0xce, 0x6f, 0xfe, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xc3, 0xce, 0xef, 0xfe, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xe3, 0xc7, 0xcf, 0xbe, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xf3, 0xc7, 0xcf, 0x3e, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xf3, 0xc7, 0xcf, 0x3e, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x7d, 0xf3, 0xc7, 0xcf, 0x3e, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x79, 0xf3, 0xc7, 0xcf, 0x3e, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x79, 0xf3, 0xc7, 0xcf, 0x3e, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x79, 0xf3, 0xc7, 0xcf, 0x3e, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x79, 0xfb, 0xc7, 0xcf, 0x3e, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0xff, 0xc7, 0xdf, 0x3e, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0xc7, 0xff, 0x3e, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xce, 0xff, 0xc7, 0xff, 0x3e, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x7f, 0xc7, 0xff, 0x3e, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x3f, 0xc7, 0xfe, 0x3e, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x07, 0xf8, 0x1c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const int LOMHOR_SPLASH_WIDTH = 128;
static const int LOMHOR_SPLASH_HEIGHT = 64;

static void showSplashScreen() {
    if (!oledReady) return;

    const unsigned long splashDurationMs = 2000UL;
    const char *version = "v0.0.12";
    const int barX = 0;
    const int barY = SCREEN_HEIGHT - 4;
    const int barWidth = SCREEN_WIDTH;
    const int barHeight = 4;
    const int barRadius = 3;
    int16_t textX, textY;
    uint16_t textWidth, textHeight;

    display.clearDisplay();
    display.drawBitmap((SCREEN_WIDTH - LOMHOR_SPLASH_WIDTH),
                       (SCREEN_HEIGHT - LOMHOR_SPLASH_HEIGHT),
                       LOMHOR_SPLASH_BITMAP, LOMHOR_SPLASH_WIDTH,
                       LOMHOR_SPLASH_HEIGHT, SSD1306_WHITE);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    display.getTextBounds(version, 0, 0, &textX, &textY, &textWidth, &textHeight);
    display.setCursor((int)SCREEN_WIDTH - (int)textWidth, 2);
    display.print(version);
    display.drawRoundRect(barX, barY, barWidth, barHeight, barRadius, SSD1306_WHITE);

    unsigned long startedAt = millis();
    unsigned long elapsed = 0;
    while (elapsed < splashDurationMs) {
        elapsed = millis() - startedAt;
        int fillWidth = (int)((unsigned long)(barWidth - 2) *
                              min(elapsed, splashDurationMs) / splashDurationMs);
        if (fillWidth > 0) {
            display.fillRoundRect(barX + 1, barY + 1, fillWidth, barHeight - 2,
                                  barRadius, SSD1306_WHITE);
        }
        display.display();
        delay(40);
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n=== LoRa Hardware Relay with WebUI ===");
    Serial.printf("[BOOT] Reset reason: %s (%d)\n",
                  resetReasonText(esp_reset_reason()), (int)esp_reset_reason());

    startTime = millis();
    lastActivityMs = millis();
    lastDisplayActivityMs = millis();
    initLog();
    loadRelayUiSettings();

    pinMode(PIN_AUX, INPUT);
    pinMode(PIN_M0,  OUTPUT);
    pinMode(PIN_M1,  OUTPUT);
    pinMode(BTN_LEFT_PIN, INPUT_PULLUP);
    pinMode(BTN_RIGHT_PIN, INPUT_PULLUP);
    pinMode(BTN_UP_PIN, INPUT_PULLUP);
    pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
    pinMode(BTN_SELECT_PIN, INPUT_PULLUP);

    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
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
    display.setRotation(0);
    showSplashScreen();
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
    processDisplayButtons();
    checkOledSleepTimeout();

    if (relayReady) {
        if (e22.available() > 0) {
            noteActivity();
        }
        handleSoftwareRelay();

    }

    if (RELAY_LIGHT_SLEEP_ENABLED && (uint32_t)(millis() - lastActivityMs) >= SLEEP_IDLE_MS) {
        enterSleepMode();
        return;
    }

    if ((uint32_t)(millis() - lastDraw) >= 1000UL) {
        lastDraw = millis();
        drawDisplay();
    }

    if (settingsSavedUntilMs != 0 && (int32_t)(millis() - settingsSavedUntilMs) >= 0) {
        settingsSavedUntilMs = 0;
        if (currentScreen == 2) drawDisplay();
    }
}
