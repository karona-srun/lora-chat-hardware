// -------------------------------------------------------------------------------
// LoRa Node with Dynamic Configuration - E22-900T33S / ESP32
// Configure Node ID, Network ID, and Repeater settings via WebUI
// -------------------------------------------------------------------------------

#include <Arduino.h>
#include <LoRa_E22.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <mbedtls/base64.h>
#include <math.h>
#include <esp_system.h>

// ----------------------------- Dynamic Configuration -------------------------
Preferences preferences;

uint8_t MY_ADDH = 0x00;
uint8_t MY_ADDL = 0x01;     // Default Node ID
uint8_t TARGET_ADDH = 0x00;
uint8_t TARGET_ADDL = 0x00; // Default Target
uint8_t REPEATER_ADDH = 0xFF;
uint8_t REPEATER_ADDL = 0xFF;
uint8_t NETWORK_ID = 0x01;  // Default Network ID
uint8_t CHANNEL = 0x41;     // Default Channel 915Mhz
uint16_t CRYPT = 0x8002;    // KEY 32770
bool USE_REPEATER = false;  // Repeater mode
// E22 TX power: 0=POWER_22, 1=POWER_17, 2=POWER_13, 3=POWER_10 (see LoRa_E22 Option.transmissionPower)
uint8_t E22_TX_POWER = 0;

// ----------------------------- WiFi credentials -------------------------------
String ssid_AP = "LM-" + String(MY_ADDL);
String password_AP = "12345678";
String callSign = ssid_AP;

String defaultApSsid() {
    return "LM-" + String(MY_ADDL);
}

#define BUZZER_ACTIVE 15 // Buzzer for alert sound

#define BAT_READER    35  // ADC pin for battery voltage  // 35 New ADC pin for battery voltage
#define BAT_CHARGING  34  // HIGH when charging (input only)
#define BAT_CHARGING_ACTIVE_LEVEL HIGH // Match this board's charge-detect circuit.

// Battery ADC. GPIO35 uses the battery divider in normal/unplugged mode.
// Charging can lift the ADC-side voltage above the real 1-cell battery voltage,
// so the converted value is capped at BAT_VOLTAGE_MAX before display/percentage.
// If your board uses a different divider, tune only BAT_DIVIDER_RATIO.
#define BAT_ADC_MAX        4095.0f
#define BAT_ADC_VREF_MV    3300.0f
#define BAT_DIVIDER_RATIO  2.0f
#define BAT_VOLTAGE_MAX    4.20f
#define BAT_ADC_SAMPLES    16

// Five-way control: Left/Right change screen; Up/Down adjust values in settings edit;
// Select opens Display settings from other screens; on Display screen, Select toggles edit.
// All pressed = LOW.

// Old
// #define BTN_LEFT_PIN    23
// #define BTN_RIGHT_PIN   32
// #define BTN_UP_PIN      5 
// #define BTN_DOWN_PIN    14
// #define BTN_SELECT_PIN  13

// New Pin
#define BTN_LEFT_PIN    32
#define BTN_RIGHT_PIN   23
#define BTN_UP_PIN      5 
#define BTN_DOWN_PIN    13
#define BTN_SELECT_PIN  14

// ────────────────────────────────────────────────
// GPS on Serial1
// ────────────────────────────────────────────────
// #define GPS_TX_PIN  25 //Old pins GPS old board (Black Box)
// #define GPS_RX_PIN  26
#define GPS_TX_PIN  26  // New pins GPS new board (Green Box)
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

unsigned long startTime = 0;
int onlineNodes = 0;
float batteryVoltage   = 3.79f;
int batteryPercent     = 100;
bool batteryCharging   = false;
bool lastBatteryCharging = false;
bool brownoutRecoveryMode = false;

int currentScreen = 0;
unsigned long lastRightBtnPress = 0;
unsigned long lastLeftBtnPress = 0;
unsigned long lastUpBtnPress = 0;
unsigned long lastDownBtnPress = 0;
unsigned long lastSelectBtnPress = 0;
bool rightBtnHeld = false;
bool rightBtnLongSent = false;
unsigned long rightBtnDownMs = 0;
bool chargeScreenSuppressed = false;
static bool chargeScreenUiReady = false;  // true after full charging screen paint (partial updates after)

enum class PopupType : uint8_t { None, Sent, Received };
PopupType popupType = PopupType::None;
String popupMsg = "";
String popupStatus = "";
unsigned long popupUntilMs = 0;

const bool AUTO_GPS_ENABLED = true;
const unsigned long AUTO_GPS_INTERVAL_MS = 5000UL;
const unsigned long BTN_RIGHT_LONG_PRESS_MS = 3000UL;
// Display screen: hold SELECT this long to explicitly write settings to flash
const unsigned long BTN_SETTINGS_HOLD_SAVE_MS = 700UL;
const unsigned long SETTINGS_SAVED_NOTICE_MS = 1500UL;
const unsigned long CHARGE_PIN_DEBOUNCE_MS = 400UL;   // pin HIGH this long → enter charging UI
const unsigned long CHARGE_PIN_RELEASE_MS = 2500UL; // pin LOW this long → leave charging UI (stops flicker)
const unsigned long UI_REFRESH_NORMAL_MS = 5000UL;
// Charging animation tick (partial OLED updates — no full clear every frame)
const unsigned long UI_REFRESH_CHARGING_MS = 500UL;
const uint8_t STABLE_E22_TX_POWER = 3; // POWER_10: lowest-current E22 transmit mode.
const bool PERIODIC_HELLO_ENABLED = false; // Avoid unattended RF current bursts.

#define NUM_SCREENS 4

// OLED display settings (persisted)
uint8_t oledSleepMode = 3;   // 0=10s, 1=40s, 2=1min, 3=never
uint8_t oledBrightPct = 90;  // 0–100
bool oledDisplayOff = false;
unsigned long lastDisplayActivityMs = 0;
uint8_t settingsEditField = 0;   // focused row: 0=sleep, 1=brightness, 2=alert beep (U/D cycles)
bool settingsAdjusting = false;  // true: L/R change values (preview, not flash)
bool settingsDirty = false;      // true: preview differs from last save
bool alertsBeepEnabled = true;   // buzzer on new MSG2/MSG (saved with OLED settings)
unsigned long settingsSavedUntilMs = 0;
bool oledIgnoreButtonsUntilRelease = false;

// ----------------------------- E22 pins --------------------------------------
#define PIN_E22_TX   4
#define PIN_E22_RX  22
#define PIN_AUX     18
#define PIN_M0     21
#define PIN_M1     19

LoRa_E22 e22(PIN_E22_TX, PIN_E22_RX, &Serial2, PIN_AUX, PIN_M0, PIN_M1, UART_BPS_RATE_9600, SERIAL_8N1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool oledReady = false;
uint8_t oledI2cAddress = SCREEN_ADDRESS;
WebServer server(80);

bool nodeReady = false;
String statusMsg = "Booting";
uint32_t sentCount = 0;
uint32_t receivedCount = 0;
String lastSent = "";
String lastReceived = "";
int16_t lastRssiSignal = -255;
uint16_t txMessageCounter = 0;
String awaitingAckId = "";
bool awaitingAckReceived = false;

const unsigned long ACK_WAIT_MS = 1200UL;
const uint8_t ACK_RETRY_MAX = 2;
const size_t MAX_CHAT_BODY_UTF8_BYTES = 150; // Safe for Khmer UTF-8 through direct or relay frames.

// Nearby nodes registry (learned from HELLO beacons)
#define MAX_NEARBY_NODES 20
struct NearbyNode {
    uint16_t addr;
    uint8_t netId;
    uint8_t channel;
    int16_t rssi;
    unsigned long lastSeen;
    bool isRepeater;
    String callSign;
    bool hasGpsFix;
    double lat;
    double lng;
};
NearbyNode nearbyNodes[MAX_NEARBY_NODES];
uint8_t nearbyCount = 0;
unsigned long lastBeaconAt = 0;

// Same rule as /api/nodes "online" field: seen a HELLO within this window (beacons ~5s).
#define ONLINE_NODE_TTL_MS 10000UL

static bool isNearbyNodeOnlineAt(uint8_t i, unsigned long now) {
    if (i >= nearbyCount) return false;
    return (now - nearbyNodes[i].lastSeen) <= ONLINE_NODE_TTL_MS;
}

static uint8_t countOnlineNearbyNodes(unsigned long now) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < nearbyCount; i++) {
        if (isNearbyNodeOnlineAt(i, now)) n++;
    }
    return n;
}

// Keep OLED "online" count in sync with /api/nodes filtering logic
void syncOnlineNodesFromRegistry() {
    onlineNodes = (int)countOnlineNearbyNodes(millis());
}

static int16_t e22RssiByteToDbm(uint8_t rawRssi) {
    return (int16_t)rawRssi - 256;
}

static int rssiDbmToSignalPercent(int16_t rssiDbm) {
    if (rssiDbm <= -250) return 0;
    const int16_t minUsableDbm = -140;
    const int16_t maxStrongDbm = -40;
    int pct = ((int32_t)(rssiDbm - minUsableDbm) * 100) / (maxStrongDbm - minUsableDbm);
    return constrain(pct, 0, 100);
}

// Message log
#define MAX_LOG_ENTRIES 20
struct LogEntry {
    unsigned long timestamp;
    String direction;
    int16_t rssi;
    String address;
    String message;
};
LogEntry messageLog[MAX_LOG_ENTRIES];
int logIndex = 0;

void addLogEntry(String direction, int16_t rssi, String addr, String msg) {
    messageLog[logIndex].timestamp = millis();
    messageLog[logIndex].direction = direction;
    messageLog[logIndex].rssi = rssi;
    messageLog[logIndex].address = addr;
    messageLog[logIndex].message = msg;
    logIndex = (logIndex + 1) % MAX_LOG_ENTRIES;
}

// =============================================================================
// Preferences - Load/Save Configuration
// =============================================================================

void loadConfig() {
    preferences.begin("lora-config", false);
    
    MY_ADDL = preferences.getUChar("my_addl", MY_ADDL);
    MY_ADDH = preferences.getUChar("my_addh", MY_ADDH);
    TARGET_ADDL = preferences.getUChar("target_addl", TARGET_ADDL);
    TARGET_ADDH = preferences.getUChar("target_addh", TARGET_ADDH);
    REPEATER_ADDL = preferences.getUChar("rep_addl", REPEATER_ADDL);
    REPEATER_ADDH = preferences.getUChar("rep_addh", REPEATER_ADDH);
    NETWORK_ID = preferences.getUChar("netid", NETWORK_ID);
    CHANNEL = preferences.getUChar("channel", CHANNEL);
    USE_REPEATER = preferences.getBool("use_repeater", false);
    if (preferences.isKey("crypt")) {
        CRYPT = preferences.getUShort("crypt", CRYPT);
    } else if (preferences.isKey("crypt_h") || preferences.isKey("crypt_l")) {
        uint8_t h = (uint8_t)preferences.getUShort("crypt_h", highByte(CRYPT));
        uint8_t l = (uint8_t)preferences.getUShort("crypt_l", lowByte(CRYPT));
        CRYPT = ((uint16_t)h << 8) | l;
    }
    ssid_AP = preferences.getString("ssid_ap", defaultApSsid());
    if (ssid_AP.length() == 0) ssid_AP = defaultApSsid();
    E22_TX_POWER = preferences.getUChar("tx_power", E22_TX_POWER);
    if (E22_TX_POWER > 3) E22_TX_POWER = 0;
    oledSleepMode = preferences.getUChar("oled_sleep", oledSleepMode);
    if (oledSleepMode > 3) oledSleepMode = 3;
    oledBrightPct = preferences.getUChar("oled_bright", oledBrightPct);
    if (oledBrightPct > 100) oledBrightPct = 100;
    alertsBeepEnabled = preferences.getBool("alerts_beep", true);
    
    preferences.end();

    settingsDirty = false;
    settingsAdjusting = false;
    
    Serial.println("\n=== Loaded Configuration ===");
    Serial.printf("My Address: 0x%02X%02X\n", MY_ADDH, MY_ADDL);
    Serial.printf("Target: 0x%02X%02X\n", TARGET_ADDH, TARGET_ADDL);
    Serial.printf("Repeater: 0x%02X%02X\n", REPEATER_ADDH, REPEATER_ADDL);
    Serial.printf("Network ID: 0x%02X\n", NETWORK_ID);
    Serial.printf("Channel: 0x%02X\n", CHANNEL);
    Serial.printf("Use Repeater: %s\n", USE_REPEATER ? "Yes" : "No");
    Serial.printf("CRYPT: 0x%04X (must match other nodes; 0x0000 = off)\n", CRYPT);
    Serial.printf("E22 TX power index: %u (0=max ~22dBm .. 3=min ~10dBm)\n", (unsigned)E22_TX_POWER);
    Serial.println("============================\n");
}

void saveConfig() {
    preferences.begin("lora-config", false);
    
    preferences.putUChar("my_addl", MY_ADDL);
    preferences.putUChar("my_addh", MY_ADDH);
    preferences.putUChar("target_addl", TARGET_ADDL);
    preferences.putUChar("target_addh", TARGET_ADDH);
    preferences.putUChar("rep_addl", REPEATER_ADDL);
    preferences.putUChar("rep_addh", REPEATER_ADDH);
    preferences.putUChar("netid", NETWORK_ID);
    preferences.putUChar("channel", CHANNEL);
    preferences.putBool("use_repeater", USE_REPEATER);
    preferences.putUShort("crypt", CRYPT);
    preferences.putUChar("tx_power", E22_TX_POWER);
    preferences.putUChar("oled_sleep", oledSleepMode);
    preferences.putUChar("oled_bright", oledBrightPct);
    preferences.putBool("alerts_beep", alertsBeepEnabled);
    
    preferences.end();
    
    Serial.println("Configuration saved!");
}

bool saveOledSettings() {
    preferences.begin("lora-config", false);
    bool saved = preferences.putUChar("oled_sleep", oledSleepMode) == sizeof(oledSleepMode);
    saved = preferences.putUChar("oled_bright", oledBrightPct) == sizeof(oledBrightPct) && saved;
    saved = preferences.putBool("alerts_beep", alertsBeepEnabled) == sizeof(alertsBeepEnabled) && saved;
    preferences.end();
    if (saved) {
        settingsDirty = false;
        settingsSavedUntilMs = millis() + SETTINGS_SAVED_NOTICE_MS;
        Serial.println("Display settings saved to flash.");
    } else {
        Serial.println("Failed to save display settings to flash.");
    }
    return saved;
}

static void commitOledSettingsIfDirty() {
    if (settingsDirty) {
        saveOledSettings();
    }
}

void saveApConfig() {
    preferences.begin("lora-config", false);
    preferences.putString("ssid_ap", ssid_AP);
    preferences.end();
    Serial.printf("AP SSID saved to flash: %s\n", ssid_AP.c_str());
}

static unsigned long oledSleepTimeoutMs() {
    switch (oledSleepMode) {
        case 0: return 10000UL;
        case 1: return 40000UL;
        case 2: return 60000UL;
        default: return 0;
    }
}

static const char *oledSleepLabel(uint8_t mode) {
    switch (mode) {
        case 0: return "10s";
        case 1: return "40s";
        case 2: return "1m";
        default: return "Never";
    }
}

void applyOledBrightness() {
    if (!oledReady) return;
    uint8_t c = (uint8_t)((unsigned long)oledBrightPct * 255UL / 100UL);
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(c);
}

static void oledHardwarePower(bool on) {
    if (!oledReady) return;
    display.ssd1306_command((uint8_t)(on ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF));
}

static bool i2cAddressResponds(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

static void showBootMessage(const char *line1, const char *line2) {
    if (!oledReady) return;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(line1);
    display.println(line2);
    display.display();
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

static bool initOled() {
    Wire.begin(OLED_SDA, OLED_SCL);
    Wire.setClock(100000);
    delay(50);

    // Many 0.96 inch SSD1306 modules are 0x3C, but some are 0x3D. Try both.
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
            display.setRotation(0);
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

void noteDisplayActivity() {
    lastDisplayActivityMs = millis();
}

void tryWakeOledFromSleep() {
    if (!oledDisplayOff) return;
    oledHardwarePower(true);
    oledDisplayOff = false;
    applyOledBrightness();
    drawCurrentScreen();
}

void checkOledSleepTimeout() {
    if (oledDisplayOff) return;
    if (batteryCharging && !chargeScreenSuppressed) return;
    if (settingsAdjusting) return;
    if (currentScreen == 3 && settingsDirty) return;
    unsigned long to = oledSleepTimeoutMs();
    if (to == 0) return;
    if ((millis() - lastDisplayActivityMs) >= to) {
        oledHardwarePower(false);
        oledDisplayOff = true;
    }
}

// Preview in RAM while adjusting; commit once editing ends or the screen is left.
static void previewSettingsAdjust(int dir) {
    if (settingsEditField == 0) {
        int v = (int)oledSleepMode + dir;
        while (v < 0) v += 4;
        oledSleepMode = (uint8_t)(v % 4);
    } else if (settingsEditField == 1) {
        int b = (int)oledBrightPct + dir * 5;
        oledBrightPct = (uint8_t)constrain(b, 0, 100);
        applyOledBrightness();
    } else {
        // Row "Alerts": Left = OFF, Right = ON
        alertsBeepEnabled = (dir > 0);
    }
    settingsDirty = true;
}

// =============================================================================
// Helpers
// =============================================================================

String toHex2(uint8_t v) {
    char b[3]; 
    snprintf(b, sizeof(b), "%02X", v); 
    return String(b);
}

String toHex4(uint16_t addr) {
    char b[5];
    snprintf(b, sizeof(b), "%04X", addr);
    return String(b);
}

static String radioEscapeField(const String &s) {
    String out;
    out.reserve(s.length() * 3);
    const char *hex = "0123456789ABCDEF";
    for (size_t i = 0; i < s.length(); i++) {
        unsigned char c = (unsigned char)s.charAt(i);
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
            out += (char)c;
        } else {
            out += '%';
            out += hex[(c >> 4) & 0x0F];
            out += hex[c & 0x0F];
        }
    }
    return out;
}

static int8_t hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static String radioUnescapeField(const String &s) {
    String out;
    out.reserve(s.length());
    for (size_t i = 0; i < s.length(); i++) {
        char c = s.charAt(i);
        if (c == '%' && (i + 2) < s.length()) {
            int8_t hi = hexNibble(s.charAt(i + 1));
            int8_t lo = hexNibble(s.charAt(i + 2));
            if (hi >= 0 && lo >= 0) {
                out += (char)((hi << 4) | lo);
                i += 2;
                continue;
            }
        }
        out += c;
    }
    return out;
}

static bool radioBase64EncodeUtf8(const String &s, String &out) {
    size_t olen = 0;
    const size_t outLen = 4 * ((s.length() + 2) / 3) + 1;
    unsigned char *buf = (unsigned char*)malloc(outLen);
    if (!buf) return false;

    int rc = mbedtls_base64_encode(buf, outLen, &olen,
                                   (const unsigned char*)s.c_str(), s.length());
    if (rc == 0) {
        buf[olen] = '\0';
        out = String((const char*)buf);
    }
    free(buf);
    return rc == 0;
}

static bool radioBase64DecodeUtf8(const String &s, String &out) {
    size_t olen = 0;
    const size_t outLen = (s.length() * 3) / 4 + 4;
    unsigned char *buf = (unsigned char*)malloc(outLen);
    if (!buf) return false;

    int rc = mbedtls_base64_decode(buf, outLen, &olen,
                                   (const unsigned char*)s.c_str(), s.length());
    if (rc == 0) {
        buf[olen] = '\0';
        out = String((const char*)buf);
    }
    free(buf);
    return rc == 0;
}

static bool isValidUtf8(const String &s) {
    const uint8_t *bytes = (const uint8_t*)s.c_str();
    size_t i = 0;
    size_t len = s.length();

    while (i < len) {
        uint8_t c = bytes[i++];
        if (c <= 0x7F) continue;

        uint8_t continuationCount = 0;
        uint32_t codePoint = 0;
        if ((c & 0xE0) == 0xC0) {
            continuationCount = 1;
            codePoint = c & 0x1F;
            if (codePoint == 0) return false; // overlong 2-byte sequence
        } else if ((c & 0xF0) == 0xE0) {
            continuationCount = 2;
            codePoint = c & 0x0F;
        } else if ((c & 0xF8) == 0xF0) {
            continuationCount = 3;
            codePoint = c & 0x07;
        } else {
            return false;
        }

        if (i + continuationCount > len) return false;
        for (uint8_t n = 0; n < continuationCount; n++) {
            uint8_t next = bytes[i++];
            if ((next & 0xC0) != 0x80) return false;
            codePoint = (codePoint << 6) | (next & 0x3F);
        }

        if ((continuationCount == 1 && codePoint < 0x80) ||
            (continuationCount == 2 && codePoint < 0x800) ||
            (continuationCount == 3 && codePoint < 0x10000) ||
            (codePoint >= 0xD800 && codePoint <= 0xDFFF) ||
            codePoint > 0x10FFFF) {
            return false;
        }
    }
    return true;
}

static String formatStructuredMessage(const String &fromContactId, const String &toContactId, const String &messageBody) {
    String out = fromContactId;
    out += "|";
    out += toContactId;
    out += "|";
    out += messageBody;
    return out;
}

int32_t parseHex16(const String &hex4) {
    if (hex4.length() != 4) return -1;
    char *end = nullptr;
    long v = strtol(hex4.c_str(), &end, 16);
    if (end == hex4.c_str() || *end != '\0') return -1;
    if (v < 0 || v > 0xFFFF) return -1;
    return (int32_t)v;
}

int16_t parseHex8(const String &hex2) {
    if (hex2.length() != 2) return -1;
    char *end = nullptr;
    long v = strtol(hex2.c_str(), &end, 16);
    if (end == hex2.c_str() || *end != '\0') return -1;
    if (v < 0 || v > 0xFF) return -1;
    return (int16_t)v;
}

static bool parseConfigHex8(const char *name, uint8_t &out, String &err) {
    if (!server.hasArg(name)) {
        err = String("Missing parameter: ") + name;
        return false;
    }

    String value = server.arg(name);
    value.trim();
    value.toUpperCase();
    int16_t parsed = parseHex8(value);
    if (parsed < 0) {
        err = String("Invalid hex byte for ") + name + " (expected 2 hex digits)";
        return false;
    }

    out = (uint8_t)parsed;
    return true;
}

static bool parseConfigHex16(const char *name, uint16_t &out, String &err) {
    if (!server.hasArg(name)) {
        err = String("Missing parameter: ") + name;
        return false;
    }

    String value = server.arg(name);
    value.trim();
    value.toUpperCase();
    int32_t parsed = parseHex16(value);
    if (parsed < 0) {
        err = String("Invalid hex word for ") + name + " (expected 4 hex digits)";
        return false;
    }

    out = (uint16_t)parsed;
    return true;
}

String getUptime() {
    unsigned long seconds = (millis() - startTime) / 1000;
    unsigned long hours = seconds / 3600;
    unsigned long minutes = (seconds % 3600) / 60;
    char buf[16];
    snprintf(buf, sizeof(buf), "%luh %lum", hours, minutes);
    return String(buf);
}

String jsonEscape(const String &s) {
    String out = "";
    out.reserve(s.length() + 8);
    for (size_t i = 0; i < s.length(); i++) {
        char c = s.charAt(i);
        if (c == '\\' || c == '"') {
            out += '\\';
            out += c;
        } else if (c == '\n') {
            out += "\\n";
        } else if (c == '\r') {
            out += "\\r";
        } else if (c == '\t') {
            out += "\\t";
        } else {
            out += c;
        }
    }
    return out;
}

String htmlEscape(const String &s) {
    String out;
    out.reserve(s.length() + 16);
    for (size_t i = 0; i < s.length(); i++) {
        char c = s.charAt(i);
        if (c == '&') {
            out += F("&amp;");
        } else if (c == '<') {
            out += F("&lt;");
        } else if (c == '>') {
            out += F("&gt;");
        } else if (c == '"') {
            out += F("&quot;");
        } else {
            out += c;
        }
    }
    return out;
}

static void appendStatusRow(String &html, const char *label, const String &value) {
    html += F("<tr><th>");
    html += label;
    html += F("</th><td>");
    html += htmlEscape(value);
    html += F("</td></tr>");
}

static String defaultCallSignForAddr(uint16_t addr) {
    return String("LM-") + String((uint8_t)(addr & 0xFF));
}

void upsertNearbyNode(uint16_t addr, uint8_t netId, uint8_t channel, int16_t rssi, bool isRepeater,
                      const String &callSignIn, bool hasGpsFixIn = false, double latIn = 0.0, double lngIn = 0.0) {
    if (addr == 0x0000) return;
    unsigned long now = millis();
    for (uint8_t i = 0; i < nearbyCount; i++) {
        if (nearbyNodes[i].addr == addr) {
            nearbyNodes[i].netId = netId;
            nearbyNodes[i].channel = channel;
            nearbyNodes[i].rssi = rssi;
            nearbyNodes[i].lastSeen = now;
            nearbyNodes[i].isRepeater = isRepeater;
            if (callSignIn.length() > 0) nearbyNodes[i].callSign = callSignIn;
            if (hasGpsFixIn) {
                nearbyNodes[i].hasGpsFix = true;
                nearbyNodes[i].lat = latIn;
                nearbyNodes[i].lng = lngIn;
            }
            return;
        }
    }
    if (nearbyCount < MAX_NEARBY_NODES) {
        nearbyNodes[nearbyCount].addr = addr;
        nearbyNodes[nearbyCount].netId = netId;
        nearbyNodes[nearbyCount].channel = channel;
        nearbyNodes[nearbyCount].rssi = rssi;
        nearbyNodes[nearbyCount].lastSeen = now;
        nearbyNodes[nearbyCount].isRepeater = isRepeater;
        nearbyNodes[nearbyCount].callSign = callSignIn.length() > 0 ? callSignIn : defaultCallSignForAddr(addr);
        nearbyNodes[nearbyCount].hasGpsFix = hasGpsFixIn;
        nearbyNodes[nearbyCount].lat = latIn;
        nearbyNodes[nearbyCount].lng = lngIn;
        nearbyCount++;
        return;
    }

    // Replace the oldest entry when full
    uint8_t oldestIdx = 0;
    for (uint8_t i = 1; i < nearbyCount; i++) {
        if (nearbyNodes[i].lastSeen < nearbyNodes[oldestIdx].lastSeen) oldestIdx = i;
    }
    nearbyNodes[oldestIdx].addr = addr;
    nearbyNodes[oldestIdx].netId = netId;
    nearbyNodes[oldestIdx].channel = channel;
    nearbyNodes[oldestIdx].rssi = rssi;
    nearbyNodes[oldestIdx].lastSeen = now;
    nearbyNodes[oldestIdx].isRepeater = isRepeater;
    nearbyNodes[oldestIdx].callSign =
        callSignIn.length() > 0 ? callSignIn : defaultCallSignForAddr(addr);
    nearbyNodes[oldestIdx].hasGpsFix = hasGpsFixIn;
    nearbyNodes[oldestIdx].lat = latIn;
    nearbyNodes[oldestIdx].lng = lngIn;
}

static double deg2rad(double deg) { return deg * (PI / 180.0); }

static double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
    // Earth radius mean (meters)
    const double R = 6371000.0;
    const double dLat = deg2rad(lat2 - lat1);
    const double dLon = deg2rad(lon2 - lon1);
    const double a =
        sin(dLat / 2.0) * sin(dLat / 2.0) +
        cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
        sin(dLon / 2.0) * sin(dLon / 2.0);
    const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return R * c;
}

// =============================================================================
// OLED Display
// =============================================================================

void drawDisplay() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    
    // Node info
    display.setCursor(0, 0);  
    display.print("Node:");
    display.print(toHex2(MY_ADDH));
    display.print(toHex2(MY_ADDL));
    display.print(" N:");
    display.print(toHex2(NETWORK_ID));
    
    // IP address (last octet only)
    display.setCursor(0, 12); 
    display.print("IP: ");
    display.print(WiFi.softAPIP().toString());
    
    // Stats
    display.setCursor(0, 22);
    display.print("RX:"); 
    display.print(receivedCount);
    display.print(" TX:"); 
    display.print(sentCount);
    
    // Last received
    display.setCursor(0, 32);
    if (lastReceived.length() > 0) {
        display.print("RX:");
        display.print(lastReceived.substring(0, 13));
    }
    
    // Last sent
    display.setCursor(0, 42);
    if (lastSent.length() > 0) {
        display.print("TX:");
        display.print(lastSent.substring(0, 13));
    }
    
    // Status
    display.setCursor(0, 52);
    display.print("Up:");
    display.print(getUptime());
    
    display.display();
}

// =============================================================================
// Wait for AUX
// =============================================================================

bool waitAuxHigh(uint32_t timeoutMs = 5000) {
    uint32_t t = millis();
    while (digitalRead(PIN_AUX) == LOW) {
        if (millis() - t > timeoutMs) return false;
        delay(10);
    }
    return true;
}

// =============================================================================
// Apply Configuration to E22
// =============================================================================

static bool radioTransmissionAllowed() {
    return !batteryCharging && !brownoutRecoveryMode;
}

bool applyNodeConfig() {
    if (!waitAuxHigh(8000)) return false;
    delay(100);

    ResponseStructContainer c = e22.getConfiguration();
    if (c.status.code != 1) { 
        c.close(); 
        return false; 
    }

    Configuration config = *(Configuration*)c.data;
    c.close();

    // Set addresses and network
    config.ADDH  = MY_ADDH;
    config.ADDL  = MY_ADDL;
    config.NETID = NETWORK_ID;
    config.CHAN  = CHANNEL;

    // E22 hardware AES key (high byte first per module register map)
    config.CRYPT.CRYPT_H = highByte(CRYPT);
    config.CRYPT.CRYPT_L = lowByte(CRYPT);

    // Common settings
    config.SPED.uartBaudRate = UART_BPS_9600;
    config.SPED.airDataRate  = AIR_DATA_RATE_010_24;
    config.SPED.uartParity   = MODE_00_8N1;

    config.OPTION.subPacketSetting  = SPS_240_00;
    config.OPTION.RSSIAmbientNoise  = RSSI_AMBIENT_NOISE_DISABLED;
    // This hardware resets on weak USB/serial power during RF bursts. Keep the
    // radio at its lowest-current output in this stable firmware version.
    config.OPTION.transmissionPower = STABLE_E22_TX_POWER;

    config.TRANSMISSION_MODE.enableRSSI            = RSSI_ENABLED;
    config.TRANSMISSION_MODE.fixedTransmission     = FT_FIXED_TRANSMISSION;
    // Application relay routing uses RELAY packets; a Node must not
    // automatically repeat received packets in hardware.
    config.TRANSMISSION_MODE.enableRepeater        = REPEATER_DISABLED;
    config.TRANSMISSION_MODE.enableLBT             = LBT_DISABLED;
    config.TRANSMISSION_MODE.WORTransceiverControl = WOR_RECEIVER;
    config.TRANSMISSION_MODE.WORPeriod             = WOR_2000_011;

    ResponseStatus rs = e22.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
    if (rs.code != 1) return false;

    delay(300);
    if (!waitAuxHigh(5000)) return false;
    delay(100);

    Serial.println("[CONFIG] Applied to E22 module");
    Serial.printf("[KEY] CRYPT=0x%04X (H:%s L:%s)\n", CRYPT,
                  toHex2(highByte(CRYPT)).c_str(), toHex2(lowByte(CRYPT)).c_str());
    Serial.println("[POWER] Stable mode active: E22 TX power ~10 dBm, hardware repeater disabled");
    return true;
}

// =============================================================================
// Send Message
// =============================================================================

static bool waitForAck(const String& msgId, unsigned long timeoutMs) {
    awaitingAckId = msgId;
    awaitingAckReceived = false;

    unsigned long started = millis();
    while ((millis() - started) < timeoutMs) {
        checkIncoming();
        if (awaitingAckReceived) {
            awaitingAckId = "";
            return true;
        }
        delay(10);
    }

    awaitingAckId = "";
    return false;
}

bool sendMessage(String msg, bool viaRepeater = false, uint8_t destAddh = 0xFF, uint8_t destAddl = 0xFF, bool overrideDest = false) {
    if (!radioTransmissionAllowed()) {
        statusMsg = batteryCharging ? "CHARGING RX ONLY" : "BROWNOUT RX ONLY";
        Serial.println("[TX] Blocked while charging/brownout protection is active");
        return false;
    }
    if (msg.length() > MAX_CHAT_BODY_UTF8_BYTES) {
        statusMsg = "MSG TOO LONG";
        Serial.printf("[TX] Message too long: %u bytes (max %u)\n",
                      (unsigned)msg.length(), (unsigned)MAX_CHAT_BODY_UTF8_BYTES);
        return false;
    }

    uint8_t finalAddh = overrideDest ? destAddh : TARGET_ADDH;
    uint8_t finalAddl = overrideDest ? destAddl : TARGET_ADDL;
    String myAddrHex = toHex4((MY_ADDH << 8) | MY_ADDL);
    String toAddrHex = toHex4((((uint16_t)finalAddh) << 8) | finalAddl);
    String msgId = toHex4(++txMessageCounter);
    if (viaRepeater) msgId += "R";  // Existing MSG3 readers still decode the UTF-8 body.
    // Send chat bodies as UTF-8 bytes, not base64. The legacy MSG parser
    // already treats everything after the source field as message text, so
    // characters such as Khmer remain human-readable through a relay.
    String payload = "MSG|" + msgId + "|" + myAddrHex + "|" + msg;
    String displayMsg = formatStructuredMessage(myAddrHex, toAddrHex, msg);

    bool acked = false;
    for (uint8_t attempt = 0; attempt <= ACK_RETRY_MAX; attempt++) {
        if (viaRepeater) {
            // Send via repeater with relay envelope.
            String destHex = toHex4((finalAddh << 8) | finalAddl);
            String relayMsg = "RELAY|" + destHex + "|" + payload;
            ResponseStatus rs = e22.sendFixedMessage(REPEATER_ADDH, REPEATER_ADDL, CHANNEL, relayMsg);
            Serial.printf("[TX] Via repeater to 0x%02X%02X (id=%s try=%u): %s\n",
                finalAddh, finalAddl, msgId.c_str(), (unsigned)(attempt + 1), rs.getResponseDescription().c_str());
            if (rs.code == 1 && waitForAck(msgId, ACK_WAIT_MS)) {
                acked = true;
                break;
            }
        } else {
            ResponseStatus rs = e22.sendFixedMessage(finalAddh, finalAddl, CHANNEL, payload);
            Serial.printf("[TX] Direct to 0x%02X%02X (id=%s try=%u): %s\n",
                finalAddh, finalAddl, msgId.c_str(), (unsigned)(attempt + 1), rs.getResponseDescription().c_str());
            if (rs.code == 1 && waitForAck(msgId, ACK_WAIT_MS)) {
                acked = true;
                break;
            }
        }
    }

    sentCount++;
    lastSent = displayMsg;
    statusMsg = acked ? "SENT ACK" : "SENT NO ACK";
    if (viaRepeater) {
        addLogEntry(acked ? "TX-RELAY-ACK" : "TX-RELAY-NOACK", 0, toHex4((REPEATER_ADDH << 8) | REPEATER_ADDL), displayMsg);
    } else {
        addLogEntry(acked ? "TX-ACK" : "TX-NOACK", 0, toHex4((finalAddh << 8) | finalAddl), displayMsg);
    }
    return acked;
}

// Short beep pattern when a chat frame (MSG3 / MSG2 / legacy MSG) is received — not HELLO / ACK.
static void buzzerAlertNewChatMessage() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(BUZZER_ACTIVE, HIGH);
        delay(45);
        digitalWrite(BUZZER_ACTIVE, LOW);
        delay(55);
    }
}

// =============================================================================
// Receive Messages
// =============================================================================

static void sendChatAck(const String &msgId, const String &srcHex, bool viaRepeater) {
    if (!radioTransmissionAllowed()) {
        Serial.printf("[ACK] Not transmitted while protected (id=%s)\n", msgId.c_str());
        return;
    }
    int32_t srcAddr = parseHex16(srcHex);
    if (srcAddr < 0) return;

    String ackPayload = "ACK|" + msgId;
    if (viaRepeater) {
        String relayAck = "RELAY|" + srcHex + "|" + ackPayload;
        e22.sendFixedMessage(REPEATER_ADDH, REPEATER_ADDL, CHANNEL, relayAck);
        return;
    }

    uint8_t srcAddh = (uint8_t)((srcAddr >> 8) & 0xFF);
    uint8_t srcAddl = (uint8_t)(srcAddr & 0xFF);
    e22.sendFixedMessage(srcAddh, srcAddl, CHANNEL, ackPayload);
}

void checkIncoming() {
    if (e22.available() <= 0) return;

    ResponseContainer rc = e22.receiveMessageRSSI();
    if (rc.status.code != 1) return;

    String payload = rc.data;
    const String rxRaw = payload;

    // ACK frame format: ACK|<msgId>
    if (payload.startsWith("ACK|")) {
        String ackId = payload.substring(4);
        int sep = ackId.indexOf('|');
        if (sep > 0) ackId = ackId.substring(0, sep);
        ackId.trim();

        if (awaitingAckId.length() > 0 && ackId == awaitingAckId) {
            awaitingAckReceived = true;
            statusMsg = "ACK RX";
            Serial.printf("[ACK] Received for id=%s\n", ackId.c_str());
        }
        return;
    }

    String logAddr = "????";

    // Current format: MSG3|<msgId>|<fromContactId>|<toContactId>|<base64Utf8Body>
    // Base64 keeps Khmer UTF-8 intact without the large expansion of percent encoding.
    if (payload.startsWith("MSG3|")) {
        int p1 = payload.indexOf('|');           // after MSG3
        int p2 = payload.indexOf('|', p1 + 1);   // after id
        int p3 = payload.indexOf('|', p2 + 1);   // after from
        int p4 = payload.indexOf('|', p3 + 1);   // after to
        if (p1 >= 0 && p2 > p1 && p3 > p2 && p4 > p3) {
            String msgId = payload.substring(p1 + 1, p2);
            String fromContactId = payload.substring(p2 + 1, p3);
            String toContactId = payload.substring(p3 + 1, p4);
            String body64 = payload.substring(p4 + 1);
            String body;
            if (!radioBase64DecodeUtf8(body64, body) || !isValidUtf8(body)) {
                body = "[invalid UTF-8 message]";
            }
            logAddr = fromContactId;

            sendChatAck(msgId, fromContactId, msgId.endsWith("R"));
            payload = formatStructuredMessage(fromContactId, toContactId, body);
        }
    }

    // Previous format: MSG2|<msgId>|<fromContactId>|<toContactId>|<urlEncodedBody>
    if (payload.startsWith("MSG2|")) {
        int p1 = payload.indexOf('|');           // after MSG2
        int p2 = payload.indexOf('|', p1 + 1);   // after id
        int p3 = payload.indexOf('|', p2 + 1);   // after from
        int p4 = payload.indexOf('|', p3 + 1);   // after to
        if (p1 >= 0 && p2 > p1 && p3 > p2 && p4 > p3) {
            String msgId = payload.substring(p1 + 1, p2);
            String fromContactId = payload.substring(p2 + 1, p3);
            String toContactId = payload.substring(p3 + 1, p4);
            String body = radioUnescapeField(payload.substring(p4 + 1));
            if (!isValidUtf8(body)) body = "[invalid UTF-8 message]";
            logAddr = fromContactId;

            sendChatAck(msgId, fromContactId, false);
            payload = formatStructuredMessage(fromContactId, toContactId, body);
        }
    }

    // Legacy frame format: MSG|<msgId>|<srcAddrHex>|<body>
    if (payload.startsWith("MSG|")) {
        int p1 = payload.indexOf('|');          // after MSG
        int p2 = payload.indexOf('|', p1 + 1);  // after id
        int p3 = payload.indexOf('|', p2 + 1);  // after src
        if (p1 >= 0 && p2 > p1 && p3 > p2) {
            String msgId = payload.substring(p1 + 1, p2);
            String srcHex = payload.substring(p2 + 1, p3);
            String body = payload.substring(p3 + 1);
            if (!isValidUtf8(body)) body = "[invalid UTF-8 message]";
            logAddr = srcHex;

            sendChatAck(msgId, srcHex, msgId.endsWith("R"));
            payload = formatStructuredMessage(srcHex, toHex4((MY_ADDH << 8) | MY_ADDL), body);
        }
    }

    receivedCount++;
    const int16_t rxRssiDbm = e22RssiByteToDbm(rc.rssi);
    lastReceived = payload;
    lastRssiSignal = rxRssiDbm;
    statusMsg = "RX OK";

    if (rxRaw.startsWith("MSG3|") || rxRaw.startsWith("MSG2|") || rxRaw.startsWith("MSG|")) {
        noteDisplayActivity();
        addLogEntry("RX", rxRssiDbm, logAddr, payload);
        if (alertsBeepEnabled) {
            buzzerAlertNewChatMessage();
        }
    }

    // Learn nearby nodes via HELLO beacons:
    // Payload format: HELLO|AABB|NN|CC|R/N  or  HELLO|AABB|NN|CC|R/N|callSign
    if (payload.startsWith("HELLO|")) {
        int p1 = payload.indexOf('|');                 // after HELLO
        int p2 = payload.indexOf('|', p1 + 1);         // after addr
        int p3 = payload.indexOf('|', p2 + 1);         // after net
        int p4 = payload.indexOf('|', p3 + 1);         // after chan

        if (p1 >= 0 && p2 > p1 && p3 > p2 && p4 > p3) {
            String addrHex = payload.substring(p1 + 1, p2);
            String netHex  = payload.substring(p2 + 1, p3);
            String chHex   = payload.substring(p3 + 1, p4);
            // Optional fields:
            // HELLO|AABB|NN|CC|R/N
            // HELLO|AABB|NN|CC|R/N|callSign
            // HELLO|AABB|NN|CC|R/N|callSign|lat|lng
            int p5 = payload.indexOf('|', p4 + 1);      // after role
            String role, csIn;
            bool hasFix = false;
            double latIn = 0.0, lngIn = 0.0;

            if (p5 < 0) {
                role = payload.substring(p4 + 1);
                role.trim();
            } else {
                role = payload.substring(p4 + 1, p5);
                role.trim();

                int p6 = payload.indexOf('|', p5 + 1);  // after callSign
                if (p6 < 0) {
                    csIn = payload.substring(p5 + 1);
                    csIn.trim();
                } else {
                    csIn = payload.substring(p5 + 1, p6);
                    csIn.trim();

                    int p7 = payload.indexOf('|', p6 + 1); // after lat
                    if (p7 > p6) {
                        String latS = payload.substring(p6 + 1, p7);
                        String lngS = payload.substring(p7 + 1);
                        latS.trim();
                        lngS.trim();
                        if (latS.length() > 0 && lngS.length() > 0) {
                            latIn = latS.toDouble();
                            lngIn = lngS.toDouble();
                            if (latIn >= -90.0 && latIn <= 90.0 && lngIn >= -180.0 && lngIn <= 180.0) {
                                hasFix = true;
                            }
                        }
                    }
                }
            }

            int32_t addrVal = parseHex16(addrHex);
            int16_t netVal  = parseHex8(netHex);
            int16_t chVal   = parseHex8(chHex);
            bool isRep = (role.length() > 0 && role.charAt(0) == 'R');
            if (addrVal >= 0 && netVal >= 0 && chVal >= 0) {
                upsertNearbyNode((uint16_t)addrVal, (uint8_t)netVal, (uint8_t)chVal, rxRssiDbm, isRep,
                                 csIn, hasFix, latIn, lngIn);
            }
        }
    }
    
    // addLogEntry("RX", (int8_t)rc.rssi, "????", rc.data);
    
    // Serial.printf("[RX] RSSI=%d msg=%s\n", (int8_t)rc.rssi, rc.data.c_str());
    // drawDisplay();
}

// =============================================================================
// Web Handlers
// =============================================================================

void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>LoRa Node - Dynamic Config</title>
  <style>
    body{font-family:Arial,'Noto Sans Khmer','Khmer OS','Khmer Sangam MN',sans-serif;margin:0;padding:20px;background:#1a1a2e;color:#eee;}
    h1{color:#0af;}
    h2{color:#0f3;margin-top:30px;}
    .container{max-width:1200px;margin:0 auto;}
    .card{background:#16213e;padding:20px;border-radius:8px;margin-bottom:20px;box-shadow:0 4px 8px rgba(0,0,0,0.3);}
    .stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:15px;margin-bottom:20px;}
    .stat-box{background:#0f3460;padding:15px;border-radius:5px;text-align:center;}
    .stat-box h3{margin:0 0 10px 0;font-size:14px;color:#aaa;}
    .stat-box .value{font-size:24px;font-weight:bold;color:#0af;}
    .config-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:15px;}
    .input-group{margin-bottom:15px;}
    .input-group label{display:block;margin-bottom:5px;color:#aaa;font-size:14px;}
    input[type=text],select{width:100%;padding:10px;background:#0f3460;color:#fff;border:1px solid #0af;border-radius:5px;font-size:14px;}
    input[type=checkbox]{width:20px;height:20px;margin-right:10px;}
    button{background:#0af;color:#000;border:none;padding:12px 25px;border-radius:5px;cursor:pointer;font-size:16px;font-weight:bold;margin:5px;}
    button:hover{background:#08d;}
    button.save{background:#0f3;}
    button.save:hover{background:#0c2;}
    button.relay{background:#f90;}
    button.relay:hover{background:#d70;}
    button.restart{background:#f00;}
    button.restart:hover{background:#c00;}
    button.gps{background:#2a9d8f;}
    button.gps:hover{background:#21867a;}
    table{width:100%;border-collapse:collapse;margin-top:15px;}
    th,td{padding:10px;text-align:left;border-bottom:1px solid #333;}
    th{background:#0f3460;color:#0af;}
    tr:hover{background:#1f2f4f;}
    .tx{color:#0f3;}
    .rx{color:#0af;}
    .warning{background:#f90;color:#000;padding:10px;border-radius:5px;margin:10px 0;}
  </style>
  <script>
    const MAX_CHAT_BODY_UTF8_BYTES = 150;

    function utf8ByteLength(s) {
      return new TextEncoder().encode(s).length;
    }

    function sendMsg(useRelay) {
      let msg = document.getElementById('msgInput').value;
      if (!msg) { alert('Enter a message!'); return; }
      const msgBytes = utf8ByteLength(msg);
      if (msgBytes > MAX_CHAT_BODY_UTF8_BYTES) {
        alert('Message too long: ' + msgBytes + ' UTF-8 bytes. Limit is ' + MAX_CHAT_BODY_UTF8_BYTES + ' bytes.');
        return;
      }
      
      let to = document.getElementById('nodeSelect') ? document.getElementById('nodeSelect').value : '';
      let url = '/send' + (useRelay ? '?relay=1' : '');
      if (to && to !== 'default') url += (url.indexOf('?') >= 0 ? '&' : '?') + 'to=' + encodeURIComponent(to);
      fetch(url, {
        method: 'POST',
        headers: { 'Content-Type': 'text/plain; charset=utf-8' },
        body: msg
      })
        .then(r => r.text().then(t => ({ ok: r.ok, text: t })))
        .then(result => {
          if (!result.ok) {
            alert(result.text.substring(0, 200));
            return;
          }
          document.getElementById('msgInput').value = '';
          refreshData();
        });
    }

    function sendGpsMsg(useRelay) {
      let to = document.getElementById('nodeSelect') ? document.getElementById('nodeSelect').value : '';
      let base = useRelay ? '/gps?relay=1' : '/gps?';
      let url = base;
      if (to && to !== 'default') url += (base.endsWith('?') ? '' : '&') + 'to=' + encodeURIComponent(to);
      fetch(url)
        .then(r => r.text())
        .then(t => { alert(t.substring(0, 200)); refreshData(); });
    }
    
    function saveConfig() {
      let data = {
        my_addh: document.getElementById('my_addh').value,
        my_addl: document.getElementById('my_addl').value,
        target_addh: document.getElementById('target_addh').value,
        target_addl: document.getElementById('target_addl').value,
        rep_addh: document.getElementById('rep_addh').value,
        rep_addl: document.getElementById('rep_addl').value,
        netid: document.getElementById('netid').value,
        channel: document.getElementById('channel').value,
        crypt: document.getElementById('crypt').value,
        use_repeater: document.getElementById('use_repeater').checked ? '1' : '0'
      };
      
      let params = new URLSearchParams(data).toString();
      
      if(confirm('Save configuration and restart node?')) {
        fetch('/config?' + params)
          .then(r => r.text())
          .then(msg => {
            alert(msg);
            setTimeout(() => location.reload(), 3000);
          });
      }
    }
    
    function refreshData() {
      fetch('/api/status')
        .then(r => r.json())
        .then(data => {
          // Support both old flat JSON and new nested JSON.
          const node = data.node || data;
          const traffic = data.traffic || data;
          const fmtHex = (value, fallback = '--') => value ? ('0x' + value) : fallback;

          document.getElementById('uptime').textContent = node.uptime || '--';
          document.getElementById('sentCount').textContent = (traffic.sent ?? 0);
          document.getElementById('rxCount').textContent = (traffic.received ?? 0);
          document.getElementById('lastTx').textContent = traffic.lastSent || '(none)';
          document.getElementById('lastRx').textContent = traffic.lastReceived || '(none)';
          document.getElementById('myAddr').textContent = fmtHex(node.myAddr);
          document.getElementById('targetAddr').textContent = fmtHex(node.targetAddr);
          document.getElementById('netId').textContent = fmtHex(node.netId);
        });
      
      fetch('/api/log')
        .then(r => r.json())
        .then(data => {
          let tbody = document.getElementById('logBody');
          tbody.innerHTML = '';
          data.forEach(entry => {
            let tr = document.createElement('tr');
            function td(text, className) {
              const cell = document.createElement('td');
              if (className) cell.className = className;
              cell.textContent = text;
              return cell;
            }
            tr.appendChild(td(entry.time || ''));
            tr.appendChild(td(entry.dir || '', String(entry.dir || '').toLowerCase()));
            tr.appendChild(td((entry.rssi ?? '') + ' dBm'));
            tr.appendChild(td('0x' + (entry.addr || '')));
            tr.appendChild(td(entry.msg || ''));
            tbody.appendChild(tr);
          });
        });

      fetch('/api/nodes')
        .then(r => r.json())
        .then(payload => {
          const sel = document.getElementById('nodeSelect');
          if (!sel) return;

          const nodes = Array.isArray(payload) ? payload : (payload.nodes || []);

          const keep = sel.value;
          sel.innerHTML = '';

          const optDefault = document.createElement('option');
          optDefault.value = 'default';
          optDefault.textContent = 'Use configured target';
          sel.appendChild(optDefault);

          nodes
            .sort((a, b) => (a.lastSeenMs ?? 0) - (b.lastSeenMs ?? 0))
            .forEach(n => {
              const opt = document.createElement('option');
              opt.value = n.addr; // AABB
              const age = (n.lastSeenMs ?? 0);
              const sec = Math.round(age / 1000);
              const on = (n.online !== false);
              opt.textContent = `0x${n.addr}  RSSI:${n.rssi}  ${on ? '' : '[off] '}seen:${sec}s${n.repeater ? '  (repeater)' : ''}`;
              sel.appendChild(opt);
            });

          if (keep) sel.value = keep;
        });
    }
    
    setInterval(refreshData, 2000);
    window.onload = refreshData;
  </script>
</head>
<body>
  <div class="container">
    <h1>📡 LoRa Node - Dynamic Configuration</h1>
    <p style="margin:-8px 0 16px 0"><a href="/setup" style="color:#0af">WiFi setup</a> · <a href="/setPower" style="color:#0af">E22 TX power</a> · <a href="/status" style="color:#0af">Device &amp; E22 status</a> · <a href="/api/status" style="color:#888">JSON</a></p>
    
    <div class="card">
      <h2>Current Status</h2>
      <div class="stats">
        <div class="stat-box">
          <h3>My Address</h3>
          <div class="value" id="myAddr">)rawliteral"+ toHex2(MY_ADDH) + toHex2(MY_ADDL)
            + R"rawliteral("</div>
        </div>
        <div class="stat-box">
          <h3>Target</h3>
          <div class="value" id="targetAddr">)rawliteral"+ toHex2(TARGET_ADDH) + toHex2(TARGET_ADDL)
            + R"rawliteral("</div>
        </div>
        <div class="stat-box">
          <h3>Network ID</h3>
          <div class="value" id="netId">)rawliteral"+ toHex2(NETWORK_ID)
            + R"rawliteral("</div>
        </div>
        <div class="stat-box">
          <h3>Uptime</h3>
          <div class="value" id="uptime">--</div>
        </div>
        <div class="stat-box">
          <h3>Received</h3>
          <div class="value" id="rxCount">0</div>
        </div>
        <div class="stat-box">
          <h3>Sent</h3>
          <div class="value" id="sentCount">0</div>
        </div>
      </div>
    </div>
    
    <div class="card">
      <h2>⚙️ Node Configuration</h2>
      <div class="warning">⚠️ Changing configuration will restart the node!</div>
      
      <div class="config-grid">
        <div class="input-group">
          <label>My Address High (ADDH)</label>
          <input type="text" id="my_addh" value=")rawliteral"+ toHex2(MY_ADDH)
            + R"rawliteral(" maxlength="2" placeholder="00">
        </div>
        <div class="input-group">
          <label>My Address Low (ADDL)</label>
          <input type="text" id="my_addl" value=")rawliteral"+ toHex2(MY_ADDL)
          +R"rawliteral(" maxlength="2" placeholder="02">
        </div>
        <div class="input-group">
          <label>Target Address High</label>
          <input type="text" id="target_addh" value=")rawliteral"+ toHex2(TARGET_ADDH)
          +R"rawliteral(" maxlength="2" placeholder="00">
        </div>
        <div class="input-group">
          <label>Target Address Low</label>
          <input type="text" id="target_addl" value=")rawliteral"+ toHex2(TARGET_ADDL)
          +R"rawliteral(" maxlength="2" placeholder="01">
        </div>
        <div class="input-group">
          <label>Repeater Address High</label>
          <input type="text" id="rep_addh" value=")rawliteral"+ toHex2(REPEATER_ADDH)
          +R"rawliteral(" maxlength="2" placeholder="FF">
        </div>
        <div class="input-group">
          <label>Repeater Address Low</label>
          <input type="text" id="rep_addl" value=")rawliteral"+ toHex2(REPEATER_ADDL)
          +R"rawliteral(" maxlength="2" placeholder="FF">
        </div>
        <div class="input-group">
          <label>Network ID (NETID)</label>
          <input type="text" id="netid" value=")rawliteral"+ toHex2(NETWORK_ID)
          +R"rawliteral(" maxlength="2" placeholder="00">
        </div>
        <div class="input-group">
          <label>Channel (00-FF)</label>
          <input type="text" id="channel" value=")rawliteral"+ toHex2(CHANNEL)
          +R"rawliteral(" maxlength="2" placeholder="41">
        </div>
        <div class="input-group">
          <label>CRYPT key (4 hex digits, same on all nodes; 0000 = off)</label>
          <input type="text" id="crypt" value=")rawliteral" + toHex4(CRYPT) + R"rawliteral(" maxlength="4" placeholder="1234">
        </div>
      </div>
      
      <div class="input-group">
        <label>
          <input type="checkbox" id="use_repeater" )rawliteral" + String(USE_REPEATER ? "checked" : "") + R"rawliteral(>
          Enable as Repeater (forward all messages)
        </label>
      </div>
      
      <button class="save" onclick="saveConfig()">💾 Save & Apply Configuration</button>
    </div>
    
    <div class="card">
      <h2>Send Message</h2>
      <p><strong>Last RX:</strong> <span id="lastRx">(none)</span></p>
      <p><strong>Last TX:</strong> <span id="lastTx">(none)</span></p>
      <div class="input-group">
        <label>Send To (nearby nodes)</label>
        <select id="nodeSelect">
          <option value="default">Use configured target</option>
        </select>
      </div>
      <input type="text" id="msgInput" placeholder="Type your message..." maxlength="200">
      <div>
        <button onclick="sendMsg(false)">📤 Send Direct</button>
        <button class="relay" onclick="sendMsg(true)">🔄 Send via Repeater</button>
      </div>
      <p style="color:#aaa;font-size:14px;margin-top:14px;">GPS snapshot (LoRa text <code style="color:#0af">GPS,Stas,sats,lat,lng</code>):</p>
      <div>
        <button class="gps" onclick="sendGpsMsg(false)">📍 Send GPS (direct)</button>
        <button class="gps" onclick="sendGpsMsg(true)">📍 Send GPS (repeater)</button>
        <br>
        <a href="/gps" style="color:#888;font-size:13px;margin-left:8px;" target="_blank" rel="noopener">GPS</a>
      </div>
    </div>
    
    <div class="card">
      <h2>Message Log</h2>
      <table>
        <thead>
          <tr>
            <th>Time</th>
            <th>Direction</th>
            <th>RSSI</th>
            <th>Address</th>
            <th>Message</th>
          </tr>
        </thead>
        <tbody id="logBody">
          <tr><td colspan="5" style="text-align:center;">Loading...</td></tr>
        </tbody>
      </table>
    </div>
  </div>
</body>
</html>
)rawliteral";
    server.send(200, "text/html; charset=utf-8", html);
}

void handleSetupPage() {
    String html;
    html.reserve(2500);
    html += F("<!DOCTYPE html><html><head><meta charset=\"utf-8\">");
    html += F("<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">");
    html += F("<title>WiFi Setup</title>");
    html += F("<style>");
    html += F("body{font-family:Arial,'Noto Sans Khmer','Khmer OS','Khmer Sangam MN',sans-serif;margin:0;padding:20px;background:#1a1a2e;color:#eee;}");
    html += F(".card{background:#16213e;padding:20px;border-radius:8px;max-width:680px;margin:0 auto;box-shadow:0 4px 8px rgba(0,0,0,0.3);}");
    html += F("h1{color:#0af;margin-top:0;}label{display:block;margin:10px 0 6px;color:#aaa;}");
    html += F("input[type=text]{width:100%;padding:10px;background:#0f3460;color:#fff;border:1px solid #0af;border-radius:5px;font-size:15px;box-sizing:border-box;}");
    html += F("button{background:#0f3;color:#000;border:none;padding:12px 20px;border-radius:5px;cursor:pointer;font-size:15px;font-weight:bold;margin-top:14px;}");
    html += F("button:hover{background:#0c2;}a{color:#0af;}");
    html += F(".muted{color:#888;font-size:13px;}.warn{margin-top:12px;padding:10px;background:#f90;color:#000;border-radius:5px;}");
    html += F("</style></head><body><div class=\"card\">");
    html += F("<p><a href=\"/\">&larr; Back to control panel</a></p>");
    html += F("<h1>WiFi AP setup</h1>");
    html += F("<form method=\"POST\" action=\"/setup/save\">");
    html += F("<label for=\"ssid_ap\">AP SSID</label>");
    html += F("<input id=\"ssid_ap\" name=\"ssid_ap\" type=\"text\" maxlength=\"32\" required value=\"");
    html += htmlEscape(ssid_AP);
    html += F("\">");
    html += F("<p class=\"muted\">Current AP IP: ");
    html += htmlEscape(WiFi.softAPIP().toString());
    html += F("</p>");
    html += F("<div class=\"warn\">Saving will restart the device and AP will reconnect with the new SSID.</div>");
    html += F("<button type=\"submit\">Save to flash & restart</button>");
    html += F("</form></div></body></html>");
    server.send(200, "text/html", html);
}

void handleSetupSave() {
    if (!server.hasArg("ssid_ap")) {
        server.send(400, "text/plain", "Missing ssid_ap");
        return;
    }

    String newSsid = server.arg("ssid_ap");
    newSsid.trim();

    if (newSsid.length() == 0 || newSsid.length() > 32) {
        server.send(400, "text/plain", "Invalid SSID length (1..32)");
        return;
    }

    ssid_AP = newSsid;
    saveApConfig();

    server.send(200, "text/plain", "SSID saved to flash. Restarting...");
    delay(1000);
    ESP.restart();
}

void handleSetPowerPage() {
    String html;
    html.reserve(2800);
    html += F("<!DOCTYPE html><html><head><meta charset=\"utf-8\">");
    html += F("<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">");
    html += F("<title>E22 TX Power</title>");
    html += F("<style>");
    html += F("body{font-family:Arial,'Noto Sans Khmer','Khmer OS','Khmer Sangam MN',sans-serif;margin:0;padding:20px;background:#1a1a2e;color:#eee;}");
    html += F(".card{background:#16213e;padding:20px;border-radius:8px;max-width:680px;margin:0 auto;box-shadow:0 4px 8px rgba(0,0,0,0.3);}");
    html += F("h1{color:#0af;margin-top:0;}label{display:block;margin:10px 0 6px;color:#aaa;}");
    html += F("select{width:100%;padding:10px;background:#0f3460;color:#fff;border:1px solid #0af;border-radius:5px;font-size:15px;box-sizing:border-box;}");
    html += F("button{background:#0f3;color:#000;border:none;padding:12px 20px;border-radius:5px;cursor:pointer;font-size:15px;font-weight:bold;margin-top:14px;}");
    html += F("button:hover{background:#0c2;}a{color:#0af;}");
    html += F(".muted{color:#888;font-size:13px;}.ok{margin-top:12px;padding:10px;background:#0a3;color:#afa;border-radius:5px;}");
    html += F("</style></head><body><div class=\"card\">");
    html += F("<p><a href=\"/\">&larr; Back to control panel</a></p>");
    html += F("<h1>E22 transmission power</h1>");
    html += F("<p class=\"muted\">Applies the same register as <code>OPTION.transmissionPower</code> (E22-900 series: ~22 / 17 / 13 / 10 dBm).</p>");
    if (server.hasArg("ok")) {
        html += F("<p class=\"ok\">Saved and applied to the radio (no restart).</p>");
    }
    html += F("<form method=\"POST\" action=\"/setPower/save\">");
    html += F("<label for=\"power\">Output power</label>");
    html += F("<select id=\"power\" name=\"power\" required>");
    const char *opts[] = {"22 dBm (max)", "17 dBm", "13 dBm", "10 dBm (min)"};
    for (uint8_t i = 0; i < 4; i++) {
        html += F("<option value=\"");
        html += String(i);
        html += F("\"");
        if (E22_TX_POWER == i) html += F(" selected");
        html += F(">");
        html += opts[i];
        html += F("</option>");
    }
    html += F("</select>");
    html += F("<button type=\"submit\">Save &amp; apply to E22</button>");
    html += F("</form>");
    html += F("<p class=\"muted\">Current stored index: ");
    html += String(E22_TX_POWER);
    html += F(" · <a href=\"/status\">Verify on status page</a></p>");
    html += F("</div></body></html>");
    server.send(200, "text/html", html);
}

void handleSetPowerSave() {
    if (!server.hasArg("power")) {
        server.send(400, "text/plain", "Missing power");
        return;
    }
    int p = server.arg("power").toInt();
    if (p < 0 || p > 3) {
        server.send(400, "text/plain", "Invalid power (0..3)");
        return;
    }
    E22_TX_POWER = (uint8_t)p;
    saveConfig();
    if (!applyNodeConfig()) {
        server.send(500, "text/html",
                     "<!DOCTYPE html><html><head><meta charset=\"utf-8\"><title>TX power</title></head>"
                     "<body style=\"font-family:sans-serif;background:#1a1a2e;color:#eee;padding:20px;\">"
                     "<p>Failed to write configuration to E22. Check wiring / AUX.</p>"
                     "<p><a style=\"color:#0af\" href=\"/setPower\">Back</a></p></body></html>");
        return;
    }
    server.sendHeader("Location", "/setPower?ok=1");
    server.send(303, "text/plain", "");
}

static void parseOptionalSendDest(uint8_t &destAddh, uint8_t &destAddl, bool &overrideDest) {
    overrideDest = false;
    if (server.hasArg("to")) {
        int32_t addr = parseHex16(server.arg("to"));
        if (addr >= 0) {
            destAddh = (uint8_t)((addr >> 8) & 0xFF);
            destAddl = (uint8_t)(addr & 0xFF);
            overrideDest = true;
        }
    }
    if (server.hasArg("addh") && server.hasArg("addl")) {
        int16_t ah = parseHex8(server.arg("addh"));
        int16_t al = parseHex8(server.arg("addl"));
        if (ah >= 0 && al >= 0) {
            destAddh = (uint8_t)ah;
            destAddl = (uint8_t)al;
            overrideDest = true;
        }
    }
}

void handleSend() {
    String msg;
    if (server.hasArg("plain")) {
        msg = server.arg("plain");
    } else if (server.hasArg("msg")) {
        msg = server.arg("msg"); // Backward-compatible query/form requests.
    }

    if (msg.length() == 0) {
        server.send(400, "text/plain; charset=utf-8", "Missing or empty message");
        return;
    }
    if (msg.length() > MAX_CHAT_BODY_UTF8_BYTES) {
        server.send(413, "text/plain; charset=utf-8",
                    "Message too long. Limit is 150 UTF-8 bytes.");
        return;
    }
    if (!isValidUtf8(msg)) {
        server.send(400, "text/plain; charset=utf-8", "Invalid UTF-8 message bytes.");
        return;
    }
    if (!radioTransmissionAllowed()) {
        server.send(503, "text/plain; charset=utf-8",
                    "Radio transmit disabled while charging or recovering from brownout.");
        return;
    }
    bool useRelay = server.hasArg("relay");
    bool overrideDest = false;
    uint8_t destAddh = 0;
    uint8_t destAddl = 0;
    parseOptionalSendDest(destAddh, destAddl, overrideDest);

    bool acked = sendMessage(msg, useRelay, destAddh, destAddl, overrideDest);
    if (acked) {
        server.send(200, "text/plain; charset=utf-8", "OK (ACK)");
    } else {
        server.send(504, "text/plain; charset=utf-8", "Timeout waiting ACK");
    }
}

// GET /gps — send one LoRa message with current GPS summary:
//   GPS,Stas,<sats>,<lat>,<lng>   ("Stas" = stats / satellite count field)
// Optional: same as /send — relay=1, to=AABB, addh=&addl=
void handleGpsSend() {
    if (!radioTransmissionAllowed()) {
        server.send(503, "text/plain; charset=utf-8",
                    "Radio transmit disabled while charging or recovering from brownout.");
        return;
    }
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }

    char buf[72];
    unsigned long sats = (unsigned long)gps.satellites.value();
    if (gps.location.isValid()) {
        snprintf(buf, sizeof(buf), "GPS,Stas,%lu,%.6f,%.6f", sats, gps.location.lat(), gps.location.lng());
    } else {
        snprintf(buf, sizeof(buf), "GPS,Stas,%lu,0,0", sats);
    }

    bool useRelay = server.hasArg("relay");
    bool overrideDest = false;
    uint8_t destAddh = 0;
    uint8_t destAddl = 0;
    parseOptionalSendDest(destAddh, destAddl, overrideDest);

    bool acked = sendMessage(String(buf), useRelay, destAddh, destAddl, overrideDest);
    if (acked) {
        server.send(200, "text/plain", String("OK (ACK) ") + buf);
    } else {
        server.send(504, "text/plain", String("Timeout waiting ACK — ") + buf);
    }
}

void handleStatusPage() {
    String html;
    html.reserve(4500);

    html += F("<!DOCTYPE html><html><head><meta charset=\"utf-8\">");
    html += F("<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">");
    html += F("<title>ESP32 & E22 Status</title>");
    html += F("<style>");
    html += F("body{font-family:Arial,'Noto Sans Khmer','Khmer OS','Khmer Sangam MN',sans-serif;margin:0;padding:20px;background:#1a1a2e;color:#eee;}");
    html += F("h1{color:#0af;}h2{color:#0f3;margin-top:28px;}a{color:#0af}");
    html += F(".card{background:#16213e;padding:20px;border-radius:8px;margin:0 auto;max-width:920px;box-shadow:0 4px 8px rgba(0,0,0,0.3);}");
    html += F("table{width:100%;border-collapse:collapse;margin-top:8px;}");
    html += F("th,td{padding:10px 8px;text-align:left;border-bottom:1px solid #333;vertical-align:top;}");
    html += F("th{width:42%;color:#aaa;font-weight:normal;}");
    html += F("td{color:#eee;font-family:monospace;font-size:14px;}");
    html += F(".muted{color:#888;font-size:13px;margin-top:12px;}");
    html += F("</style></head><body><div class=\"card\">");
    html += F("<p><a href=\"/\">← Control panel</a></p>");
    html += F("<h1>Device &amp; E22 status</h1>");

    html += F("<h2>ESP32</h2><table>");
    appendStatusRow(html, "Chip", String(ESP.getChipModel()));
    appendStatusRow(html, "SDK", String(ESP.getSdkVersion()));
    html += F("<tr><th>CPU MHz</th><td>");
    html += String(ESP.getCpuFreqMHz());
    html += F(" MHz</td></tr>");
    html += F("<tr><th>Flash</th><td>");
    html += String(ESP.getFlashChipSize() / 1024);
    html += F(" KB</td></tr>");
    html += F("<tr><th>Free heap</th><td>");
    html += String(ESP.getFreeHeap());
    html += F(" bytes</td></tr>");
    appendStatusRow(html, "Uptime", getUptime());
    appendStatusRow(html, "WiFi mode", String(WiFi.getMode() == WIFI_AP ? "AP" : "Other"));
    appendStatusRow(html, "callSign", ssid_AP);
    appendStatusRow(html, "AP SSID", ssid_AP);
    appendStatusRow(html, "AP IP", WiFi.softAPIP().toString());
    uint8_t mac[6];
    WiFi.softAPmacAddress(mac);
    char macBuf[18];
    snprintf(macBuf, sizeof(macBuf), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    appendStatusRow(html, "AP MAC", String(macBuf));
    appendStatusRow(html, "Node ready", String(nodeReady ? "yes" : "no"));
    appendStatusRow(html, "Status", statusMsg);
    appendStatusRow(html, "My address", "0x" + toHex2(MY_ADDH) + toHex2(MY_ADDL));
    appendStatusRow(html, "Target address", "0x" + toHex2(TARGET_ADDH) + toHex2(TARGET_ADDL));
    appendStatusRow(html, "Repeater address", "0x" + toHex2(REPEATER_ADDH) + toHex2(REPEATER_ADDL));
    appendStatusRow(html, "Network ID", "0x" + toHex2(NETWORK_ID));
    appendStatusRow(html, "Channel", "0x" + toHex2(CHANNEL));
    appendStatusRow(html, "CRYPT (hex)", "0x" + toHex4(CRYPT));
    appendStatusRow(html, "Use repeater (app)", USE_REPEATER ? "yes" : "no");
    appendStatusRow(html, "Battery", getBattery());
    appendStatusRow(html, "Charging", String(batteryCharging));
    appendStatusRow(html, "Power protection", brownoutRecoveryMode ? "brownout receive-only" :
                    (batteryCharging ? "charging receive-only" : "stable low TX power"));
    appendStatusRow(html, "Automatic beacon TX", PERIODIC_HELLO_ENABLED ? "enabled" : "disabled");
    html += F("</table>");

    html += F("<h2>E22 module (read from radio)</h2><table>");
    ResponseStructContainer c = e22.getConfiguration();
    if (c.status.code == 1) {
        Configuration cfg = *(Configuration*)c.data;
        appendStatusRow(html, "ADDH / ADDL", "0x" + toHex2(cfg.ADDH) + " / 0x" + toHex2(cfg.ADDL));
        appendStatusRow(html, "NETID", "0x" + toHex2(cfg.NETID));
        appendStatusRow(html, "CHAN", "0x" + toHex2(cfg.CHAN));
        html += F("<tr><th>UART baud (enum)</th><td>");
        html += String((uint8_t)cfg.SPED.uartBaudRate);
        html += F("</td></tr>");
        html += F("<tr><th>Air data rate (enum)</th><td>");
        html += String((uint8_t)cfg.SPED.airDataRate);
        html += F("</td></tr>");
        html += F("<tr><th>UART parity (enum)</th><td>");
        html += String((uint8_t)cfg.SPED.uartParity);
        html += F("</td></tr>");
        html += F("<tr><th>Sub-packet (enum)</th><td>");
        html += String((uint8_t)cfg.OPTION.subPacketSetting);
        html += F("</td></tr>");
        html += F("<tr><th>RSSI ambient noise (enum)</th><td>");
        html += String((uint8_t)cfg.OPTION.RSSIAmbientNoise);
        html += F("</td></tr>");
        html += F("<tr><th>TX power</th><td>");
        html += cfg.OPTION.getTransmissionPowerDescription();
        html += F(" (enum ");
        html += String((uint8_t)cfg.OPTION.transmissionPower);
        html += F(")</td></tr>");
        html += F("<tr><th>RSSI on wire (enum)</th><td>");
        html += String((uint8_t)cfg.TRANSMISSION_MODE.enableRSSI);
        html += F("</td></tr>");
        html += F("<tr><th>Last RX RSSI (dBm)</th><td>");
        html += String(lastRssiSignal);
        html += F("</td></tr>");
        html += F("<tr><th>Fixed transmission (enum)</th><td>");
        html += String((uint8_t)cfg.TRANSMISSION_MODE.fixedTransmission);
        html += F("</td></tr>");
        html += F("<tr><th>Repeater (enum)</th><td>");
        html += String((uint8_t)cfg.TRANSMISSION_MODE.enableRepeater);
        html += F("</td></tr>");
        html += F("<tr><th>LBT (enum)</th><td>");
        html += String((uint8_t)cfg.TRANSMISSION_MODE.enableLBT);
        html += F("</td></tr>");
        html += F("<tr><th>WOR transceiver (enum)</th><td>");
        html += String((uint8_t)cfg.TRANSMISSION_MODE.WORTransceiverControl);
        html += F("</td></tr>");
        html += F("<tr><th>WOR period (enum)</th><td>");
        html += String((uint8_t)cfg.TRANSMISSION_MODE.WORPeriod);
        html += F("</td></tr>");
    } else {
        html += F("<tr><td colspan=\"2\">getConfiguration failed (code ");
        html += String(c.status.code);
        html += F(")</td></tr>");
    }
    c.close();
    html += F("</table>");

    html += F("<p class=\"muted\">Machine-readable: <a href=\"/api/status\">/api/status</a></p>");
    html += F("</div></body></html>");

    server.send(200, "text/html", html);
}

void handleConfig() {
    String err;
    uint8_t myAddh, myAddl, targetAddh, targetAddl, repAddh, repAddl, netId, channel;
    uint16_t crypt;

    if (!parseConfigHex8("my_addh", myAddh, err) ||
        !parseConfigHex8("my_addl", myAddl, err) ||
        !parseConfigHex8("target_addh", targetAddh, err) ||
        !parseConfigHex8("target_addl", targetAddl, err) ||
        !parseConfigHex8("rep_addh", repAddh, err) ||
        !parseConfigHex8("rep_addl", repAddl, err) ||
        !parseConfigHex8("netid", netId, err) ||
        !parseConfigHex8("channel", channel, err) ||
        !parseConfigHex16("crypt", crypt, err)) {
        server.send(400, "text/plain", err);
        return;
    }

    MY_ADDH = myAddh;
    MY_ADDL = myAddl;
    TARGET_ADDH = targetAddh;
    TARGET_ADDL = targetAddl;
    REPEATER_ADDH = repAddh;
    REPEATER_ADDL = repAddl;
    NETWORK_ID = netId;
    CHANNEL = channel;
    CRYPT = crypt;
    USE_REPEATER = server.arg("use_repeater") == "1";

    saveConfig();

    server.send(200, "text/plain", "Configuration saved! Restarting...");

    delay(1000);
    ESP.restart();
}

void handleAPIStatus() {
    readBattery();
    String json = "{";

    // High-level runtime and node addressing info.
    json += "\"node\":{";
    json += "\"ready\":" + String(nodeReady ? "true" : "false") + ",";
    json += "\"status\":\"" + jsonEscape(statusMsg) + "\",";
    json += "\"callSign\":\"" + jsonEscape(ssid_AP) + "\",";
    json += "\"uptime\":\"" + jsonEscape(getUptime()) + "\",";
    json += "\"battery\":" + String(batteryPercent) + ",";
    json += "\"batteryVoltage\":" + String(batteryVoltage, 2) + ",";
    json += "\"charging\":\"" + jsonEscape(getCharging()) + "\",";
    json += "\"brownoutRecovery\":" + String(brownoutRecoveryMode ? "true" : "false") + ",";
    json += "\"txAllowed\":" + String(radioTransmissionAllowed() ? "true" : "false") + ",";
    json += "\"ip\":\"" + jsonEscape(WiFi.softAPIP().toString()) + "\",";
    json += "\"myAddr\":\"" + toHex2(MY_ADDH) + toHex2(MY_ADDL) + "\",";
    json += "\"targetAddr\":\"" + toHex2(TARGET_ADDH) + toHex2(TARGET_ADDL) + "\",";
    json += "\"repeaterAddr\":\"" + toHex2(REPEATER_ADDH) + toHex2(REPEATER_ADDL) + "\",";
    json += "\"netId\":\"" + toHex2(NETWORK_ID) + "\",";
    json += "\"channel\":\"" + toHex2(CHANNEL) + "\",";
    json += "\"crypt\":\"" + toHex4(CRYPT) + "\",";
    json += "\"useRepeater\":" + String(USE_REPEATER ? "true" : "false") + ",";
    json += "\"txPower\":" + String(E22_TX_POWER) + ",";
    json += "\"runtimeTxPower\":" + String(STABLE_E22_TX_POWER) + ",";
    json += "\"alertsBeep\":" + String(alertsBeepEnabled ? "true" : "false");
    json += "},";

    // Traffic counters and last payloads.
    json += "\"traffic\":{";
    json += "\"sent\":" + String(sentCount) + ",";
    json += "\"received\":" + String(receivedCount) + ",";
    json += "\"lastSent\":\"" + jsonEscape(lastSent) + "\",";
    json += "\"lastReceived\":\"" + jsonEscape(lastReceived) + "\"";
    json += "},";

    // Read back active E22 register-level config from the module.
    json += "\"e22\":{";
    ResponseStructContainer c = e22.getConfiguration();
    if (c.status.code == 1) {
        Configuration cfg = *(Configuration*)c.data;
        json += "\"ok\":true,";
        json += "\"addh\":\"" + toHex2(cfg.ADDH) + "\",";
        json += "\"addl\":\"" + toHex2(cfg.ADDL) + "\",";
        json += "\"netid\":\"" + toHex2(cfg.NETID) + "\",";
        json += "\"chan\":\"" + toHex2(cfg.CHAN) + "\",";
        json += "\"uartBaudRate\":" + String((uint8_t)cfg.SPED.uartBaudRate) + ",";
        json += "\"airDataRate\":" + String((uint8_t)cfg.SPED.airDataRate) + ",";
        json += "\"uartParity\":" + String((uint8_t)cfg.SPED.uartParity) + ",";
        json += "\"subPacketSetting\":" + String((uint8_t)cfg.OPTION.subPacketSetting) + ",";
        json += "\"rssiAmbientNoise\":" + String((uint8_t)cfg.OPTION.RSSIAmbientNoise) + ",";
        json += "\"transmissionPower\":" + String((uint8_t)cfg.OPTION.transmissionPower) + ",";
        json += "\"enableRSSI\":" + String((uint8_t)cfg.TRANSMISSION_MODE.enableRSSI) + ",";
        json += "\"lastSignalRssi\":" + String(lastRssiSignal) + ",";
        json += "\"fixedTransmission\":" + String((uint8_t)cfg.TRANSMISSION_MODE.fixedTransmission) + ",";
        json += "\"enableRepeater\":" + String((uint8_t)cfg.TRANSMISSION_MODE.enableRepeater) + ",";
        json += "\"enableLBT\":" + String((uint8_t)cfg.TRANSMISSION_MODE.enableLBT) + ",";
        json += "\"worTransceiverControl\":" + String((uint8_t)cfg.TRANSMISSION_MODE.WORTransceiverControl) + ",";
        json += "\"worPeriod\":" + String((uint8_t)cfg.TRANSMISSION_MODE.WORPeriod) + ",";
        json += "\"cryptH\":\"" + toHex2(cfg.CRYPT.CRYPT_H) + "\",";
        json += "\"cryptL\":\"" + toHex2(cfg.CRYPT.CRYPT_L) + "\"";
    } else {
        json += "\"ok\":false,";
        json += "\"error\":\"getConfiguration failed (" + String(c.status.code) + ")\"";
    }
    c.close();
    json += "}";

    json += "}";
    server.send(200, "application/json; charset=utf-8", json);
}

void handleAPILog() {
    String json = "[";
    for (int i = 0; i < MAX_LOG_ENTRIES; i++) {
        int idx = (logIndex - MAX_LOG_ENTRIES + i + MAX_LOG_ENTRIES) % MAX_LOG_ENTRIES;
        if (messageLog[idx].timestamp == 0) continue;
        
        if (json.length() > 1) json += ",";
        json += "{";
        json += "\"time\":\"" + String(messageLog[idx].timestamp / 1000) + "s\",";
        json += "\"dir\":\"" + jsonEscape(messageLog[idx].direction) + "\",";
        json += "\"rssi\":\"" + String(messageLog[idx].rssi) + "\",";
        json += "\"addr\":\"" + jsonEscape(messageLog[idx].address) + "\",";
        json += "\"msg\":\"" + jsonEscape(messageLog[idx].message) + "\"";
        json += "}";
    }
    json += "]";
    server.send(200, "application/json; charset=utf-8", json);
}

void handleAPINodes() {
    String json = "[";
    unsigned long now = millis();
    const bool myFix = gps.location.isValid();
    const double myLat = myFix ? gps.location.lat() : 0.0;
    const double myLng = myFix ? gps.location.lng() : 0.0;
    for (uint8_t i = 0; i < nearbyCount; i++) {
        if (json.length() > 1) json += ",";
        bool online = isNearbyNodeOnlineAt(i, now);
        json += "{";
        json += "\"addr\":\"" + toHex4(nearbyNodes[i].addr) + "\",";
        json += "\"netId\":\"" + toHex2(nearbyNodes[i].netId) + "\",";
        json += "\"channel\":\"" + toHex2(nearbyNodes[i].channel) + "\",";
        json += "\"rssi\":" + String(nearbyNodes[i].rssi) + ",";
        json += "\"lastSeenMs\":" + String(now - nearbyNodes[i].lastSeen) + ",";
        json += "\"repeater\":" + String(nearbyNodes[i].isRepeater ? "true" : "false") + ",";
        json += "\"online\":" + String(online ? "true" : "false") + ",";
        json += "\"callSign\":\"" + jsonEscape(nearbyNodes[i].callSign) + "\",";
        json += "\"gpsFix\":" + String(nearbyNodes[i].hasGpsFix ? "true" : "false") + ",";
        if (nearbyNodes[i].hasGpsFix) {
            json += "\"lat\":" + String(nearbyNodes[i].lat, 6) + ",";
            json += "\"lng\":" + String(nearbyNodes[i].lng, 6) + ",";
        }
        if (myFix && nearbyNodes[i].hasGpsFix) {
            double dm = haversineMeters(myLat, myLng, nearbyNodes[i].lat, nearbyNodes[i].lng);
            json += "\"distanceM\":" + String(dm, 1) + ",";
            json += "\"distanceKm\":" + String(dm / 1000.0, 3) + ",";
        } else {
            json += "\"distanceM\":0,";
            json += "\"distanceKm\":0,";
        }
        if (CRYPT != 0x0000) {
            json += "\"crypt\":\"enabled\"";
        } else {
            json += "\"crypt\":\"disabled\"";
        }
        json += "}";
    }
    json += "]";
    json = "{\"onlineCount\":" + String(countOnlineNearbyNodes(now)) +
           ",\"myGpsFix\":" + String(myFix ? "true" : "false") +
           ",\"myLat\":" + (myFix ? String(myLat, 6) : String("null")) +
           ",\"myLng\":" + (myFix ? String(myLng, 6) : String("null")) +
           ",\"nodes\":" + json + "}";
    server.send(200, "application/json; charset=utf-8", json);
}

void sendHelloBeacon() {
    if (!PERIODIC_HELLO_ENABLED || !radioTransmissionAllowed()) {
        return;
    }
    // Beacon format (backward compatible):
    // HELLO|AABB|NN|CC|R/N|callSign
    // HELLO|AABB|NN|CC|R/N|callSign|lat|lng   (if GPS fix)
    String payload = "HELLO|";
    payload += toHex4((MY_ADDH << 8) | MY_ADDL);
    payload += "|";
    payload += toHex2(NETWORK_ID);
    payload += "|";
    payload += toHex2(CHANNEL);
    payload += "|";
    payload += (USE_REPEATER ? "R" : "N");
    {
        String cs = ssid_AP;
        cs.replace("|", "");
        payload += "|";
        payload += cs;
    }

    if (gps.location.isValid()) {
        payload += "|";
        payload += String(gps.location.lat(), 6);
        payload += "|";
        payload += String(gps.location.lng(), 6);
    }

    // Best-effort broadcast (0xFFFF). Doesn't affect counters/log.
    e22.sendFixedMessage(0xFF, 0xFF, CHANNEL, payload.c_str());
}

// Clear configuration (erase stored NVS prefs for this namespace)
void ClearConfig() {
    preferences.begin("lora-config", false);
    preferences.clear();
    preferences.end();
    Serial.println("Configuration cleared!");
}

// ────────────────────────────────────────────────
// Read battery from BAT_READER (ADC), charging from BAT_CHARGING
// ────────────────────────────────────────────────
static float readBatteryVoltageInstant() {
  uint32_t rawSum = 0;
  uint32_t mvSum = 0;
  for (int i = 0; i < BAT_ADC_SAMPLES; i++) {
    rawSum += (uint32_t)analogRead(BAT_READER);
    mvSum += (uint32_t)analogReadMilliVolts(BAT_READER);
    delayMicroseconds(200);
  }
  float mvAvg = (float)mvSum / (float)BAT_ADC_SAMPLES;
  if (mvAvg <= 0.0f) {
    float rawAvg = (float)rawSum / (float)BAT_ADC_SAMPLES;
    mvAvg = rawAvg * BAT_ADC_VREF_MV / BAT_ADC_MAX;
  }
  float adcPinV = mvAvg / 1000.0f;
  return min(adcPinV * BAT_DIVIDER_RATIO, BAT_VOLTAGE_MAX);
}

static float filterBatteryVoltage(float vInstant) {
  static bool battInit = false;
  static float battFilt = 0.0f;
  const float alpha = 0.15f;
  if (!battInit) {
    battFilt = vInstant;
    battInit = true;
  } else {
    battFilt += alpha * (vInstant - battFilt);
  }
  return battFilt;
}

static int batteryPercentFromVoltage(float v) {
  struct BatteryPoint {
    float voltage;
    int percent;
  };

  static const BatteryPoint curve[] = {
    {3.30f, 0},
    {3.50f, 10},
    {3.60f, 20},
    {3.68f, 30},
    {3.73f, 40},
    {3.79f, 50},
    {3.85f, 60},
    {3.92f, 70},
    {4.00f, 80},
    {4.10f, 90},
    {4.20f, 100},
  };

  const size_t pointCount = sizeof(curve) / sizeof(curve[0]);
  if (v <= curve[0].voltage) return curve[0].percent;
  if (v >= curve[pointCount - 1].voltage) return curve[pointCount - 1].percent;

  for (size_t i = 1; i < pointCount; i++) {
    if (v <= curve[i].voltage) {
      const BatteryPoint &low = curve[i - 1];
      const BatteryPoint &high = curve[i];
      float pct = (float)low.percent +
                  (v - low.voltage) * (float)(high.percent - low.percent) /
                  (high.voltage - low.voltage);
      return constrain((int)(pct + 0.5f), 0, 100);
    }
  }

  return 0;
}

void readBattery() {
  float vInstant = readBatteryVoltageInstant();
  batteryVoltage = filterBatteryVoltage(vInstant);
  batteryPercent = batteryPercentFromVoltage(batteryVoltage);

  static bool chargeSignalActive = false;
  static unsigned long chargeActiveSince = 0;
  static unsigned long chargeInactiveSince = 0;
  static bool chargeDebounceInit = false;
  bool rawNow = (digitalRead(BAT_CHARGING) == BAT_CHARGING_ACTIVE_LEVEL);
  unsigned long now = millis();
  bool wasCharging = batteryCharging;

  if (!chargeDebounceInit) {
    chargeDebounceInit = true;
    chargeActiveSince = now;
    chargeInactiveSince = now;
    chargeSignalActive = rawNow;

    // Show charging immediately when USB/charging power is already present at boot.
    if (rawNow) {
      batteryCharging = true;
    }
  }

  if (rawNow) {
    if (!chargeSignalActive) {
      chargeSignalActive = true;
      chargeActiveSince = now;
    }
    if (!batteryCharging && (now - chargeActiveSince) >= CHARGE_PIN_DEBOUNCE_MS) {
      lastBatteryCharging = batteryCharging;
      batteryCharging = true;
    }
  } else {
    if (chargeSignalActive) {
      chargeSignalActive = false;
      chargeInactiveSince = now;
    }
    if (batteryCharging && (now - chargeInactiveSince) >= CHARGE_PIN_RELEASE_MS) {
      lastBatteryCharging = batteryCharging;
      batteryCharging = false;
      chargeScreenUiReady = false;
    }
  }

  if (batteryCharging == wasCharging) {
    return;
  }

  if (batteryCharging) {
    if (nodeReady) statusMsg = "CHARGING RX ONLY";
    chargeScreenSuppressed = false;
    chargeScreenUiReady = false;
    if (oledDisplayOff) {
      oledHardwarePower(true);
      oledDisplayOff = false;
      applyOledBrightness();
    }
    noteDisplayActivity();
    drawCurrentScreen();
  } else if (wasCharging) {
    if (nodeReady && !brownoutRecoveryMode) statusMsg = USE_REPEATER ? "REPEATER" : "READY";
    noteDisplayActivity();
    drawCurrentScreen();
  }
}

String getBattery() {
  char buf[40];
  snprintf(buf, sizeof(buf), "%.2fV %d%%", batteryVoltage, batteryPercent);
  return String(buf);
}

String getCharging() {
  return batteryCharging ? "true" : "false";
}

String getCurrentTime() {
  unsigned long totalSeconds = (millis() - startTime) / 1000;
  char buf[8];
  snprintf(buf, sizeof(buf), "%02d:%02d", (int)((totalSeconds / 3600) % 24), (int)((totalSeconds / 60) % 60));
  return String(buf);
}

String getGPSMessage() {
  while (gpsSerial.available() > 0) gps.encode(gpsSerial.read());
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
    char buf[80];
    snprintf(buf, sizeof(buf), "T:%lus Lat:%.5f Lng:%.5f S:%02lu",
             (unsigned long)(millis() / 1000), gps.location.lat(), gps.location.lng(),
             (unsigned long)gps.satellites.value());
    return String(buf);
  }
  char buf[64];
  snprintf(buf, sizeof(buf), "No fix  Sats:%02lu  T:%lus",
           (unsigned long)gps.satellites.value(), (unsigned long)(millis() / 1000));
  return String(buf);
}

// ────────────────────────────────────────────────
// OLED layout helpers (1px font = ~6px wide per character)
// ────────────────────────────────────────────────
static void drawScreenDivider() {
  // display.drawFastHLine(0, 14, SCREEN_WIDTH, SSD1306_WHITE);
}

static String ellipsizeMaxChars(const String &s, size_t maxChars) {
  if ((size_t)s.length() <= maxChars) return s;
  if (maxChars <= 1) return String(".");
  return s.substring(0, maxChars - 1) + ".";
}

// Small “signal bars” glyph used as a nodes / mesh hint (replaces opaque control chars).
static void drawTinySignalGlyph(int x, int y) {
  display.drawFastVLine(x, y + 4, 3, SSD1306_WHITE);
  display.drawFastVLine(x + 2, y + 2, 5, SSD1306_WHITE);
  display.drawFastVLine(x + 4, y, 7, SSD1306_WHITE);
}

static void drawRssiProgressBar(int x, int y, int w, int h, int signalPct) {
  const int fillW = ((w - 2) * constrain(signalPct, 0, 100)) / 100;

  display.drawRect(x, y, w, h, SSD1306_WHITE);
  if (fillW > 0) {
    display.fillRect(x + 1, y + 1, fillW, h - 2, SSD1306_WHITE);
  }
}3

// Bottom page dots: active filled, others hollow for clearer hierarchy.
static void drawScreenPageDots() {
  const int n = NUM_SCREENS;
  const int cy = 61;
  const int spacing = 10;
  const int totalW = (n - 1) * spacing;
  const int startX = (SCREEN_WIDTH - totalW) / 2;
  for (int i = 0; i < n; i++) {
    const int cx = startX + i * spacing;
    if (i == currentScreen) {
      display.fillCircle(cx, cy, 2, SSD1306_WHITE);
    } else {
      display.drawCircle(cx, cy, 1, SSD1306_WHITE);
    }
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

  int batX = 6;   // left position of the battery body
  int batY = 2;   // top position of the whole battery (including nub)
  // Nub
  display.fillRect(batX + 1, batY - 1, 3, 1, SSD1306_BLACK);

  // Main body outline
  display.drawRect(batX - 1, batY, 7, 9, SSD1306_BLACK);

  // Fill (4 pixels usable height inside) – refreshed from BAT_READER
  int bars = map(batteryPercent, 0, 100, 0, 4);
  if (bars > 0) {
    display.fillRect(batX + 1, batY + 7 - bars, 3, bars, SSD1306_WHITE);
  }

  // Charging: lightning bolt inside battery when BAT_CHARGING is HIGH
  if (batteryCharging) {
    display.drawPixel(batX + 3, batY + 2, SSD1306_BLACK);
    display.drawPixel(batX + 2, batY + 3, SSD1306_BLACK);
    display.drawPixel(batX + 2, batY + 5, SSD1306_BLACK);
    display.drawPixel(batX + 1, batY + 4, SSD1306_BLACK);
    display.drawPixel(batX + 3, batY + 4, SSD1306_BLACK);
    display.drawPixel(batX + 1, batY + 6, SSD1306_BLACK);
  }

  // Percentage next to icon (refreshed from BAT_READER)
  display.setCursor(batX + 7, 3);
  display.print(batteryPercent);
  display.print("%");
  // if (batteryCharging) {
  //   display.print("+");  // charging indicator
  // }

  // Time on the right
  display.setCursor(92, 3);
  display.print(getCurrentTime());
  
  // Reset text color for rest of screen
  display.setTextColor(SSD1306_WHITE);
}

// ────────────────────────────────────────────────
// OLED drawing - Meshtastic-style layout
// ────────────────────────────────────────────────

// Screen 1: Status screen
// ────────────────────────────────────────────────
void drawStatusScreen() {
  display.clearDisplay();
  
  // Draw header with rounded corners
  drawHeader();
  drawScreenDivider();
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Same source as /api/nodes: registry + ONLINE_NODE_TTL_MS
  syncOnlineNodesFromRegistry();
  
  drawTinySignalGlyph(4, 16);
  display.setCursor(12, 16);
  display.print(onlineNodes);
  display.print(" nodes");
  display.setCursor(72, 16);
  display.print("Up ");
  display.print(getUptime());
  
  display.setCursor(4, 26);
  display.print("GPS ");
  display.print(gps.satellites.value());
  display.print(" sats");
  display.setCursor(82, 26);
  display.print(batteryVoltage, 2);
  display.print(" V");
  
  const int signalPct = rssiDbmToSignalPercent(lastRssiSignal);
  display.setCursor(4, 37);
  display.print("RSSI");
  drawRssiProgressBar(34, 37, 48, 7, signalPct);
  display.setCursor(84, 37);
  if (lastRssiSignal <= -250) {
    display.print("--dBm");
  } else {
    display.print(lastRssiSignal);
    display.print("dBm");
  }
  
  {
    String line = ellipsizeMaxChars(ssid_AP, 16);
    line += "  DBR";
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(line.c_str(), 0, 0, &x1, &y1, &w, &h);
    int cx = (SCREEN_WIDTH - (int)w) / 2;
    if (cx < 0) cx = 0;
    display.setCursor(cx, 50);
    display.print(line);
  }

  drawScreenPageDots();
  display.display();
}

// ────────────────────────────────────────────────
// Screen 2: WiFi info
// ────────────────────────────────────────────────
void drawWifiScreen() {
  display.clearDisplay();
  drawHeader();
  drawScreenDivider();
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // display.setCursor(4, 16);
  // display.print("Hotspot");
  
  // Row 1: Wi-Fi SSID with signal icon
  const int wifiIconX = 4;
  const int wifiIconY = 22;
  display.drawLine(wifiIconX,     wifiIconY + 6, wifiIconX + 3, wifiIconY + 3, SSD1306_WHITE);
  display.drawLine(wifiIconX + 3, wifiIconY + 3, wifiIconX + 6, wifiIconY + 6, SSD1306_WHITE);
  display.drawPixel(wifiIconX + 3, wifiIconY + 7, SSD1306_WHITE);
  display.setCursor(16, 22);
  display.print("SSID ");
  display.print(ellipsizeMaxChars(ssid_AP, 14));
  
  const int keyX = 4;
  const int keyY = 34;
  display.drawCircle(keyX + 2, keyY + 2, 2, SSD1306_WHITE);
  display.drawLine(keyX + 4, keyY + 2, keyX + 8, keyY + 2, SSD1306_WHITE);
  display.drawPixel(keyX + 8, keyY + 1, SSD1306_WHITE);
  display.drawPixel(keyX + 8, keyY + 3, SSD1306_WHITE);
  display.setCursor(16, 34);
  display.print("PASS ");
  display.print(ellipsizeMaxChars(password_AP, 14));
  
  const int ipIconX = 4;
  const int ipIconY = 46;
  display.drawRect(ipIconX, ipIconY, 9, 6, SSD1306_WHITE);
  display.drawFastHLine(ipIconX + 2, ipIconY + 7, 5, SSD1306_WHITE);
  display.setCursor(16, 46);
  display.print("IP ");
  display.print(WiFi.softAPIP().toString());

  drawScreenPageDots();
  display.display();
}

// ────────────────────────────────────────────────
// Screen 3: GPS info
// ────────────────────────────────────────────────
static void drawNavigationCompass(double bearingDeg, bool bearingValid) {
  // Small “navigation compass” on the right side.
  // Note: SSD1306 coordinates use +Y downward; we want 0deg (north) to point up.
  const int cx = 118;
  const int cy = 34;
  const int r = 9;

  display.drawCircle(cx, cy, r, SSD1306_WHITE);
  display.drawPixel(cx, cy, SSD1306_WHITE);

  // Compass arrow
  if (bearingValid) {
    // Normalize to [0, 360)
    while (bearingDeg < 0) bearingDeg += 360.0;
    while (bearingDeg >= 360.0) bearingDeg -= 360.0;

    const double rad = deg2rad(bearingDeg);
    const int tipX = cx + (int)(sin(rad) * r);
    const int tipY = cy - (int)(cos(rad) * r);
    const int tipXc = constrain(tipX, 0, SCREEN_WIDTH - 1);
    const int tipYc = constrain(tipY, 0, SCREEN_HEIGHT - 1);

    display.drawLine(cx, cy, tipXc, tipYc, SSD1306_WHITE);

    // Simple arrow head
    const double headHalfAngle = deg2rad(20.0);
    const int headLen = 4;
    const double a1 = rad + PI + headHalfAngle;
    const double a2 = rad + PI - headHalfAngle;

    const int x1 = tipXc + (int)(sin(a1) * headLen);
    const int y1 = tipYc - (int)(cos(a1) * headLen);
    const int x2 = tipXc + (int)(sin(a2) * headLen);
    const int y2 = tipYc - (int)(cos(a2) * headLen);
    const int x1c = constrain(x1, 0, SCREEN_WIDTH - 1);
    const int y1c = constrain(y1, 0, SCREEN_HEIGHT - 1);
    const int x2c = constrain(x2, 0, SCREEN_WIDTH - 1);
    const int y2c = constrain(y2, 0, SCREEN_HEIGHT - 1);

    display.drawLine(tipXc, tipYc, x1c, y1c, SSD1306_WHITE);
    display.drawLine(tipXc, tipYc, x2c, y2c, SSD1306_WHITE);

    // Cardinal letter + bearing degrees
    const char dirLetters[] = {'N', 'E', 'S', 'W'};
    const int dirIdx = ((int)((bearingDeg + 45.0) / 90.0)) % 4;
    display.setCursor(86, 54);
    display.print(dirLetters[dirIdx]);
    display.print(' ');
    display.print((int)bearingDeg);
  } else {
    // No course: draw an X inside the compass.
    display.drawLine(cx - r, cy - r, cx + r, cy + r, SSD1306_WHITE);
    display.drawLine(cx + r, cy - r, cx - r, cy + r, SSD1306_WHITE);
  }
}

void drawGpsScreen() {
  getGPSMessage();
  display.clearDisplay();
  drawHeader();
  drawScreenDivider();
  
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(4, 16);
  display.print("GPS ");
  display.print(gps.satellites.value());
  display.print(" sv");

  // Right side navigation compass (uses GPS course/track angle).
  const bool courseValid = gps.course.isValid();
  const double courseDeg = courseValid ? gps.course.deg() : 0.0;
  drawNavigationCompass(courseDeg, courseValid);
  
  if (gps.location.isValid()) {
    display.setCursor(4, 26);
    display.print("Lat ");
    display.print(gps.location.lat(), 5);
    display.setCursor(4, 36);
    display.print("Lng ");
    display.print(gps.location.lng(), 5);
    if (gps.altitude.isValid()) {
      display.setCursor(4, 46);
      display.print("Alt ");
      display.print((int)gps.altitude.meters());
      display.print(" m");
    }
  } else {
    display.setCursor(4, 28);
    display.print("No fix yet");
    display.setCursor(4, 40);
    display.print("In view ");
    display.print(gps.satellites.value());
  }

  drawScreenPageDots();
  display.display();
}

// ────────────────────────────────────────────────
// Screen 4: Display settings — U/D row, SEL toggles L/R adjust, hold SEL saves
// Layout tuned for 128x64 (e.g. 1.54" SSD1306): airy cards, soft radius, clear hierarchy
// ────────────────────────────────────────────────
static void drawSettingsRow(int x, int w, int rowTop, int rowH, bool focused, bool adjusting,
                            const __FlashStringHelper *label, const char *valueStr) {
  const int r = 4;
  const int textY = rowTop + 2;

  if (adjusting) {
    display.fillRoundRect(x, rowTop, w, rowH, r, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.setTextColor(SSD1306_WHITE);
    if (focused) {
      display.fillRoundRect(x + 2, rowTop + 3, 3, rowH - 6, 1, SSD1306_WHITE);
    }
  }

  display.setCursor(x + (focused && !adjusting ? 10 : 8), textY);
  display.print(label);

  int16_t bx, by;
  uint16_t bw, bh;
  display.getTextBounds(valueStr, 0, 0, &bx, &by, &bw, &bh);
  display.setCursor(x + w - 8 - (int)bw, textY);
  display.print(valueStr);
}

void drawSettingsScreen() {
  display.clearDisplay();
  drawHeader();
  drawScreenDivider();

  const int mx = 8;
  const int cardW = SCREEN_WIDTH - 2 * mx;
  const int cardX = mx;
  const int rowH = 10;
  const int rowGap = 1;
  const int row1Top = 25;
  const int row2Top = row1Top + rowH + rowGap;
  const int row3Top = row2Top + rowH + rowGap;

  display.setTextSize(1);

  // Title row: light weight, unsaved = soft dot.
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(mx, 16);
  display.print(F("Display"));
  if (settingsDirty && (long)(settingsSavedUntilMs - millis()) <= 0) {
    display.fillCircle(118, 19, 2, SSD1306_WHITE);
  }

  char sleepStr[8];
  snprintf(sleepStr, sizeof(sleepStr), "%s", oledSleepLabel(oledSleepMode));
  drawSettingsRow(cardX, cardW, row1Top, rowH, settingsEditField == 0,
                  settingsAdjusting && settingsEditField == 0, F("Sleep"), sleepStr);

  char pctStr[8];
  snprintf(pctStr, sizeof(pctStr), "%u%%", (unsigned)oledBrightPct);
  drawSettingsRow(cardX, cardW, row2Top, rowH, settingsEditField == 1,
                  settingsAdjusting && settingsEditField == 1, F("Brightness"), pctStr);

  const char *alertVal = alertsBeepEnabled ? "ON" : "OFF";
  drawSettingsRow(cardX, cardW, row3Top, rowH, settingsEditField == 2,
                  settingsAdjusting && settingsEditField == 2, F("Alerts"), alertVal);

  drawScreenPageDots();

  if ((long)(settingsSavedUntilMs - millis()) > 0) {
    const char *savedText = "SAVED !";
    const int popupW = 68;
    const int popupH = 24;
    const int popupX = (SCREEN_WIDTH - popupW) / 2;
    const int popupY = 25;
    int16_t bx, by;
    uint16_t bw, bh;

    display.fillRoundRect(popupX, popupY, popupW, popupH, 4, SSD1306_WHITE);
    display.drawRoundRect(popupX, popupY, popupW, popupH, 4, SSD1306_BLACK);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.setTextSize(1);
    display.getTextBounds(savedText, 0, 0, &bx, &by, &bw, &bh);
    display.setCursor(popupX + (popupW - (int)bw) / 2,
                      popupY + (popupH - (int)bh) / 2);
    display.print(savedText);
  }

  display.display();
}

// ────────────────────────────────────────────────
// Screen 5: Charging screen (partial updates to avoid full-screen blink)
// ────────────────────────────────────────────────
static void drawChargingTitle(unsigned ndots) {
  char titleLine[16];
  int n = snprintf(titleLine, sizeof(titleLine), "Charging");
  if (n < 0) n = 0;
  for (unsigned i = 0; i < ndots && n < (int)sizeof(titleLine) - 1; i++)
    titleLine[n++] = '.';
  titleLine[n] = '\0';

  display.fillRect(0, 15, SCREEN_WIDTH, 12, SSD1306_BLACK);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  int16_t tx, ty;
  uint16_t tw, th;
  display.getTextBounds(titleLine, 0, 0, &tx, &ty, &tw, &th);
  display.setCursor((SCREEN_WIDTH - (int)tw) / 2, 17);
  display.print(titleLine);
}

static void drawChargingBatteryAnim(int inX, int inY, int inW, int inH, int fillW, int marchHx) {
  display.fillRect(inX, inY, inW, inH, SSD1306_BLACK);
  if (fillW > 0) {
    display.fillRect(inX, inY, fillW, inH, SSD1306_WHITE);
    if (fillW >= 4 && marchHx >= inX && marchHx < inX + fillW) {
      display.drawFastVLine(marchHx, inY, inH, SSD1306_BLACK);
    }
  }
}

void drawChargingScreen(bool fullRedraw) {
  static unsigned lastDots = 99;
  static int lastFillW = -1;
  static int lastMarchHx = -999;

  const int batX = 46;
  const int batY = 30;
  const int bodyW = 30;
  const int bodyH = 12;
  const int inX = batX + 2;
  const int inY = batY + 2;
  const int inW = bodyW - 4;
  const int inH = bodyH - 4;

  const unsigned ndots = (unsigned)(millis() / 500UL) % 4U;

  const unsigned long breathMs = 4400UL;
  unsigned long ph = millis() % breathMs;
  int fillW;
  if (ph < breathMs / 2)
    fillW = (int)((unsigned long)inW * ph / (breathMs / 2));
  else
    fillW = (int)((unsigned long)inW * (breathMs - ph) / (breathMs / 2));
  fillW = constrain(fillW, 1, inW);

  int marchHx = -1;
  if (fillW >= 4) {
    const int nSeg = 5;
    int segPx = max(1, fillW / nSeg);
    int march = (int)((millis() / 450UL) % (unsigned long)(nSeg + 1));
    marchHx = constrain(inX + march * segPx, inX, inX + fillW - 2);
  }

  bool needDisplay = false;

  if (fullRedraw || !chargeScreenUiReady) {
    display.clearDisplay();
    // drawHeader();
    // drawScreenDivider();
    drawChargingTitle(ndots);
    display.drawRoundRect(batX, batY, bodyW, bodyH, 2, SSD1306_WHITE);
    display.fillRect(batX + bodyW, batY + 4, 2, 4, SSD1306_WHITE);
    drawChargingBatteryAnim(inX, inY, inW, inH, fillW, marchHx);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(4, 50);
    // display.print(F("Hold RIGHT 3s: Home"));
    chargeScreenUiReady = true;
    lastDots = ndots;
    lastFillW = fillW;
    lastMarchHx = marchHx;
    needDisplay = true;
  } else {
    if (ndots != lastDots) {
      drawChargingTitle(ndots);
      lastDots = ndots;
      needDisplay = true;
    }
    if (fillW != lastFillW || marchHx != lastMarchHx) {
      drawChargingBatteryAnim(inX, inY, inW, inH, fillW, marchHx);
      lastFillW = fillW;
      lastMarchHx = marchHx;
      needDisplay = true;
    }
  }

  if (needDisplay) {
    display.display();
  }
}

// ────────────────────────────────────────────────
// Draw current screen (router)
// ────────────────────────────────────────────────
void drawCurrentScreen() {
  if (!oledReady) return;
  if (oledDisplayOff) {
    if (batteryCharging && !chargeScreenSuppressed) {
      oledHardwarePower(true);
      oledDisplayOff = false;
      applyOledBrightness();
    } else {
      return;
    }
  }
  if (batteryCharging && !chargeScreenSuppressed) {
    drawChargingScreen(!chargeScreenUiReady);
  } else {
    chargeScreenUiReady = false;
    if (currentScreen == 0) {
      drawStatusScreen();
    } else if (currentScreen == 1) {
      drawWifiScreen();
    } else if (currentScreen == 2) {
      drawGpsScreen();
    } else {
      drawSettingsScreen();
    }
  }
}

void showSent(const String &message, const String &status) {
  if (!oledReady) return;
  (void)status;
  display.clearDisplay();
  drawHeader();
  drawScreenDivider();

  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  {
    const char *t = "SENT";
    int16_t bx, by;
    uint16_t bw, bh;
    display.getTextBounds(t, 0, 0, &bx, &by, &bw, &bh);
    display.setCursor((SCREEN_WIDTH - (int)bw) / 2, 22);
    display.print(t);
  }

  display.setTextSize(1);
  String line1 = message.substring(0, 21);
  String line2 = "";
  if (message.length() > 21) {
    line2 = message.substring(21, min((int)message.length(), 42));
    if (message.length() > 42) line2 += ".";
  }

  display.setCursor(4, 42);
  display.print(line1);
  if (line2.length() > 0) {
    display.setCursor(4, 52);
    display.print(line2);
  }

  display.display();
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== LoRa Node with Dynamic Config ===");
    esp_reset_reason_t resetReason = esp_reset_reason();
    brownoutRecoveryMode = (resetReason == ESP_RST_BROWNOUT);
    Serial.printf("[BOOT] Reset reason: %d%s\n", (int)resetReason,
                  brownoutRecoveryMode ? " (brownout protection enabled)" : "");

    startTime = millis();
    // Clear cache
    // ClearConfig();
    delay(1000);

    // Load saved configuration
    loadConfig();

    // Charging must be known before WiFi/radio setup so boot can remain
    // receive-only and select the charging screen on an attached charger.
    pinMode(BAT_READER, INPUT);
    pinMode(BAT_CHARGING, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(BAT_READER, ADC_11db);
    readBattery();

    pinMode(PIN_AUX, INPUT);
    pinMode(PIN_M0,  OUTPUT);
    pinMode(PIN_M1,  OUTPUT);

    // Initialize WiFi AP
    WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    WiFi.softAP(ssid_AP, password_AP,1,0,1);
    Serial.print("AP: ");
    Serial.println(ssid_AP);
    Serial.print("IP: ");
    Serial.println(WiFi.softAPIP());

    // Setup web server
    server.on("/", handleRoot);
    server.on("/setup", HTTP_GET, handleSetupPage);
    server.on("/setup/save", HTTP_POST, handleSetupSave);
    server.on("/setPower", HTTP_GET, handleSetPowerPage);
    server.on("/setPower/save", HTTP_POST, handleSetPowerSave);
    server.on("/send", HTTP_ANY, handleSend);
    server.on("/gps", handleGpsSend);
    server.on("/config", handleConfig);
    server.on("/status", handleStatusPage);
    server.on("/api/status", handleAPIStatus);
    server.on("/api/log", handleAPILog);
    server.on("/api/nodes", handleAPINodes);
    server.begin();
    Serial.println("Web server started");

    // Initialize GPS
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("GPS UART1 started @ 9600 baud on pins 25(RX),26(TX)");
    
    // Initialize OLED first and show boot status before LoRa init.
    // This fixes the blank-screen case where E22/config fails before the first draw.
    initOled();
    showSplashScreen();
    noteDisplayActivity();
    // showBootMessage("LoRa Node", "OLED starting...");

    // Initialize E22
    bool e22Started = e22.begin();
    if (!e22Started) {
        Serial.println("[E22] FAILED");
        statusMsg = "E22 FAIL";
        showBootMessage("E22 FAILED", "Check LoRa pins");
        // Do not block forever; keep OLED/Web UI alive so the error is visible.
    }

    // Apply configuration
    if (e22Started && applyNodeConfig()) {
        nodeReady = true;
        statusMsg = radioTransmissionAllowed() ? (USE_REPEATER ? "REPEATER" : "READY") :
                    (batteryCharging ? "CHARGING RX ONLY" : "BROWNOUT RX ONLY");
        Serial.printf("[READY] Node 0x%02X%02X active (Net: 0x%02X)\n", 
            MY_ADDH, MY_ADDL, NETWORK_ID);
    } else if (e22Started) {
        statusMsg = "CFG FAIL";
        showBootMessage("CONFIG FAILED", "Open Web setup");
    }

    // Initialize Buzzer
    pinMode(BUZZER_ACTIVE, OUTPUT);
    digitalWrite(BUZZER_ACTIVE, LOW);

    pinMode(BTN_LEFT_PIN, INPUT_PULLUP);
    pinMode(BTN_RIGHT_PIN, INPUT_PULLUP);

    pinMode(BTN_UP_PIN, INPUT_PULLUP);
    pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
    pinMode(BTN_SELECT_PIN, INPUT_PULLUP);

    readBattery();
    // lastSyncBroadcastMs = millis();
    drawCurrentScreen();
    // drawDisplay();
}

// =============================================================================
// Loop
// =============================================================================

void loop() {
  static unsigned long lastUiRefresh = 0;

  server.handleClient();

  // Update GPS early so beacons/API have freshest fix.
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  checkOledSleepTimeout();

  if (nodeReady) {
      checkIncoming();
  }

  // Periodic discovery beacon to help UIs list nearby nodes
  if (nodeReady && PERIODIC_HELLO_ENABLED && radioTransmissionAllowed() &&
      (millis() - lastBeaconAt >= 5000)) {
      lastBeaconAt = millis();
      sendHelloBeacon();
  }

  // Buttons (5-way: Left, Right, Up, Down, Select — all active-low)
  bool leftLow = (digitalRead(BTN_LEFT_PIN) == LOW);
  bool rightLow = (digitalRead(BTN_RIGHT_PIN) == LOW);
  bool upLow = (digitalRead(BTN_UP_PIN) == LOW);
  bool downLow = (digitalRead(BTN_DOWN_PIN) == LOW);
  bool selectLow = (digitalRead(BTN_SELECT_PIN) == LOW);
  bool anyNavLow = leftLow || rightLow || upLow || downLow || selectLow;

  if (oledIgnoreButtonsUntilRelease) {
    if (!anyNavLow) oledIgnoreButtonsUntilRelease = false;
  } else if (oledDisplayOff && anyNavLow) {
    tryWakeOledFromSleep();
    oledIgnoreButtonsUntilRelease = true;
    noteDisplayActivity();
  } else {
    static bool leftBtnHeld = false;
    static unsigned long leftBtnDownMs = 0;

    if (leftLow) {
      if (!leftBtnHeld) {
        leftBtnHeld = true;
        leftBtnDownMs = millis();
      }
    } else if (leftBtnHeld) {
      unsigned long heldMs = millis() - leftBtnDownMs;
      if (heldMs > 20 && (millis() - lastLeftBtnPress) > 300) {
        lastLeftBtnPress = millis();
        noteDisplayActivity();
        popupType = PopupType::None;
        if (currentScreen == 3 && settingsAdjusting) {
          previewSettingsAdjust(-1);
        } else if (currentScreen == 3) {
          commitOledSettingsIfDirty();
          settingsAdjusting = false;
          currentScreen = (currentScreen + NUM_SCREENS - 1) % NUM_SCREENS;
        } else {
          currentScreen = (currentScreen + NUM_SCREENS - 1) % NUM_SCREENS;
        }
        drawCurrentScreen();
      }
      leftBtnHeld = false;
    }

    if (rightLow) {
      if (!rightBtnHeld) {
        rightBtnHeld = true;
        rightBtnLongSent = false;
        rightBtnDownMs = millis();
      } else if (!rightBtnLongSent && (millis() - rightBtnDownMs >= BTN_RIGHT_LONG_PRESS_MS)) {
        rightBtnLongSent = true;
        chargeScreenSuppressed = true;
        chargeScreenUiReady = false;
        if (currentScreen == 3) {
          commitOledSettingsIfDirty();
        }
        currentScreen = 0;
        settingsAdjusting = false;
        popupType = PopupType::None;
        noteDisplayActivity();
        drawCurrentScreen();
      }
    } else if (rightBtnHeld) {
      unsigned long heldMs = millis() - rightBtnDownMs;
      if (!rightBtnLongSent && heldMs > 20 && (millis() - lastRightBtnPress) > 300) {
        lastRightBtnPress = millis();
        noteDisplayActivity();
        popupType = PopupType::None;
        if (currentScreen == 3 && settingsAdjusting) {
          previewSettingsAdjust(1);
        } else if (currentScreen == 3) {
          commitOledSettingsIfDirty();
          settingsAdjusting = false;
          currentScreen = (currentScreen + 1) % NUM_SCREENS;
        } else {
          currentScreen = (currentScreen + 1) % NUM_SCREENS;
        }
        drawCurrentScreen();
      }
      rightBtnHeld = false;
    }

    static bool upHeld = false;
    static unsigned long upDownMs = 0;
    if (upLow) {
      if (!upHeld) {
        upHeld = true;
        upDownMs = millis();
      }
    } else if (upHeld) {
      unsigned long heldMs = millis() - upDownMs;
      if (heldMs > 20 && (millis() - lastUpBtnPress) > 300 && currentScreen == 3) {
        lastUpBtnPress = millis();
        noteDisplayActivity();
        popupType = PopupType::None;
        commitOledSettingsIfDirty();
        settingsAdjusting = false;
        settingsEditField = (uint8_t)((settingsEditField + 3 - 1) % 3);
        drawCurrentScreen();
      }
      upHeld = false;
    }

    static bool downHeld = false;
    static unsigned long downDownMs = 0;
    if (downLow) {
      if (!downHeld) {
        downHeld = true;
        downDownMs = millis();
      }
    } else if (downHeld) {
      unsigned long heldMs = millis() - downDownMs;
      if (heldMs > 20 && (millis() - lastDownBtnPress) > 300 && currentScreen == 3) {
        lastDownBtnPress = millis();
        noteDisplayActivity();
        popupType = PopupType::None;
        commitOledSettingsIfDirty();
        settingsAdjusting = false;
        settingsEditField = (uint8_t)((settingsEditField + 1) % 3);
        drawCurrentScreen();
      }
      downHeld = false;
    }

    static bool selectBtnHeld = false;
    static unsigned long selectBtnDownMs = 0;
    static bool selectHoldSaveDone = false;
    if (selectLow) {
      if (!selectBtnHeld) {
        selectBtnHeld = true;
        selectBtnDownMs = millis();
        selectHoldSaveDone = false;
      } else if (!selectHoldSaveDone && currentScreen == 3 &&
                 (millis() - selectBtnDownMs >= BTN_SETTINGS_HOLD_SAVE_MS)) {
        selectHoldSaveDone = true;
        saveOledSettings();
        settingsAdjusting = false;
        noteDisplayActivity();
        popupType = PopupType::None;
        drawCurrentScreen();
      }
    } else if (selectBtnHeld) {
      unsigned long heldMs = millis() - selectBtnDownMs;
      if (!selectHoldSaveDone && heldMs > 20 && heldMs < BTN_SETTINGS_HOLD_SAVE_MS &&
          (millis() - lastSelectBtnPress) > 300) {
        lastSelectBtnPress = millis();
        noteDisplayActivity();
        popupType = PopupType::None;
        if (currentScreen == 3) {
          if (settingsAdjusting) {
            settingsAdjusting = false;
            commitOledSettingsIfDirty();
          } else {
            settingsAdjusting = true;
          }
        } else {
          currentScreen = 3;
          settingsAdjusting = false;
        }
        drawCurrentScreen();
      }
      selectBtnHeld = false;
    }
  }

  if (popupType != PopupType::None && (long)(millis() - popupUntilMs) >= 0) {
    popupType = PopupType::None;
    drawCurrentScreen();
  }

  if (settingsSavedUntilMs != 0 && (long)(millis() - settingsSavedUntilMs) >= 0) {
    settingsSavedUntilMs = 0;
    if (currentScreen == 3 && popupType == PopupType::None &&
        !(batteryCharging && !chargeScreenSuppressed)) {
      drawCurrentScreen();
    }
  }

  static unsigned long lastBatteryRead = 0;
  if (millis() - lastBatteryRead >= 2000UL) {
    lastBatteryRead = millis();
    readBattery();
    // readBattery() redraws only on plug-in; avoid extra full clear while charging
    if (popupType == PopupType::None && !(batteryCharging && !chargeScreenSuppressed)) {
      drawCurrentScreen();
    }
  }

  bool chargeScreenVisible = (batteryCharging && !chargeScreenSuppressed);
  unsigned long uiRefreshPeriod = chargeScreenVisible ? UI_REFRESH_CHARGING_MS : UI_REFRESH_NORMAL_MS;
  if (popupType == PopupType::None && millis() - lastUiRefresh >= uiRefreshPeriod) {
    lastUiRefresh = millis();
    drawCurrentScreen();
  }
}
