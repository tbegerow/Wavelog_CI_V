// =====================
// ESP32 CI-V to Wavelog
// =====================

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include <WebServer.h>
//#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --------------------- CONFIG / DEBUG ---------------------
#define DEBUG_WS 1                     // WebSocket debug broadcast enabled (1 = allow broadcast when level >=1)
#define DEFAULT_DEBUG_LEVEL 1          // 0=off, 1=normal, 2=Debug+ (1 sec. Loop), 3=Debug++ (+JSON), 4=Debug++ (+XML-RPC), 5=Debug++ (+PTTquery)
int debugLevel = DEFAULT_DEBUG_LEVEL;
#define MAX_LOG_LINES 500
String debugLog[MAX_LOG_LINES];
int debugIndex = 0;

// WebSocket
WebSocketsServer webSocket = WebSocketsServer(81); // WebSocket-Server on Port 81

// ---------------- Web / params / servers ------------------
const char* html_username = "sysop";  // Webif username
const char* html_password = "admin";  // Webif password
const int numParams = 6; // number of  parameters
String params[numParams] = {"", "", "", "", "", ""}; // initialization of parameters

WebServer server(80);
WiFiServer  rpcServer(12345);
WiFiClient rpcClient;

// ---------------- timing / constants ---------------------
unsigned long time_current_baseloop;
unsigned long time_last_baseloop;
unsigned long time_last_update;
#define BASELOOP_TICK 5000 // = 5 seconds
#define DEBOUNCE_TIME 1500 // = 1,5sec. process ONLY if TRX has settled
unsigned long lastPostTime = 0;

// define 0.96" SDD1306 display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
unsigned long lastOLED = 0;
const unsigned long OLED_UPDATE_INTERVAL = 500; // ms
char qrgBuf[16];
char pwrBuf[10];
char ipBuf[20];
bool oledRotated = false;

// define state led for ptt-state
#define LED_TX 26
#define LED_RX 27

#define CIV_COUNT (sizeof(civ_addresses) / sizeof(civ_addresses[0]))
uint8_t activeCivAddr = 0x00;
uint8_t civRadioAddr;
uint8_t lastCivAddr = 0;
String radioName = "detecting...";

unsigned long frequency = 0;
float power = 0.0;
String mode_str = "";
volatile bool ptt_state; // = false; // false = RX, true = TX
bool readPTTState() {
  noInterrupts();
  bool v = ptt_state;
  interrupts();
  return v;
}


unsigned long old_frequency = 0;
float old_power = 0.0;
String old_mode_str = "";

// ----- state -> create/post json -----
volatile bool civFreqUpdated  = false;
volatile bool civModeUpdated  = false;
volatile bool civPowerUpdated = false;
// ----- event -> display / XML-RPC
volatile bool civPTTUpdated   = false;

// Realtime-PTT Query
unsigned long lastPTTQuery = 0;
const unsigned long PTT_INTERVAL = 200; // 5x per second
unsigned long lastDebug = 0;

// ICOM coms variables
String autodetectedCiv = "";
const byte TERM_address(0xE0); // Identifies the terminal (ESP32)
const byte startMarker = 0xFE;  // Indicates where the icom signal string starts
const byte endMarker = 0xFD;    // Indicates where the icom signal string ends

const char* civ_options[][2] = { // List of names for select-options in config-page
  {"0", "IC-7300 (94h)"},
  {"1", "IC-7200 (76h)"},
  {"2", "IC-7100 (88h)"},
  {"3", "IC-7000 (70h)"},
  {"4", "IC-7410 (80h)"},
  {"5", "IC-9100 (7Ch)"},
  {"6", "IC-9700 (A2h)"},
  {"7", "ID-5100 (8Ch)"}
};

const byte civ_addresses[] = { // corresponding list of CI-V-Adresses to list above
  0x94,
  0x76,
  0x88,
  0x70,
  0x80,
  0x7C,
  0xA2,
  0x8C
};

const int maxDataLength = 64;
byte receivedData[maxDataLength];  // Array for the recieved data bytes
int dataIndex = 0;                 // Pointer into the data array
bool newData2 = false;             // Set to true when a new data-array has been received

const uint8_t qrg_query[] = {0x03};
const uint8_t mode_query[] = {0x04};
const uint8_t ptt_query[] = {0x1C, 0x00};
const uint8_t power_query[] = {0x14, 0x0A};
size_t qrg_query_length = sizeof(qrg_query) / sizeof(qrg_query[0]);
size_t mode_query_length = sizeof(mode_query) / sizeof(mode_query[0]);
size_t ptt_query_length = sizeof(ptt_query) / sizeof(ptt_query[0]);
size_t power_query_length = sizeof(power_query) / sizeof(power_query[0]);

String buffer;
DynamicJsonDocument jsonDoc(512);
//String rpcBody;
String httpHeader;
String httpBody;

int contentLength = -1;
bool headerComplete = false;

String civCommandToHex(const uint8_t *cmd, size_t len) {
  String s;
  for (size_t i = 0; i < len; i++) {
    if (cmd[i] < 0x10) s += "0";
    s += String(cmd[i], HEX);
    if (i < len - 1) s += " ";
  }
  s.toUpperCase();
  return s;
}

String civCmdName(uint8_t cmd) {
  switch (cmd) {
    case 0x00: return "FREQ(bc)";
    case 0x03: return "FREQ";
    case 0x01: return "MODE";
    case 0x14: return "POWER";
    case 0x1C: return "PTT";
    default:
      return "CMD 0x" + String(cmd, HEX);
  }
}

String civAddrToName(uint8_t addr) {
  for (size_t i = 0; i < CIV_COUNT; i++) {
    if (civ_addresses[i] == addr) {
      return String(civ_options[i][1]);
    }
  }
  return String("unknown");
}

// --------------------- Debug functions ---------------------
#define LOG1(msg) if (debugLevel >= 1) addDebugPrint(msg)
#define LOG2(msg) if (debugLevel >= 2) addDebugPrint(msg)
#define LOG3(msg) if (debugLevel >= 3) addDebugPrint(msg)
#define LOG4(msg) if (debugLevel >= 4) addDebugPrint(msg)
#define LOG5(msg) if (debugLevel >= 5) addDebugPrint(msg)

void loadDebugLevel() {
    if (params[4].length() == 0) {
        // Default
        debugLevel = 1;
        params[4] = "1";
    } else {
        debugLevel = params[4].toInt();
    }

    if (debugLevel < 0 || debugLevel > 5) {
        debugLevel = 1;
        params[4] = "1";
    }

    LOG1(String("Loaded debug level: ") + debugLevel);
//    Serial.printf("Loaded debug level: %d\n", debugLevel);
}

void addDebugRaw(const String &msg) {
  debugLog[debugIndex] = msg;
  debugIndex = (debugIndex + 1) % MAX_LOG_LINES;
}

void broadcastDebug(const String &s) {
  // broadcastTXT(String&) → mutable copy
  String tmp = s;
  webSocket.broadcastTXT(tmp);
}

void addDebugPrint(const String &msg) {

  addDebugRaw(msg);

  // Serial onla if level >= 1
  if (debugLevel >= 1) {
    Serial.println(msg);
  }

  // WebSocket only allowed if enabled & level >= 1
  if (DEBUG_WS && debugLevel >= 1) {
    broadcastDebug(msg);
  }
}

// --------------------- end Debug ------------------------
// ---------------- WiFi connect ----------------
void connectToWifi() {
  WiFiManager wfm;
  wfm.setDebugOutput(false);
  if (!wfm.autoConnect("Wavelog_CI_V_AP", "12345678")) {
    LOG1("failed to connect and hit timeout");
    ESP.restart();
    delay(1000);
  } else {
    LOG1(String("Connected to WiFi. IP: ") + WiFi.localIP().toString());
  }
}

// ---------------- CI-V send / receive ----------------
bool civDataValid = false;
void updateFromCIV() {

  static unsigned long lastPttLog = 0;

  // -------- PTT only → LOG5 (rate limited) --------
  if (civPTTUpdated) {

    if (debugLevel >= 5 && millis() - lastPttLog > 1000) {
      LOG5("[CI-V] PTT update: " + ptt_state);
      lastPttLog = millis();
    }
  }

  // -------- Normal CI-V updates → LOG3 --------
  if (civFreqUpdated || civModeUpdated || civPowerUpdated) {
    LOG3("[CI-V] freq=" + String(frequency) +
         " mode=" + mode_str +
         " power=" + String(power));
  }

  // ---------- First-time validity ----------
  static bool freqSeen = false;
  static bool modeSeen = false;

  if (civFreqUpdated) freqSeen = true;
  if (civModeUpdated) modeSeen = true;

  if (!civDataValid && freqSeen && modeSeen) {
    civDataValid = true;
    LOG1("CI-V data now fully valid – enabling POST");
  }

  // --------- radioName initializing ----------
  static bool radioNameInitialized = false;
  if (!radioNameInitialized && activeCivAddr != 0) {
    LOG1("activeCivAddr now = 0x" + String(activeCivAddr, HEX));

    updateRadioName();
    radioNameInitialized = true;
  }

  civFreqUpdated  = false;
  civModeUpdated  = false;
  civPowerUpdated = false;
  civPTTUpdated   = false;
}

void sendCIVQuery(const uint8_t *commands, size_t length) {

  bool isPTT = (commands[0] == 0x1C); // && commands[1] == 0x00 );
  newData2 = false;

  Serial2.write(0xFE);
  Serial2.write(0xFE);

  // to = Radio
  Serial2.write(civRadioAddr);

  // from = Controller (E0)
  Serial2.write(TERM_address);

  Serial2.write(commands, length);
  Serial2.write(0xFD);

  if (isPTT) {
    LOG5("CI-V PTT query sent");
  } else {
    LOG3(
      "CI-V query sent to radio 0x" +
      String(civRadioAddr, HEX) +
      " CMD: " +
      civCommandToHex(commands, length)
    );
  }
}

void getpower() { sendCIVQuery(power_query, power_query_length); }
void getqrg()   { sendCIVQuery(qrg_query, qrg_query_length); }
void getmode()  { sendCIVQuery(mode_query, mode_query_length); }
void getptt()   { sendCIVQuery(ptt_query, ptt_query_length); LOG5("PTT query sent"); }

bool geticomdata() {
    static uint8_t state = 0;  // 0=idle, 1=first FE, 2=collecting
    static uint16_t idx = 0;

    while (Serial2.available() > 0) {
        uint8_t b = Serial2.read();

        switch (state) {

        case 0: // idle
            if (b == 0xFE) {
                state = 1;
            }
            break;

        case 1: // first FE seen
            if (b == 0xFE) {
                idx = 0;
                receivedData[idx++] = 0xFE;
                receivedData[idx++] = 0xFE;
                state = 2;
            } else {
                state = 0;
            }
            break;

        case 2: // collecting frame
            if (idx >= maxDataLength) {
                LOG1("CI-V RX overflow — frame dropped");
                state = 0;
                idx = 0;
                break;
            }

            receivedData[idx++] = b;

            if (b == 0xFD) {
                dataIndex = idx;
                state = 0;
                idx = 0;
                return true;
            }
            break;
        }
    }

    return false;
}

// ---------------- utility ----------------
int bcd2Dec(int bcdValue) {
  int units = bcdValue & 0xF;  // lower 4 bits
  int tens = (bcdValue >> 4) & 0xF;  // upper 4 bits
  return tens * 10 + units;
}

void logReceivedHex() {
  if (dataIndex > maxDataLength) {
  LOG1("logReceivedHex: dataIndex overflow");
  return;
}

  static char buf[3 * maxDataLength + 16]; // "FF " pro Byte + Prefix
  int pos = 0;

  pos += snprintf(buf + pos, sizeof(buf) - pos, "CI-V RX: ");

  for (int i = 0; i < dataIndex && pos < (int)sizeof(buf) - 4; i++) {
    pos += snprintf(buf + pos, sizeof(buf) - pos,
                    "%02X ", receivedData[i]);
  }

  LOG2(buf);
}

void updateRadioName() {
  if (activeCivAddr == 0) {
    radioName = "detecting...";
    return;
  }

  if (activeCivAddr == lastCivAddr) return;

  lastCivAddr = activeCivAddr;
  radioName = civAddrToName(activeCivAddr);

  if (radioName.length() == 0) {
    radioName = "unknown";
  }

  LOG1("Active TRX → " + radioName +
       " (0x" + String(activeCivAddr, HEX) + ")");
}

// ---------------- parsers ----------------
void calculateQRG() {
  if (dataIndex < 9) {
    LOG2("calculateQRG: frame too short");
    return;
  }

  uint32_t freq = 0;

  freq += bcd2Dec(receivedData[5]);          // Hz + 10 Hz
  freq += bcd2Dec(receivedData[6]) * 100;    // 100 Hz
  freq += bcd2Dec(receivedData[7]) * 10000;  // 10 kHz
  freq += bcd2Dec(receivedData[8]) * 1000000;// 1 MHz

  // optional >100 MHz
  if (dataIndex >= 10) {
    freq += bcd2Dec(receivedData[9]) * 100000000;
  }

  frequency = freq;

  LOG2(String("calculateQRG -> ") + String(frequency));
}

void calculateMode() {

  if (dataIndex < 7) {
    LOG2("Mode frame too short – ignored");
    return;
  }

  uint8_t mode_int = receivedData[5];

  switch (mode_int) {
    case 0x00: mode_str = "LSB"; break;
    case 0x01: mode_str = "USB"; break;
    case 0x02: mode_str = "AM"; break;
    case 0x03: mode_str = "CW"; break;
    case 0x04: mode_str = "RTTY"; break;
    case 0x05: mode_str = "FM"; break;
    case 0x06: mode_str = "FM"; break;      // WFM / FM wide
    case 0x07: mode_str = "CW"; break;      // CW-R
    case 0x08: mode_str = "RTTY"; break;    // RTTY-R
    case 0x17: mode_str = "DSTAR"; break;   // 0x17 == 23
    default:
      mode_str = "UNK";
      LOG2("Unknown mode byte: 0x" + String(mode_int, HEX));
      break;
  }

  LOG2("Mode parsed -> " + mode_str);
  updateFromCIV();
}

void calculatePTT() {
  // Safe check: if dataIndex is small, log and return
  if (dataIndex < 5) {
    LOG1(String("calculatePTT: frame too short (len=") + dataIndex + ")");
    return;
  }

  bool new_ptt = (receivedData[6] == 1);

  if (ptt_state != new_ptt) {
    ptt_state = new_ptt;

    LOG1(new_ptt
         ? "PTT state updated: TX (1)"
         : "PTT state updated: RX (0)");
  }
}

void dumpCivRawFrame(uint8_t len, uint8_t cmd) {
  String s = "[CI-V RX " + civCmdName(cmd) + "] ";
  for (uint8_t i = 0; i < len; i++) {
    if (receivedData[i] < 0x10) s += "0";
    s += String(receivedData[i], HEX);
    s += " ";
  }
  LOG3(s);
}

void processReceivedData(void) {

  static bool inRX = false;
  if (inRX) return;
  inRX = true;

  do {
    // -------------------------------------------------
    // Basic sanity checks
    // -------------------------------------------------
    if (dataIndex > maxDataLength) {
      LOG1("processReceivedData: dataIndex overflow");
      break;
    }

    if (dataIndex < 6) break;

    if (receivedData[0] != 0xFE || receivedData[1] != 0xFE) break;

    byte toAddr   = receivedData[2];
    byte fromAddr = receivedData[3];
    byte cmd      = receivedData[4];

    

    const uint8_t CIV_CONTROLLER = 0xE0;
    const uint8_t CIV_RADIO = civRadioAddr;

    static unsigned long bootTime = millis();
    bool startupPhase = (millis() - bootTime < 3000);

    bool isRadioFrame =
      (fromAddr == CIV_RADIO) &&
      (toAddr == CIV_CONTROLLER || toAddr == 0x00);

    if (startupPhase && fromAddr == CIV_RADIO) {
      isRadioFrame = true;
    }
    if ((cmd == 0x01 || cmd == 0x00 || cmd == 0x03) &&
        fromAddr == CIV_RADIO) {
      isRadioFrame = true;
    }

    if (!isRadioFrame) break;

    // ---- RAW CI-V frame dump (debug only) ----
    if (cmd != 0x1C || debugLevel >= 5) {
      dumpCivRawFrame(dataIndex, cmd);
    }
    // -------------------------------------------------
    // Detect active CI-V device
    // -------------------------------------------------
    if (activeCivAddr != fromAddr) {
      activeCivAddr = fromAddr;
    }

    // -------------------------------------------------
    // Drop malformed MODE frames early
    // -------------------------------------------------
    if (cmd == 0x01 && dataIndex < 7) break;

    // -------------------------------------------------
    // Command decoding
    // -------------------------------------------------
    switch (cmd) {

      case 0x00:
      case 0x03:
        if (dataIndex >= 10) {
          calculateQRG();
          civFreqUpdated = true;

          if (frequency > 1000000UL) {
            civDataValid = true;
            time_last_update = millis();
          }
        }
        break;

      case 0x01:
        if (dataIndex >= 6) {
          calculateMode();
          civModeUpdated = true;
        }
        break;

      case 0x1C:
        if (dataIndex >= 7) {
          bool old_ptt = ptt_state;

          calculatePTT();

          if (ptt_state != old_ptt) {
            civPTTUpdated = true;
          }

          civDataValid = true;
          time_last_update = millis();
        }
        break;

      case 0x14:
        if (dataIndex >= 9) {
          uint8_t hi = receivedData[6];
          uint8_t lo = receivedData[7];

          power = ((hi) * 100 + bcd2Dec(lo)) / 2.55;

          civPowerUpdated = true;
        }
        break;

      default:
        break;
    }

  } while (false);

  inRX = false;
}

// ---------------- Display update ----------------
// updates the SSD1306 display with QRG/Mode/PWR and RX/TX-dot and IP
void updateDisplay() {
  if (millis() - lastOLED < OLED_UPDATE_INTERVAL) return;
  lastOLED = millis();

  display.clearDisplay();

  // RX/TX dot top-right
  if (ptt_state) {
    display.fillCircle(120, 6, 4, SSD1306_WHITE);
  } else {
    display.drawCircle(120, 6, 4, SSD1306_WHITE);
  }

  display.setTextSize(1);
  display.setCursor(98, 0);
  display.print(ptt_state ? "TX" : "RX");

  // ---------- TRX ----------
  display.setCursor(0, 0);
  display.print(radioName);

  // ---------- QRG ----------
  display.setCursor(0, 14);
  display.print("QRG: ");
  snprintf(qrgBuf, sizeof(qrgBuf), "%.4f MHz", frequency / 1000000.0);
  display.print(qrgBuf);

  // ---------- MODE ----------
  display.setCursor(0, 26);
  display.print("Mode: ");
  display.print(mode_str.length() ? mode_str : "-");

  // ---------- POWER ----------
  display.setCursor(0, 38);
  display.print("PWR: ");
  snprintf(pwrBuf, sizeof(pwrBuf), "%.1f W", power);
  display.print(pwrBuf);

  // ---------- IP ----------
  display.setCursor(0, 54);
  display.print("IP: ");
  display.print(ipBuf);

  display.display();

  // LED sync
  digitalWrite(LED_TX, ptt_state ? HIGH : LOW);
  digitalWrite(LED_RX, ptt_state ? LOW  : HIGH);
}

// ---------------- JSON / HTTP / XMLRPC ----------------
void create_json(unsigned long frequency, String mode, bool ptt, float power) {  
  jsonDoc.clear();  
  jsonDoc["radio"] = radioName + " Wavelog CI-V";
  jsonDoc["frequency"] = frequency;
  jsonDoc["mode"] = mode;
  jsonDoc["ptt"] = ptt_state ? "tx" : "rx";
  jsonDoc["ptt_state"] = ptt_state ? 1 : 0;
  jsonDoc["power"] = power;
  jsonDoc["downlink_freq"] = 0;
  jsonDoc["uplink_freq"] = 0;
  jsonDoc["downlink_mode"] = 0;
  jsonDoc["uplink_mode"] = 0;
  jsonDoc["key"] = params[2];
  serializeJson(jsonDoc, buffer);
  LOG3(String("create_json -> ") + buffer);
}

void post_json() {

  if (!civDataValid) {
    LOG1("CI-V data not valid yet, delaying POST...");
    return;
  }

  HTTPClient http;

  String url = params[0] + params[1];
  LOG3(String("POST to URL: ") + url);

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  LOG3(String("POST JSON: ") + String(buffer));

  int httpResponseCode = http.POST(buffer);

  if (httpResponseCode == 200) {
    LOG3(String("HTTP-state: ") + String(httpResponseCode));

    String response = http.getString();
    LOG3(String("HTTP response: ") + response);

    old_mode_str = mode_str;
    old_frequency = frequency;
    old_power = power;

  } else {
    LOG1(String("POST failed. HTTP-state: ") + String(httpResponseCode));
  }

  http.end();
}

// ---------- Web root with JS that polls /trx ----------
void handleRoot() {
  if (!server.authenticate(html_username, html_password)) {
    return server.requestAuthentication();
  }

  String html = "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";
  html += "<meta charset=\"UTF-8\">";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  html += "<title>ESP32 Configuration</title>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; }";
  html += ".container { max-width: 400px; margin: 0 auto; padding: 20px; }";
  html += "input[type='text'] { width: 100%; padding: 10px; margin: 5px 0; }";
  html += "input[type='submit'] { width: 100%; padding: 10px; margin-top: 10px; background-color: #4CAF50; color: white; border: none; }";
  html += ".btn-reboot { width: 100%; padding: 10px; margin-top: 10px; background-color: #FF0000; color: white; border: none; }";
  html += "input[type='submit']:hover { background-color: #45a049; }";
  html += "</style>";
  html += "</head>";
  html += "<body>";
  html += "<div class=\"container\">";
  html += "<h1>ESP32 Wavelog CI-V Configuration</h1>";
  html += "<p>Current QRG/Mode from CI-V:&nbsp;<span id='qrg'></span>&nbsp;/&nbsp;<span id='mode'></span>&nbsp;<span id='ptt'></span></p>";
  html += "<form action='/save' method='post'>";
  html += "<label for='wavelogUrl'>Wavelog URL:</label><br>";
  html += "<input type='text' id='wavelogUrl' name='wavelogUrl' value='" + params[0] + "'><br>";
  html += "<label for='wavelogApiEndpoint'>Wavelog API Endpoint:</label><br>";
  html += "<input type='text' id='wavelogApiEndpoint' name='wavelogApiEndpoint' value='" + params[1] + "'><br>";
  html += "<label for='wavelogApiKey'>Wavelog API Key:</label><br>";
  html += "<input type='text' id='wavelogApiKey' name='wavelogApiKey' value='" + params[2] + "'><br>";
  // --- TRX ADDRESS (HEX values, plus AUTO option) ---
  html += "<div style='display:flex; align-items:center; gap:10px;'>";
  html += "<div>";
  html += "<label for='TrxAddress'>Trx Address:</label><br>";
  html += "<select id='TrxAddress' name='TrxAddress'>";

  // Auto-Option
  html += "<option value='AUTO'";
  if (params[3] == "AUTO" || (params[3].length() == 0 && autodetectedCiv != ""))
    html += " selected";
  html += ">Autodetect";
  if (autodetectedCiv != "")
    html += " (found: " + autodetectedCiv + ")";
  html += "</option>";

  // fixed address
  for (size_t i = 0; i < sizeof(civ_addresses) / sizeof(civ_addresses[0]); ++i) {
    uint8_t a = civ_addresses[i];
    char buf[3];
    sprintf(buf, "%02X", a);
    String hexStr = String(buf);
    html += "<option value='" + hexStr + "'";
    if (params[3].equalsIgnoreCase(hexStr)) html += " selected";
    html += ">" + String(civ_options[i][1]) + " (" + hexStr + "h)</option>";
  }
  html += "</select>";
  html += "<button type='button' onclick='autodetectCiv()'>Autodetect CI-V</button>";
  html += "<button type='button' class='btn-display' onclick='toggleOled()'>OLED rotate 180°</button>";
  html += "<input type='submit' value='Save'>";
  html += "</form>";
  html += "</div>";
  html += "</div>";
  html += "<br>";
  html += "<form action='/reboot' method='post'>";
  html += "<button type='submit' class='btn-reboot'>Reboot ESP32</button>";
  html += "</form>";
  // Debug-level selector
  html += "<h3>Debug Level</h3>";
  html += "<select id='debugLevelSelect'>";
  html += "<option value='0'" + String((debugLevel==0) ? " selected" : "") + ">0 = OFF</option>";
  html += "<option value='1'" + String((debugLevel==1) ? " selected" : "") + ">1 = NORMAL</option>";
  html += "<option value='2'" + String((debugLevel==2) ? " selected" : "") + ">2 = DEBUG+ (1s Loop)</option>";
  html += "<option value='3'" + String((debugLevel==3) ? " selected" : "") + ">3 = DEBUG++ (+CI-V/JSON)</option>";
  html += "<option value='4'" + String((debugLevel==4) ? " selected" : "") + ">4 = DEBUG++ (+XML-RPC)</option>";
  html += "<option value='5'" + String((debugLevel==5) ? " selected" : "") + ">5 = DEBUG++ (+PTT-query)</option>";
  html += "</select>";
  html += "<button onclick='setDebugLevel()'>Set</button>";
  html += "<p id='debugSetResult'></p>";

  html += "<p><a href='/debug'>Debug (page)</a> | <a href='/debugws'>Debug WS (live)</a></p>";
  html += "<form action='/logged-out' method='post'>";
  html += "<button type='submit' class='btn-logout'>Logout</button>";
  html += "</form>";
  html += "</div>";

  // JS: polling + debug-level setter
  html += "<script>";
  html += R"JS(
    function refreshTrx() {
      fetch('/trx')
        .then(response => response.json())
        .then(data => {
          document.getElementById('qrg').textContent = (data.qrg/1000) + ' kHz';
          document.getElementById('mode').textContent = data.mode;
          document.getElementById('ptt').textContent = (data.ptt === 'tx') ? '🔴 TX' : '🟢 RX';
        })
        .catch(error => {
          console.error('Fetch error:', error);
        });
    }
    setInterval(refreshTrx, 1000);
    refreshTrx();

    function setDebugLevel() {
      var lvl = document.getElementById('debugLevelSelect').value;
      fetch('/setDebug?debuglevel=' + lvl)
        .then(r => r.json())
        .then(j => {
          document.getElementById('debugSetResult').textContent = 'Set debug level to ' + j.debuglevel;
        })
        .catch(e => {
          document.getElementById('debugSetResult').textContent = 'Error setting debug level';
        });
    }

    function autodetectCiv() {
      fetch('/autodetect', { method: 'POST' })
        .then(r => r.json())           
        .then(j => {                   
          console.log(j);
          if (j.status === "ok") {
            alert('CI-V Address detected: ' + j.civAddr);
            location.reload();
          } else {
            alert('Autodetect failure');
          }
        })
        .catch(e => {
          alert('Autodetect failure');
          console.error(e);
        });
    }

    function toggleOled() {
      fetch('/toggleOled', { method: 'POST' })
        .then(r => r.json())
        .then(j => {
          console.log("OLED rotated:", j.rotated);
          document.getElementById('oledBtn').textContent =
            j.rotated ? 'OLED normal' : 'OLED 180°';
        })
        .catch(e => console.error(e));
    }
  )JS";
  html += "</script></body></html>";

  server.send(200, "text/html", html);
}

// handle /trx used by root page
void handleTrx() {
  if (!server.authenticate(html_username, html_password)) {
    return server.requestAuthentication();
  }
  jsonDoc.clear();
  jsonDoc["qrg"] = frequency;
  jsonDoc["mode"] = mode_str;
  jsonDoc["power"] = power;
  jsonDoc["ptt"] = ptt_state;
  serializeJson(jsonDoc, buffer);
  server.send(200, "application/json", buffer);
}

void applyOledRotation() {
  if (oledRotated) {
    display.setRotation(2);   // 180°
  } else {
    display.setRotation(0);   // normal
  }
  display.clearDisplay();
  display.display();
}

void handleToggleOled() {
  oledRotated = !oledRotated;
  params[5] = oledRotated ? "1" : "0";

  saveParametersToSPIFFS();
  applyOledRotation();

  server.send(200, "application/json",
    String("{\"rotated\":") + (oledRotated ? "true" : "false") + "}");
}

void handleAutoDetect() {

  if (!server.authenticate(html_username, html_password)) {
    return server.requestAuthentication();
  }

  LOG1("Starting CI-V autodetection...");

  int detectedIndex = detectCivAddress();

  if (detectedIndex >= 0) {

    String detectedHex = String(civ_addresses[detectedIndex], HEX);
    detectedHex.toUpperCase();
    if (detectedHex.length() < 2)
      detectedHex = "0" + detectedHex;

    params[3] = detectedHex;
    civRadioAddr = civ_addresses[detectedIndex];
    saveParametersToSPIFFS();

    LOG1("Autodetected CI-V Address saved: " + detectedHex);

    server.send(200, "application/json",
                "{\"status\":\"ok\",\"civAddr\":\"" + detectedHex + "\"}");
  } else {

    LOG1("No CI-V address detected!");
    server.send(200, "application/json",
                "{\"status\":\"fail\"}");
  }
}

void handleLogout() {
  server.send(401);
}

void handleLoggedout() {
  String html = "<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><title>ESP32 Configuration Logout</title></head><body><div><h1>Logout</h1><form action='/' method='post'><button type='submit'>Return to Homepage</button></form></div></body></html>";
  server.send(200, "text/html", html);
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

// Endpoint to set debug level and persist it
void handleSetDebug() {
    if (!server.hasArg("debuglevel")) {
        server.send(400, "application/json", "{\"error\":\"missing debuglevel\"}");
        return;
    }

    int lvl = server.arg("debuglevel").toInt();
    if (lvl < 0) lvl = 0;
    if (lvl > 5) lvl = 5;
    params[4] = String(lvl);
    
    saveParametersToSPIFFS();

    debugLevel = lvl;

    server.send(200, "application/json",
                "{\"debuglevel\":" + String(lvl) + "}");

}

// Debug page (plain log)
void handleDebugPage() {
  if (!server.authenticate(html_username, html_password)) {
    return server.requestAuthentication();
  }
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>ESP32 Debug Log</title>";
  html += "<style>body{font-family:monospace;background:#000;color:#0f0;padding:10px;white-space:pre-wrap}</style></head><body>";
  html += "<h3>ESP32 Debug Log (last " + String(MAX_LOG_LINES) + " lines)</h3>\n";
  int idx = debugIndex;
  for (int i = 0; i < MAX_LOG_LINES; ++i) {
    String line = debugLog[idx];
    if (line.length() > 0) html += line + "\n";
    idx = (idx + 1) % MAX_LOG_LINES;
  }
  html += "<p><a href='/debugws'>Open Live WebSocket Console</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Live WebSocket Debug page (connects to ws://<ip>:81)
void handleDebugWSPage() {
  if (!server.authenticate(html_username, html_password)) {
    return server.requestAuthentication();
  }
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Live Debug Console</title>";
  html += "<style>body{font-family:monospace;background:#000;color:#0f0;padding:10px}#log{height:70vh;overflow:auto;border:1px solid #444;padding:5px;background:#050505}</style></head><body>";
  html += "<h3>Live Debug Console (WebSocket)</h3>";
  html += "<div id='controls'><button onclick='sendCmd(\"last\")'>Send last</button> <button onclick='sendCmd(\"clear\")'>Clear server log</button></div>";
  html += "<div id='log'></div>";
  html += R"JS(
    <script>
    let host = location.hostname;
    let ws = new WebSocket('ws://' + host + ':81/');
    let log = document.getElementById('log');
    ws.onopen = function() {
      log.appendChild(document.createTextNode('[WS] connected\n'));
      ws.send('last'); // ask for last logs on connect
    };
    ws.onmessage = function(evt) {
      let line = evt.data;
      let node = document.createElement('div');
      node.textContent = line;
      log.appendChild(node);
      log.scrollTop = log.scrollHeight;
      };
    ws.onclose = function() {
      log.appendChild(document.createTextNode('[WS] disconnected\n'));
    };
    function sendCmd(cmd) {
    if (ws.readyState === WebSocket.OPEN) ws.send(cmd);
    }
    </script>
  )JS";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// ---------------- WebSocket event handler ----------------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    LOG2(String("WS RX from client ") + String(num) + ": " + msg);
    if (msg == "clear") {
      // clear server buffer
      for (int i = 0; i < MAX_LOG_LINES; ++i) debugLog[i] = "";
      debugIndex = 0;
      webSocket.sendTXT(num, "Server log cleared");
    } else if (msg == "last") {
      // send last N lines (up to MAX_LOG_LINES)
      int idx = debugIndex;
      for (int i = 0; i < MAX_LOG_LINES; ++i) {
        String line = debugLog[idx];
        if (line.length() > 0) webSocket.sendTXT(num, line);
        idx = (idx + 1) % MAX_LOG_LINES;
      }
      webSocket.sendTXT(num, "[END OF LOG]");
    } else {
      webSocket.sendTXT(num, "Unknown command: " + msg);
    }
  } else if (type == WStype_CONNECTED) {
    IPAddress ip = webSocket.remoteIP(num);
    LOG1(String("WebSocket client connected: ") + ip.toString());
    webSocket.sendTXT(num, "Welcome to ESP32 Debug WebSocket");
  } else if (type == WStype_DISCONNECTED) {
    LOG1(String("WebSocket client disconnected #") + String(num));
  }
}

int detectCivAddress() {

  LOG1("Starting CI-V autodetect...");

  const byte controller = 0xE0;   // ESP32 CI-V address
  const byte cmd = 0x03;          // Read frequency

  for (int i = 0; i < sizeof(civ_addresses); i++) {

    byte addr = civ_addresses[i];
    LOG2("Probing CI-V address 0x" + String(addr, HEX));

    // --- SEND PROBE ---
    Serial2.write(0xFE);
    Serial2.write(0xFE);
    Serial2.write(addr);        // TO radio
    Serial2.write(controller);  // FROM ESP
    Serial2.write(cmd);
    Serial2.write(0xFD);

    unsigned long timeout = millis() + 200;

    int state = 0;
    byte toAddr = 0;
    byte fromAddr = 0;
    byte rxCmd = 0;

    while (millis() < timeout) {

      if (!Serial2.available()) continue;
      byte b = Serial2.read();

      switch (state) {

        case 0:  // FE
          if (b == 0xFE) state = 1;
          break;

        case 1:  // FE
          if (b == 0xFE) state = 2;
          else state = 0;
          break;

        case 2:  // TO
          toAddr = b;
          state = 3;
          break;

        case 3:  // FROM
          fromAddr = b;
          state = 4;
          break;

        case 4:  // CMD
          rxCmd = b;
          state = 5;
          break;

        case 5:  // wait for FD
          if (b == 0xFD) {
            if (toAddr == controller &&
                fromAddr == addr &&
                rxCmd == cmd) {

              LOG1("Auto-detected CI-V address: 0x" + String(addr, HEX));
              return i;
            }
            state = 0;
          }
          break;
      }
    }
  }
  LOG1("Auto-detect failed: no CI-V device responded");
  return -1;
}

// ---------------- SPIFFS params IO ----------------
void loadParametersFromSPIFFS() {
  for (int i = 0; i < numParams; ++i) {
    File file = SPIFFS.open("/param" + String(i) + ".txt", "r");
    if (!file) {
      LOG1("Failed to open file for reading");
      return;
    }
    params[i] = file.readStringUntil('\n');
    file.close();
  }
}

void saveParametersToSPIFFS() {
  for (int i = 0; i < numParams; ++i) {
    File file = SPIFFS.open("/param" + String(i) + ".txt", "w");
    if (!file) {
      LOG1("Failed to open file for writing");
      return;
    }
    file.print(params[i]);
    file.close();
  }
}

void handleSave() {
  if (!server.authenticate(html_username, html_password)) {
    return server.requestAuthentication();
  }

  // Basiskonfiguration
  // -------------------------------
  params[0] = server.arg("wavelogUrl");
  params[1] = server.arg("wavelogApiEndpoint");
  params[2] = server.arg("wavelogApiKey");

  // TRX Address (HEX oder AUTO)
  // -------------------------------
  String trxSel = server.arg("TrxAddress");
  trxSel.trim();
  trxSel.toUpperCase();
  if (trxSel == "AUTO") {
  // Auto-Modus active → save "AUTO"
    params[3] = "AUTO";
  } else {
  // fix HEX-address exp. "7C"
    params[3] = trxSel;
  } 

  saveParametersToSPIFFS();

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleReboot() {
  ESP.restart();
}

String rig_get_vfo() {
  return String(frequency);
}

String rig_get_mode() {
  return mode_str;
}

//String rig_get_ptt() {
//  return ptt_state;
//}

// ---------------- XML-RPC handling ----------------
void processXMLRPC(String &req, WiFiClient &client) {

  String response;

  if (req.indexOf("<methodName>rig.get_vfo</methodName>") != -1) {
    response = "VFOA";
    LOG4("XMLRPC-Response (rig.get_vfo): VFOA");
  }
  else if (req.indexOf("<methodName>rig.get_freq</methodName>") != -1) {
    response = String(frequency);
    LOG4("XMLRPC-Response (rig.get_freq): " + String(frequency));
  }
  else if (req.indexOf("<methodName>rig.get_mode</methodName>") != -1) {
    response = mode_str;
    LOG4("XMLRPC-Response (rig.get_mode): " + mode_str);
  }
  else if (req.indexOf("<methodName>rig.get_ptt</methodName>") != -1) {
    int ptt = readPTTState() ? 1 : 0;
    sendXMLRPCResponsePTT(client, ptt);
    LOG4("XMLRPC-Response (rig.get_ptt): " + String(ptt));
    return;
  }
  else {
    if (req.indexOf("<methodCall>") == -1) return;
    return;
  }

  sendXMLRPCResponse(client, response);
}

void sendXMLRPCResponse(WiFiClient &client, String value) {
  String body =
    "<?xml version=\"1.0\"?>\r\n"
    "<methodResponse>\r\n"
      "<params><param>\r\n"
        "<value><string>" + value + "</string></value>\r\n"
      "</param></params>\r\n"
    "</methodResponse>\r\n";

  String header =
    "HTTP/1.1 200 OK\r\n"
    "Server: XMLRPC++ 0.8\r\n"
    "Content-Type: text/xml\r\n"
    "Content-Length: " + String(body.length()) + "\r\n"
    "Connection: keep-alive\r\n\r\n";

  LOG4("RPC TX HEADER:");
  LOG4(header);
  LOG4("RPC TX BODY:");
  LOG4(body);

  client.print(header);
  client.print(body);
}

void sendXMLRPCResponsePTT(WiFiClient &client, int value) {
  String body =
    "<?xml version=\"1.0\"?>\r\n"
    "<methodResponse>\r\n"
      "<params>\r\n"
        "<param>\r\n"
          "<value><i4>" + String(value) + "</i4></value>\r\n"
        "</param>\r\n"
      "</params>\r\n"
    "</methodResponse>\r\n";

  String response =
    "HTTP/1.1 200 OK\r\n"
    "Server: XMLRPC++ 0.8\r\n"
    "Content-Type: text/xml\r\n"
    "Content-Length: " + String(body.length()) + "\r\n"
//    "Connection: keep-alive\r\n\r\n";
    "\r\n" +
    body;

  LOG4("RPC TX RESPONSE:");
  LOG4(response);
  
  client.print(response);
}

unsigned long lastRPC = 0;

void handleRPCServer() {
  // ---- Keep-Alive Timeout ----
  if (rpcClient && rpcClient.connected()) {
    if (!headerComplete && httpHeader.length() == 0) {
    } else if (millis() - lastRPC > 15000) {   // 15 sec
      LOG4("RPC keep-alive timeout, closing");
      rpcClient.stop();
      return;
    }
  }

  if (!rpcClient || !rpcClient.connected()) {
    rpcClient = rpcServer.available();
    if (rpcClient) {
      httpHeader = "";
      httpBody = "";
      contentLength = -1;
      headerComplete = false;
      lastRPC = millis();
      LOG4("RPC client connected");
    }
    return;
  }

  while (rpcClient.available()) {
    lastRPC = millis();
    char c = rpcClient.read();

    // ---- HEADER PHASE ----
    if (!headerComplete) {
      httpHeader += c;

      if (httpHeader.endsWith("\r\n\r\n")) {
        headerComplete = true;

        String headerLower = httpHeader;
        headerLower.toLowerCase();

        int idx = headerLower.indexOf("content-length:");
          if (idx >= 0) {
            int end = headerLower.indexOf("\r\n", idx);
            String lenStr = headerLower.substring(idx + 15, end);
            lenStr.trim();
            contentLength = lenStr.toInt();
          }

        LOG4("RPC RX HEADER:");
        LOG4(httpHeader);
        LOG4("Parsed Content-Length: " + String(contentLength));

        if (contentLength <= 0) {
          LOG4("Invalid Content-Length → 400");
          sendHTTPError(rpcClient);
          rpcClient.stop();
          return;
        }
      }
    }

    // ---- BODY PHASE ----
    else {
      httpBody += c;

      if (httpBody.length() >= contentLength) {
        LOG4("RPC RX BODY:");
        LOG4(httpBody);

        processXMLRPC(httpBody, rpcClient);

        httpHeader = "";
        httpBody = "";
        contentLength = -1;
        headerComplete = false;

        lastRPC = millis();
        LOG4("RPC request handled, keep-alive");
//        return;
      }
    }
  }
}

void sendHTTPError(WiFiClient &client) {
  client.print(
    "HTTP/1.1 400 Bad Request\r\n"
    "Content-Length: 0\r\n"
    "Connection: close\r\n"
    "\r\n"
  );
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);  // Serial console
  Serial2.begin(19200, SERIAL_8N1, 16, 17);  // ICOM CI-V
  delay(1000);

  LOG1("");
  LOG1("Booting Sketch...");
  MDNS.begin("esp32-ci-v");
  
  if (!SPIFFS.begin(true)) {
    LOG1("SPIFFS initialization failed!");
    return;
  }
  
  loadParametersFromSPIFFS();

  delay(5);
  loadDebugLevel();
  delay(5);

  if (params[3].length() == 0 || params[3] == "00") {
    LOG1("No CI-V address stored. Starting auto-detection...");

    detectCivAddress();

    if (autodetectedCiv.length() == 2) {
        params[3] = autodetectedCiv;
        saveParametersToSPIFFS();
        LOG1("Autodetected CI-V Address saved: " + autodetectedCiv);
    } else {
        LOG1("CI-V auto-detect failed. Keeping address unset.");
    }
  } else {
    LOG1("CI-V address loaded from SPIFFS: " + params[3]);
  }
  // -------------------------------------------------
  // Convert CI-V address string ("7C") to byte (0x7C)
  // -------------------------------------------------
  civRadioAddr = strtol(params[3].c_str(), nullptr, 16);

  LOG1("CI-V address parsed as hex: 0x" + String(civRadioAddr, HEX));

  if (params[5].length() == 0) {
    params[5] = "0";
    oledRotated = false;
  }

  Wire.begin(21, 22);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for(;;); // stop forever
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Wavelog CI-V Booting...");
    display.display();
    delay(1500);
  }
  oledRotated = (params[5] == "1");
  applyOledRotation();

  pinMode(LED_RX, OUTPUT);
  pinMode(LED_TX, OUTPUT);

  // Initialstatus: RX = green
  digitalWrite(LED_RX, HIGH);
  digitalWrite(LED_TX, LOW);

  LOG1(String("URL: ") + params[0]);
  LOG1(String("Endpoint: ") + params[1]);
  LOG1(String("API-Key: ") + params[2]);
  LOG1(String("CI-V-Address: ") + params[3]);
  connectToWifi();

  snprintf(ipBuf, sizeof(ipBuf), "%s", WiFi.localIP().toString().c_str());

  LOG1(String("IP-Address: ") + WiFi.localIP().toString());
  
  // --- Initial CI-V read at startup ---
  delay(500);   // small delay to let CI-V settle
  getqrg();     // read qrg immidiately
  getmode();    // read mode immediately
  getpower();   // read power immediately
  getptt();     // get PTT state at startup

  time_last_baseloop = millis();   // reset timer to avoid double polling

  server.on("/", HTTP_GET, handleRoot);
  server.on("/trx", HTTP_GET, handleTrx);
  server.on("/toggleOled", HTTP_POST, handleToggleOled);
  server.on("/autodetect", HTTP_POST, handleAutoDetect);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/reboot", HTTP_POST, handleReboot);
  server.on("/logout", HTTP_GET, handleLogout, handleLoggedout);
  server.on("/logged-out", HTTP_GET, handleLoggedout);
  server.onNotFound(handle_NotFound);

  // Debug Live WebSocket page
  server.on("/debugws", HTTP_GET, handleDebugWSPage);

  // Endpoint to set debug level
  server.on("/setDebug", HTTP_GET, handleSetDebug);

  // Debug web page (shows last MAX_LOG_LINES log entries)
  server.on("/debug", HTTP_GET, handleDebugPage);

  server.begin();
  LOG1("HTTP server started");

  // WebSocket started
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  LOG1("WebSocket server started on port 81");

  // Routing for XML-RPC-requests
  rpcServer.begin();
  rpcServer.setNoDelay(true);
  LOG1("FLRIG-compatible XML-RPC server started on port 12345");
  
  delay(1000);
  time_current_baseloop = millis();
  time_last_baseloop = time_current_baseloop;

  if (civDataValid) {
    create_json(frequency, mode_str, readPTTState(), power);
    post_json();
  } else {
    Serial.println("Skipping POST: No valid TRX data yet.");
  }
}

// ---------------- Loop ----------------
void loop() {
  server.handleClient();
  // === XML-RPC (HamClock /FLRIG ) ===
  handleRPCServer();

  webSocket.loop();
//  server.handleClient();

  if (WiFi.status() == WL_CONNECTED) {
    time_current_baseloop = millis();

    //query tx-power every 5 secs
    if ((time_current_baseloop - time_last_baseloop) > BASELOOP_TICK) {
      getpower();  // get tx-power from TRX
      time_last_baseloop = time_current_baseloop;
    }
    
    //query ptt-state every 200 ms
    if (millis() - lastPTTQuery > PTT_INTERVAL) {
      getptt();  // get ptt-state from TRX
      lastPTTQuery = millis();
    }
    
    if (geticomdata()) {
      processReceivedData();
      
      // ---------- Initial CI-V sync (once, after RX is alive) ----------
      static bool initialCivQueryDone = false;

      if (!initialCivQueryDone && activeCivAddr != 0) {
        LOG1("CI-V RX active – performing initial TRX sync");

        delay(50);
        getqrg();
        delay(30);
        getmode();
        delay(30);
        getpower();

        initialCivQueryDone = true;
      }
      
      bool civFrameChanged =
        civFreqUpdated ||
        civModeUpdated ||
        civPowerUpdated;

      if (civFrameChanged) {
        updateFromCIV();
      }
    }
    static bool oldInitialized = false;

    if (civDataValid) {

      // ---- initial sync once ----
      if (!oldInitialized) {
        old_frequency = frequency;
        old_mode_str  = mode_str;
        old_power     = power;
        lastPostTime  = millis();
        oldInitialized = true;
      }

      bool trxChanged =
      frequency != old_frequency ||
      mode_str  != old_mode_str  ||
      power     != old_power;

      if (trxChanged && millis() - lastPostTime > DEBOUNCE_TIME) {

        create_json(frequency, mode_str, readPTTState(), power);
        post_json();

        old_frequency = frequency;
        old_mode_str  = mode_str;
        old_power     = power;
        lastPostTime  = millis();
      }
    }

    // Debug-Output every second
    if (millis() - lastDebug > 1000) {
      LOG2(String("[DBG] QRG: ") + String(frequency) + " | MODE: " + mode_str + " | PWR: " + String(power) + " | PTT: " + ptt_state);
      lastDebug = millis();
    }
    
    // periodic display refresh (if no new CI-V frames)
    updateDisplay();
    
    } 
    else {
    LOG1("No connection to your WiFi-network.");
    connectToWifi();
  }
} // end loop