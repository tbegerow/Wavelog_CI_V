// =====================
// ESP32 CI-V to Wavelog
// =====================

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <WebSocketsServer.h>

// --------------------- DEBUG SYSTEM ---------------------
#define DEBUG_LEVEL 2     // 0=none, 1=normal, 2=Debug++ (human readable)
#define MAX_LOG_LINES 500
String debugLog[MAX_LOG_LINES];
int debugIndex = 0;

WebSocketsServer webSocket = WebSocketsServer(81); // WebSocket-Server on Port 81

void addDebugRaw(const String &msg) {
  debugLog[debugIndex] = msg;
  debugIndex = (debugIndex + 1) % MAX_LOG_LINES;
}

void broadcastDebug(String msg) {
  // send to websocket clients immediately if any connected
  // webSocket.broadcastTXT exists in the library
  webSocket.broadcastTXT(msg);
}

void addDebugPrint(const String &msg) {
  if (DEBUG_LEVEL > 0) Serial.println(msg);
  addDebugRaw(msg);
  if (DEBUG_LEVEL > 0) broadcastDebug(msg);
}

#define LOG1(msg) if (DEBUG_LEVEL >= 1) addDebugPrint(msg)
#define LOG2(msg) if (DEBUG_LEVEL >= 2) addDebugPrint(msg)
// --------------------- end Debug ------------------------

const char* html_username = "sysop";  // Webif username
const char* html_password = "admin";  // Webif password
const int numParams = 4; // number of  parameters
String params[numParams] = {"", "", "", ""}; // initialization of parameters
WebServer server(80);
WebServer XMLRPCserver(12345);

unsigned long time_current_baseloop;
unsigned long time_last_baseloop;
unsigned long time_last_update;
#define BASELOOP_TICK 5000 // = 5 seconds
#define DEBOUNCE_TIME 1500 // = 1,5sec. process ONLY if TRX has settled

unsigned long frequency = 0;
float power = 0.0;
String mode_str = "";
String ptt_str = "";
bool ptt_state = false; // false = RX, true = TX

unsigned long old_frequency = 0;
float old_power = 0.0;
String old_mode_str = "";
String old_ptt_str = "";

// Realtime-PTT Query
unsigned long lastPTTQuery = 0;
const unsigned long PTT_INTERVAL = 200; // 5x per second
unsigned long lastDebug = 0;

// ICOM coms variables
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

const int maxDataLength = 16;
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
void sendCIVQuery(const uint8_t *commands, size_t length) {
  newData2 = false;
  Serial2.write(startMarker);  // always twice 0xFE in the beginning
  Serial2.write(startMarker);
  Serial2.write(civ_addresses[params[3].toInt()]);
  Serial2.write(TERM_address);
  Serial2.write(commands, length);
  Serial2.write(endMarker);

  LOG2("CI-V query sent");
}

void getpower() { sendCIVQuery(power_query, power_query_length); }
void getqrg()   { sendCIVQuery(qrg_query, qrg_query_length); }
void getmode()  { sendCIVQuery(mode_query, mode_query_length); }
void getptt()   { sendCIVQuery(ptt_query, ptt_query_length); LOG2("PTT query sent"); }

void geticomdata() {
  while (Serial2.available() > 0 && newData2 == false) {
    byte currentByte = Serial2.read();
    if (currentByte == startMarker) {
      dataIndex = 0;
    } else if (currentByte == endMarker) {
      newData2 = true;
    } else {
      if (dataIndex < maxDataLength) {
        receivedData[dataIndex] = currentByte;
        dataIndex++;
      }
    }  
  }
}

// ---------------- utility ----------------
int bcd2Dec(int bcdValue) {
    int units = bcdValue & 0xF;  // lower 4 bits
    int tens = (bcdValue >> 4) & 0xF;  // upper 4 bits

    return tens * 10 + units;
}

void logReceivedHex() {
  // build hex string of receivedData up to dataIndex
  String s = "CI-V RX: ";
  for (int i = 0; i < dataIndex; ++i) {
    if (receivedData[i] < 0x10) s += "0";
    s += String(receivedData[i], HEX);
    s += " ";
  }
  LOG2(s);
}

// ---------------- parsers ----------------
void calculateQRG() {
  uint8_t GHZ = 0, MHZ = 0, KHZ = 0, HZ = 0, mHZ = 0;
  uint32_t t_QRG = 0;
  GHZ = bcd2Dec(receivedData[7]);           // 1GHz & 100Mhz
  MHZ = bcd2Dec(receivedData[6]);           // 10Mhz & 1Mhz
  KHZ = bcd2Dec(receivedData[5]);           // 100Khz & 10KHz
  HZ = bcd2Dec(receivedData[4]);         // 1Khz & 100Hz
  
  t_QRG = ((GHZ * 1000000) + (MHZ * 10000) + (KHZ * 100) + (HZ * 1));  // QRG variable stores frequency in GMMMkkkH format - Frequ divided by 100
  frequency = t_QRG*100;
  LOG2(String("calculateQRG -> ") + String(frequency));
}

void calculateMode() {
  uint8_t mode_int = receivedData[3];
  switch (mode_int) {
    case 0: mode_str = "LSB"; break;
    case 1: mode_str = "USB"; break;
    case 2: mode_str = "AM"; break;
    case 3: mode_str = "CW"; break;
    case 4: mode_str = "RTTY"; break;
    case 5: mode_str = "FM"; break;
    case 6: mode_str = "FM"; break;
    case 7: mode_str = "CW"; break;
    case 8: mode_str = "RTTY"; break;
    case 23: mode_str = "DSTAR"; break;
  }
  LOG2(String("Mode parsed -> ") + mode_str);
}

void calculatePTT() {
  // Safe check: if dataIndex is small, log and return
  if (dataIndex < 5) {
    LOG2(String("calculatePTT: frame too short (len=") + dataIndex + ")");
    return;
  }

  uint8_t ptt_int = receivedData[4]; // many ICOMs use byte 5 (index 4) or 6; this is your current mapping
  switch (ptt_int) {
    case 0:
      if (ptt_str != "rx") {
        ptt_str = "rx";
        ptt_state = false;
        LOG1("PTT state updated: RX (0)");
      }
      break;
    case 1:
      if (ptt_str != "tx") {
        ptt_str = "tx";
        ptt_state = true;
        LOG1("PTT state updated: TX (1)");
      }
      break;
  }
}

void processReceivedData(void) {
  // Debug hex dump
  logReceivedHex();

  if ((receivedData[0] == 0x00 || receivedData[0] == 0xE0)) { // broadcast to all (0x00), reply to terminal (0xE0)
    switch(receivedData[2]) {
      case 0x0: { // reply at broadcast - frequency?
        calculateQRG();
        break;
      }
      case 0x1: { // mode broadcast
        calculateMode();
        break;
      }
      case 0x1C: { // reply to terminal on query
        calculatePTT();
        break;
      }
      case 0x3: { // qrg reply to terminal on query
        calculateQRG();
        break;
      }
      case 0x14: {
        power = ((receivedData[4]) * 100 + bcd2Dec(receivedData[5])) / 2.55;
        LOG2(String("Power parsed -> ") + String(power));
        break;
      }
      default:
        LOG2(String("Unhandled CI-V cmd: 0x") + String(receivedData[2], HEX));
        break;
    }
  } else {
    LOG2("Frame not for us");
  }
}

// ---------------- JSON / HTTP / XMLRPC ----------------
void create_json(unsigned long frequency, String mode, String ptt, float power) {  
  jsonDoc.clear();  
  jsonDoc["radio"] = String(civ_options[params[3].toInt()][1]) + " Wavelog CI-V";
  jsonDoc["frequency"] = frequency;
  jsonDoc["mode"] = mode;
  jsonDoc["ptt"] = ptt;
  jsonDoc["ptt_state"] = ptt_state ? 1 : 0;
  jsonDoc["power"] = power;
  jsonDoc["downlink_freq"] = 0;
  jsonDoc["uplink_freq"] = 0;
  jsonDoc["downlink_mode"] = 0;
  jsonDoc["uplink_mode"] = 0;
  jsonDoc["key"] = params[2];
  serializeJson(jsonDoc, buffer);
  LOG2(String("create_json -> ") + buffer);
}

void post_json() {
  HTTPClient http;
  http.begin(params[0] + params[1]); // Endpoint of REST-API on the Wavelog-webserver
  http.addHeader("Content-Type", "application/json");
  Serial.println(buffer); // keep this raw print for compatibility (still visible only if Serial active)
  int httpResponseCode = http.POST(buffer);

  if (httpResponseCode > 0) {
    LOG1(String("HTTP-state: ") + String(httpResponseCode));
    String response = http.getString();
    LOG2(String("HTTP response: ") + response);
    old_mode_str = mode_str;
    old_ptt_str = ptt_str;
    old_frequency = frequency;
    old_power = power;
  } else {
    LOG1(String("Error on sending the post-request. HTTP-state: ") + String(httpResponseCode));
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
  html += "<label for='TrxAddress'>Trx Address:</label><br>";
  html += "<select id='TrxAddress' name='TrxAddress'>";

  for (int i = 0; i < sizeof(civ_options) / sizeof(civ_options[0]); ++i) {
    String optionHTML = "<option value='" + String(civ_options[i][0]) + "'";
    optionHTML += (params[3].toInt() == atoi(civ_options[i][0]) ? " selected" : "");
    optionHTML += ">" + String(civ_options[i][1]) + "</option>";
    html += optionHTML;
  }

  html += "</select><br>";
  html += "<input type='submit' value='Save'>";
  html += "</form>";
  html += "<form action='/reboot' method='post'>";
  html += "<button type='submit' class='btn-reboot'>Reboot ESP32</button>";
  html += "</form>";
  html += "<p><a href='/debug'>Debug (page)</a> | <a href='/debugws'>Debug WS (live)</a></p>";
  html += "<form action='/logged-out' method='post'>";
  html += "<button type='submit' class='btn-logout'>Logout</button>";
  html += "</form>";
  html += "</div>";
  html += "<script>";
  html += R"JS(
  setInterval(function() {
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
  }, 1000);
  )JS";
  html += "</script></body></html>";

  server.send(200, "text/html", html);
}

void handleLogout() {
  server.send(401);
}

// handle /trx used by root page
void handleTrx() {
  jsonDoc.clear();  
  jsonDoc["qrg"] = frequency;
  jsonDoc["mode"] = mode_str;
  jsonDoc["ptt"] = ptt_str;
  serializeJson(jsonDoc, buffer);
  server.send(200, "application/json", buffer);
}

void handleLoggedout() {
  String html = "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";
  html += "<meta charset=\"UTF-8\">";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">";
  html += "<title>ESP32 Configuration Logout</title>";
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
  html += "<h1>Logout</h1>";
  html += "<form action='/' method='post'>";
  html += "<button type='submit' class='btn'>Return to Homepage</button>";
  html += "</form>";
  html += "</div>";
  html += "</body>";
  html += "</html>";

  server.send(200, "text/html", html);
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

// Debug page (plain log)
void handleDebugPage() {
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
  String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>Live Debug Console</title>";
  html += "<style>body{font-family:monospace;background:#000;color:#0f0;padding:10px}#log{height:70vh;overflow:auto;border:1px solid #444;padding:5px;background:#050505}</style></head><body>";
  html += "<h3>Live Debug Console (WebSocket)</h3>";
  html += "<div id='controls'><button onclick='sendCmd(\"last\")'>Send last</button> <button onclick='sendCmd(\"clear\")'>Clear server log</button></div>";
  html += "<div id='log'></div>";
  html += R"JS(
<script>
let host = location.hostname;
let port = 81;
let ws = new WebSocket('ws://' + host + ':' + port + '/');
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
  params[0] = server.arg("wavelogUrl");
  params[1] = server.arg("wavelogApiEndpoint");
  params[2] = server.arg("wavelogApiKey");
  params[3] = server.arg("TrxAddress");

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

String rig_get_ptt() {
  return ptt_str;
}

void handleRPC() {
  // DEBUG Trace
  LOG2("=== Incoming RPC Request ===");
  LOG2(String("Client: ") + XMLRPCserver.client().remoteIP().toString());
  
  // HTTP Header-Daten
  for (int i = 0; i < XMLRPCserver.args(); i++) {
    LOG2(String("Arg ") + i + ": " + XMLRPCserver.argName(i));
    LOG2(String("Value: ") + XMLRPCserver.arg(i));
  }

  String request = XMLRPCserver.arg(0);
  LOG2("=== XML Request ===");
  LOG2(request);
  LOG2("===============================");

  // processing XML-RPC-request
  // === rig.get_vfo ===
  if (request.indexOf("<methodName>rig.get_vfo</methodName>") != -1) {
    String response = rig_get_vfo();
    String xmlResponse =
      "<?xml version=\"1.0\"?>"
      "<methodResponse>"
        "<params>"
          "<param>"
            "<value><string>" + response + "</string></value>"
          "</param>"
        "</params>"
      "</methodResponse>";
    XMLRPCserver.send(200, "text/xml", xmlResponse);
    LOG2(String("XMLRPC-Response (rig.get_vfo): ") + xmlResponse);
  }

  // === rig.get_mode ===
  else if (request.indexOf("<methodName>rig.get_mode</methodName>") != -1) {
    String response = mode_str;
    String xmlResponse =
      "<?xml version=\"1.0\"?>"
      "<methodResponse>"
        "<params>"
          "<param>"
            "<value><string>" + response + "</string></value>"
          "</param>"
        "</params>"
      "</methodResponse>";
    XMLRPCserver.send(200, "text/xml", xmlResponse);
    LOG2(String("XMLRPC-Response (rig.get_mode): ") + xmlResponse);
  }

  // === rig.get_ptt ===
  else if (request.indexOf("<methodName>rig.get_ptt</methodName>") != -1) {
    String response = ptt_state ? "1" : "0";
    String xmlResponse =
      "<?xml version=\"1.0\"?>"
      "<methodResponse>"
        "<params>"
          "<param>"
            "<value><boolean>" + response + "</boolean></value>"
            "<value><string>" + response + "</string></value>"   // Fallback for HAMClock
          "</param>"
        "</params>"
      "</methodResponse>";
    XMLRPCserver.send(200, "text/xml", xmlResponse);

    LOG2("----- XML-RPC Response Sent -----");
    LOG2(xmlResponse);
    LOG2("----- END Response -----");
  // === Unbekannte Methode ===
  }  else {
    XMLRPCserver.send(400, "text/plain", "Unknown method");
    LOG1("XMLRPC: Unknown method requested");
    return;
  }
}

// WebSocket event handler
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

  LOG1(String("URL: ") + params[0]);
  LOG1(String("Endpoint: ") + params[1]);
  LOG1(String("API-Key: ") + params[2]);
  LOG1(String("CI-V-Address: ") + params[3]);
  connectToWifi();

  LOG1(String("IP-Address: ") + WiFi.localIP().toString());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/trx", HTTP_GET, handleTrx);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/reboot", HTTP_POST, handleReboot);
  server.on("/logout", HTTP_GET, handleLogout, handleLoggedout);
  server.on("/logged-out", HTTP_GET, handleLoggedout);
  server.onNotFound(handle_NotFound);

  // Debug web page (shows last MAX_LOG_LINES log entries)
  server.on("/debug", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><title>ESP32 Debug Log</title>";
    html += "<style>body{font-family:monospace;background:#000;color:#0f0;padding:10px;white-space:pre-wrap}</style></head><body>";
    html += "<h3>ESP32 Debug Log (last " + String(MAX_LOG_LINES) + " lines)</h3>\n";
    int idx = debugIndex;
    for (int i = 0; i < MAX_LOG_LINES; ++i) {
      String line = debugLog[idx];
      if (line.length() > 0) html += line + "\n";
      idx = (idx + 1) % MAX_LOG_LINES;
    }
    html += "</body></html>";
    server.send(200, "text/html", html);
  });

  server.begin();
  LOG1("HTTP server started");

  // WebSocket started
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  LOG1("WebSocket server started on port 81");

  // Routing for XML-RPC-requests
  XMLRPCserver.on("/", handleRPC);
  XMLRPCserver.begin();
  LOG1("XMLRPCserver started");

  delay(1000);
  time_current_baseloop = millis();
  time_last_baseloop = time_current_baseloop;

  if (params[0].length() > 0) {
    getmode();
    if (params[3].toInt() >= 0) 
      geticomdata();
    if (newData2) {
      processReceivedData();
    }
    newData2 = false;
    delay(1000);
    getqrg();
    if (params[3].toInt() >= 0) 
      geticomdata();
    if (newData2) {
      processReceivedData();
    }
    newData2 = false;
    delay(1000);
    getptt();
    if (params[3].toInt() >= 0) 
      geticomdata();
    if (newData2) {
      processReceivedData();
    }
    newData2 = false;
    create_json(frequency, mode_str, ptt_str, power);
    post_json();
  }
}

// ---------------- Loop ----------------
void loop() {
  server.handleClient();
  XMLRPCserver.handleClient();
  webSocket.loop(); //  für WebSocket

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
    
    if (params[3].toInt() >= 0) 
      geticomdata();
    if (newData2) {
      time_last_update = millis();
      processReceivedData();
      newData2 = false;
    }
  // Debug-Output every second
    if (millis() - lastDebug > 1000) {
      LOG2(String("[DBG] QRG: ") + String(frequency) + " | MODE: " + mode_str + " | PTT: " + ptt_str + " | PWR: " + String(power));
      lastDebug = millis();
    }

    if (( mode_str != old_mode_str || frequency != old_frequency || ptt_str != old_ptt_str || power != old_power) && ((time_current_baseloop-time_last_update)>DEBOUNCE_TIME)) {
      // send json to server
      create_json(frequency, mode_str, ptt_str, power);
      post_json();
    }
  } else {
    LOG1("No connection to your WiFi-network.");
    connectToWifi();
  }
  delay(5);
} // end loop
