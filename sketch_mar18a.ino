// ================================================================
//  ESP32-S3 — CONTROLLER (CHỈ FIX LỖI GEMINI 400 & JOYSTICK)
//  - Fix chính tả API Google (inlineData, mimeType)
//  - 1 Click = Local AI, 2 Click = Gemini AI
// ================================================================
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <LoRa.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <driver/i2s.h>
#include <esp_now.h>
#include <hoang1901-project-1_inferencing.h>
#include <math.h>
#include <mbedtls/base64.h>
#include <qrcode.h>

char wifiSSID[64] = "Honor Magic6 Pro";
char wifiPASS[64] = "tamsotam";
#define GEMINI_API_KEY "AIzaSyAnirBSAQDfGsX1XHW7cpYUFfsXbD_SiWA"
#define AP_SSID "MEI-Controller"
#define AP_PASS "mei12345"
#define WIFI_SSID wifiSSID
#define WIFI_PASS wifiPASS

#define TELEGRAM_BOT_TOKEN                                                     \
  "8785859839:AAExccWXsA-k1ljx_CtbgBjGxF8dbk_kPEs" // Diem thong tin bot token
                                                   // cua ban
#define TELEGRAM_CHAT_ID "1471617872"              // Chat ID cua ban hoac Group

#define SPI_SCK 12
#define SPI_MOSI 11
#define SPI_MISO 13
SPIClass sharedSPI(FSPI);

#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 14
Adafruit_ST7789 tft(&sharedSPI, TFT_CS, TFT_DC, TFT_RST);

#define LORA_CS 15
#define LORA_RST 16
#define LORA_DIO0 17
#define LORA_FREQ 433E6

#define JOY_X 4
#define JOY_Y 5
#define JOY_SW 6
#define BTN_AI 21

#define I2S_WS_PIN 42
#define I2S_SCK_PIN 41
#define I2S_SD_PIN 2
#define I2S_PORT I2S_NUM_0
#define SAMPLE_RATE 16000

// Thời gian ghi âm tùy chỉnh riêng biệt
#define GEMINI_RECORD_SECS 1.5
#define LOCAL_RECORD_SECS 2.0
#define GEMINI_REC_BYTES ((int)(SAMPLE_RATE * GEMINI_RECORD_SECS * 2))
#define LOCAL_REC_BYTES ((int)(SAMPLE_RATE * LOCAL_RECORD_SECS * 2))
#define MAX_REC_BYTES                                                          \
  (LOCAL_REC_BYTES > GEMINI_REC_BYTES ? LOCAL_REC_BYTES : GEMINI_REC_BYTES)

enum AlertT { AN, APL, APH, ATDS, ATEMP, ANTU, ABAT };
volatile bool isTelegramSending = false;

#define HB_MS 300UL
#define CONN_LOST_MS 12000UL
#define ALERT_CD_MS 30000UL

// Cấu hình chuẩn Pin 2S
#define BAT_MAX 8.4f
#define BAT_MIN 6.8f
#define BAT_LO 7.2f
#define PH_LO 6.0f
#define PH_HI 9.0f
#define TDS_HI 1000
#define TEMP_HI 35.0f
#define NTU_HI 100

uint8_t BOAT_MAC[] = {0x98, 0xA3, 0x16, 0x9F, 0x09, 0xF0};

typedef struct {
  char lenh;
  int tocDo;
} Msg;
Msg txMsg;
typedef struct {
  float ph;
  int tds;
  float temp;
  int ntu;
  float heading, pitch, roll;
  int gpsStatus;
  float lat, lon, gpsSpeedKt, gpsCourse;
  float batVolt;
  int isReturning;
  uint32_t boatPing;
} SensorData;
SensorData rx;

volatile bool newData = false;
unsigned long lastRxMs = 0, lastRxLora = 0;
char curDir = 'G';
int curSpd = 0;
bool homeMode = false, loraOk = false, wifiOnline = false;
unsigned long lastAlertMs = 0;
int joyXc = 2048, joyYc = 2048;
String pendingAiResp = "";
bool hasNewAiResp = false;
bool aiPanelOn = false;
unsigned long aiPanelMs = 0;
enum AiState { AI_IDLE, AI_RECORDING, AI_PROCESSING, AI_SHOWING };
AiState aiState = AI_IDLE;
// ==== BỘ NHỚ LÕI TRÁNH PHÂN MẢNH MÃI MÃI ====
// Tránh dùng malloc/free trên ESP32 vì TLS handshake cần một mảng RAM
// contiguous 40KB.
uint8_t staticWavBuf[MAX_REC_BYTES + 44 + 4];
int8_t *recBuf = (int8_t *)(staticWavBuf + 44);
char staticPayloadBuf[68000]; // 68KB đủ sức chứa Base64 của 1.5s Audio
// ============================================
volatile bool triggerWifiScan = false;

void onRecv(const uint8_t *, const uint8_t *d, int) {
  memcpy(&rx, d, sizeof(rx));
  newData = true;
  lastRxMs = millis();
}
bool connected() {
  return max(lastRxMs, lastRxLora) > 0 &&
         millis() - max(lastRxMs, lastRxLora) < CONN_LOST_MS;
}

uint16_t CBG, CHDR, CDIV, CDIM, CCYAN, CAMBER, CGREEN, CRED, CMUTED, CWHITE,
    CORANGE, CTEAL, CPURPLE, CNAVY;
void initColors() {
  CBG = tft.color565(3, 6, 18);
  CHDR = tft.color565(6, 11, 28);
  CDIV = tft.color565(20, 34, 66);
  CDIM = tft.color565(12, 20, 44);
  CNAVY = tft.color565(8, 18, 50);
  CCYAN = tft.color565(0, 210, 255);
  CAMBER = tft.color565(255, 180, 0);
  CGREEN = tft.color565(40, 220, 100);
  CRED = tft.color565(235, 50, 50);
  CMUTED = tft.color565(60, 85, 125);
  CWHITE = tft.color565(220, 230, 245);
  CORANGE = tft.color565(255, 115, 25);
  CTEAL = tft.color565(0, 190, 175);
  CPURPLE = tft.color565(170, 80, 255);
}

#define HDR_H 22
#define LW 148
#define DIV_X (LW + 1)
#define RX_S (LW + 3)
#define RW (320 - LW - 3)
#define N_CARDS 4
#define CARD_H ((240 - HDR_H) / N_CARDS)
#define DIR_Y HDR_H
#define DIR_H_ 24
#define COMP_Y (DIR_Y + DIR_H_)
#define COMP_H_ 86
#define GPS_Y (COMP_Y + COMP_H_)
#define GPS_H_ 34
#define SPD_Y (GPS_Y + GPS_H_)
#define SPD_H_ 24
#define BAT_Y_ (SPD_Y + SPD_H_)
#define BAT_H_ (240 - BAT_Y_ - 1)
#define CCX (RX_S + RW / 2)
#define CCY (COMP_Y + COMP_H_ / 2 + 2)
#define CR 36

AlertT curAlert = AN;
AlertT chkAlert() {
  if (!connected())
    return AN;
  if (rx.ph > 0 && rx.ph < 14 && rx.ph < PH_LO)
    return APL;
  if (rx.ph > 0 && rx.ph < 14 && rx.ph > PH_HI)
    return APH;
  if (rx.batVolt > 5 && rx.batVolt < 15 && rx.batVolt < BAT_LO)
    return ABAT;
  if (rx.tds > 0 && rx.tds < 50000 && rx.tds > TDS_HI)
    return ATDS;
  if (rx.temp > 0 && rx.temp < 60 && rx.temp > TEMP_HI)
    return ATEMP;
  if (rx.ntu > 0 && rx.ntu < 10000 && rx.ntu > NTU_HI)
    return ANTU;
  return AN;
}
const char *aKey(AlertT t) {
  switch (t) {
  case APL:
    return "pH_THAP";
  case APH:
    return "pH_CAO";
  case ATDS:
    return "TDS_CAO";
  case ATEMP:
    return "NHIET_DO_CAO";
  case ANTU:
    return "DO_DUC_CAO";
  case ABAT:
    return "PIN_YEU";
  default:
    return "";
  }
}
const char *aLbl(AlertT t) {
  switch (t) {
  case APL:
    return "pH QUA THAP";
  case APH:
    return "pH QUA CAO";
  case ATDS:
    return "TDS QUA CAO";
  case ATEMP:
    return "NHIET DO CAO";
  case ANTU:
    return "DO DUC CAO";
  case ABAT:
    return "PIN YEU!";
  default:
    return "CANH BAO";
  }
}
uint16_t aColor(AlertT t) { return (t == ABAT || t == APL) ? CAMBER : CRED; }

String ruleMsg(AlertT t) {
  switch (t) {
  case APL:
    return "pH=" + String(rx.ph, 2) + " THAP\nTDS cao: axit?\nCHU Y khu vuc";
  case APH:
    return "pH=" + String(rx.ph, 2) + " CAO\nNhiet cao: tao?\nNen kiem tra";
  case ATDS:
    return "TDS=" + String(rx.tds) + "ppm\nDo duc cao\nNGUY HIEM";
  case ATEMP:
    return "Nhiet=" + String(rx.temp, 1) + "C\nVi khuan phat trien";
  case ANTU:
    return "Duc=" + String(rx.ntu) + "NTU\nSau mua/bun";
  case ABAT:
    return "Pin=" + String(rx.batVolt, 1) + "V THAP!\nHay dieu thuyen ve!";
  default:
    return "";
  }
}

String sensorJson() {
  char b[700];
  snprintf(
      b, 700,
      "{\"ph\":%.2f,\"tds\":%d,\"temp\":%.1f,\"ntu\":%d,\"heading\":%.1f,"
      "\"lat\":%.6f,\"lon\":%.6f,\"gps_fix\":%d,\"speed_kmh\":%.2f,\"course\":%"
      ".1f,\"bat_volt\":%.2f,\"bat_pct\":%d,\"returning\":%d,\"connected\":%d,"
      "\"lora_ok\":%d,\"wifi_on\":%d,"
      "\"alert\":\"%s\"}",
      rx.ph, rx.tds, rx.temp, rx.ntu, rx.heading, rx.lat, rx.lon, rx.gpsStatus,
      rx.gpsSpeedKt * 1.852f, rx.gpsCourse, rx.batVolt,
      (int)(constrain((rx.batVolt - BAT_MIN) / (BAT_MAX - BAT_MIN), 0.f, 1.f) *
            100),
      rx.isReturning, connected() ? 1 : 0, loraOk ? 1 : 0, wifiOnline ? 1 : 0,
      aKey(curAlert));
  return String(b);
}

AsyncWebServer webSrv(80);
void processLocalAI(String text);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>MEI Dashboard</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',sans-serif;background:#060d1f;color:#e0e8f0;padding:12px}
h1{color:#00d2ff;font-size:22px;margin-bottom:2px;text-align:center}
.sub{color:#6a7fa0;font-size:12px;text-align:center;margin-bottom:12px}
.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:8px;max-width:400px;margin:0 auto}
.c{background:#0f1b35;border-radius:10px;padding:12px;position:relative;overflow:hidden}
.c::before{content:'';position:absolute;left:0;top:0;bottom:0;width:4px}
.c.ph::before{background:#28dc64}.c.tds::before{background:#00d2ff}
.c.tmp::before{background:#ffb400}.c.ntu::before{background:#ff7319}
.c.bat::before{background:#aa50ff}.c.spd::before{background:#00beaf}
.c h3{color:#6a7fa0;font-size:11px;font-weight:500}
.c .v{font-size:26px;font-weight:700;margin-top:4px}
.c .u{color:#4a5a70;font-size:10px}
.conn{display:flex;justify-content:center;gap:14px;margin:10px 0;flex-wrap:wrap}
.dot{display:flex;align-items:center;gap:4px;font-size:11px;color:#6a7fa0}
.dot i{width:8px;height:8px;border-radius:50%;display:inline-block}
.gps{background:#0f1b35;border-radius:10px;padding:10px;margin:10px auto;max-width:400px;font-size:12px;color:#6a7fa0;text-align:center}
.gps .coord{color:#00d2ff;font-size:16px;font-weight:600;margin:4px 0}
.alert{background:#eb3232;padding:10px;margin:10px auto;border-radius:8px;display:none;font-weight:bold;max-width:400px;text-align:center}
hr{border:none;border-top:1px solid #1a2540;margin:18px 0}
.wf{background:linear-gradient(145deg,#112040,#0a1428);border:1px solid #1a2c5b;box-shadow:0 8px 16px rgba(0,0,0,0.4);border-radius:12px;padding:18px;max-width:400px;margin:10px auto}
.wf h3{color:#fff;font-size:16px;margin-bottom:12px;display:flex;align-items:center;gap:8px}
.wf-status{background:rgba(0,0,0,0.2);padding:8px 12px;border-radius:6px;margin-bottom:12px;font-size:12px;color:#a0aec0;display:flex;justify-content:space-between}
.wf-status span{font-weight:600;color:#28dc64}
.wf input,.wf select{background:#0f1b35;border:1px solid #23355a;color:#fff;border-radius:8px;padding:12px;font-size:14px;width:100%;margin-bottom:10px;transition:all 0.3s;outline:none}
.wf input:focus{border-color:#00d2ff;box-shadow:0 0 0 2px rgba(0,210,255,0.2)}
.wf button{background:linear-gradient(135deg,#00d2ff,#0070ff);border:none;border-radius:8px;padding:12px;color:#fff;font-size:14px;font-weight:600;width:100%;cursor:pointer;transition:transform 0.1s}
.wf button:active{transform:scale(0.97)}
.wf .btn-scan{background:#1a2c5b;color:#a0aec0;margin-bottom:12px}
#wifiMsg{text-align:center;margin-top:12px;font-size:13px;font-weight:500;min-height:18px}
</style></head>
<body>
<h1>MEI DASHBOARD</h1>
<p class="sub">Live Boat Tracking & Telemetry</p>
<div id="alertBox" class="alert">CANH BAO: <span id="alertMsg"></span></div>
<div class="conn">
<div class="dot"><i id="dEnow" style="background:#f44"></i>ESPNOW</div>
<div class="dot"><i id="dLora" style="background:#f44"></i>LORA</div>
<div class="dot"><i id="dWifi" style="background:#f44"></i>WIFI</div>
<div class="dot"><i id="dGps" style="background:#555"></i>GPS</div>
</div>
<div class="grid">
<div class="c ph"><h3>pH Level</h3><div class="v" id="ph">--</div></div>
<div class="c tds"><h3>TDS</h3><div class="v"><span id="tds">--</span> <span class="u">ppm</span></div></div>
<div class="c tmp"><h3>Temperature</h3><div class="v"><span id="temp">--</span> <span class="u">&deg;C</span></div></div>
<div class="c ntu"><h3>Turbidity</h3><div class="v"><span id="ntu">--</span> <span class="u">NTU</span></div></div>
<div class="c bat"><h3>Boat Battery</h3><div class="v"><span id="bat">--</span> <span class="u">V</span></div></div>
<div class="c spd"><h3>Speed</h3><div class="v"><span id="spd">--</span> <span class="u">km/h</span></div></div>
</div>
<div class="gps"><b>GPS Position</b><div class="coord" id="gpsCoord">-- / --</div><div id="gpsSt">Searching...</div></div>
<hr>
<div class="wf">
<h3>🌐 Thiết lập cấu hình WiFi</h3>
<div class="wf-status">Trạng thái kết nối: <span id="wifiSt">Offline</span></div>
<button class="btn-scan" onclick="scanWifi()">🔍 Quét tìm mạng WiFi</button>
<select id="scanList" style="display:none" onchange="document.getElementById('ssid').value=this.value"><option value="">-- Chọn mạng có sẵn --</option></select>
<input id="ssid" placeholder="Tên mạng WiFi (SSID)">
<input id="pass" type="password" placeholder="Mật khẩu WiFi">
<button onclick="saveWifi()">🚀 Lưu và Kết Nối Interet</button>
<div id="wifiMsg"></div>
</div>
<script>
function scanWifi(){
  document.getElementById('wifiMsg').style.color='#a0aec0';
  document.getElementById('wifiMsg').innerText='Đang quét mạng (vài giây)...';
  fetch('/scan').then(()=>{
    var iv = setInterval(()=>{
      fetch('/scanresults').then(r=>r.json()).then(res=>{
        if(res.status==='done'){
          clearInterval(iv);
          var sel=document.getElementById('scanList');
          sel.innerHTML='<option value="">-- Chọn sóng mạng ('+res.list.length+') --</option>';
          res.list.forEach(n=>{var o=document.createElement('option');o.value=n.ssid;o.textContent=n.ssid+' ('+n.rssi+'dBm)';sel.appendChild(o);});
          sel.style.display='block';
          document.getElementById('wifiMsg').innerText='✅ Tìm thấy '+res.list.length+' mạng WiFi!';
        }
      });
    }, 1000);
  }).catch(e=>{document.getElementById('wifiMsg').innerText='Lỗi quét!';});
}
function saveWifi(){
  var s=document.getElementById('ssid').value,p=document.getElementById('pass').value;
  if(!s){
    document.getElementById('wifiMsg').style.color='#f44';
    document.getElementById('wifiMsg').innerText='⚠️ Vui lòng nhập SSID!';
    return;
  }
  document.getElementById('wifiMsg').style.color='#00d2ff';
  document.getElementById('wifiMsg').innerText='⏳ Đang kết nối ngầm, vui lòng đợi...';
  fetch('/wifi',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(p)}).then(()=>{
    var iv = setInterval(()=>{
      fetch('/wifi_status').then(r=>r.text()).then(st=>{
         if(st==='failed') {
            document.getElementById('wifiMsg').style.color='#f44';
            document.getElementById('wifiMsg').innerText='❌ Sai mạng/mật khẩu! Đã dừng và lui về an toàn.';
            clearInterval(iv);
         } else if(st==='connected' || (st==='idle' && document.getElementById('wifiSt').innerText==='Online')) {
            document.getElementById('wifiMsg').style.color='#28dc64';
            document.getElementById('wifiMsg').innerText='✅ Đã truy cập mạng thành công!';
            clearInterval(iv);
         }
      });
    }, 2000);
  });
}
setInterval(function(){
  fetch('/data').then(r=>r.json()).then(d=>{
    document.getElementById('ph').innerText=d.ph>0?d.ph.toFixed(2):'--';
    document.getElementById('tds').innerText=d.tds>0?d.tds:'--';
    document.getElementById('temp').innerText=d.temp>0?d.temp.toFixed(1):'--';
    document.getElementById('ntu').innerText=d.ntu>0?d.ntu:'--';
    document.getElementById('bat').innerText=d.bat_volt>0?d.bat_volt.toFixed(1):'--';
    document.getElementById('spd').innerText=d.speed_kmh.toFixed(1);
    document.getElementById('dEnow').style.background=d.connected?'#28dc64':'#f44';
    document.getElementById('dLora').style.background=d.lora_ok?'#28dc64':'#f44';
    document.getElementById('dWifi').style.background=d.wifi_on?'#28dc64':'#f44';
    document.getElementById('dGps').style.background=d.gps_fix>=2?'#28dc64':(d.gps_fix==1?'#ffb400':'#555');
    document.getElementById('wifiSt').innerText=d.wifi_on?'Online':'Offline';
    if(d.lat!==0&&d.lon!==0){document.getElementById('gpsCoord').innerText=d.lat.toFixed(6)+' / '+d.lon.toFixed(6);document.getElementById('gpsSt').innerText='Fix: '+(d.gps_fix>=2?'DGPS':'FIX');}
    else{document.getElementById('gpsCoord').innerText='-- / --';document.getElementById('gpsSt').innerText='Searching...';}
    var a=document.getElementById('alertBox');
    if(d.alert!==''){a.style.display='block';document.getElementById('alertMsg').innerText=d.alert;}else a.style.display='none';
  }).catch(e=>{});
},500);
</script></body></html>)rawliteral";

QueueHandle_t telegramQueue;

void telegramTask(void *pvParameters) {
  while (1) {
    char msgBuf[512];
    if (xQueueReceive(telegramQueue, &msgBuf, portMAX_DELAY)) {
      // ĐỢI GEMINI DÙNG XONG RAM TRƯỚC KHI GỬI TELEGRAM ĐỂ CHỐNG TRÀN RAM DÂY
      // CHUYỀN
      while (aiState != AI_IDLE) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }

      isTelegramSending = true;
      if (!wifiOnline || String(TELEGRAM_BOT_TOKEN) == "YOUR_BOT_TOKEN_HERE") {
        isTelegramSending = false;
        continue;
      }
      String message = String(msgBuf);
      message.replace(" ", "%20");
      message.replace("\n", "%0A");

      WiFiClientSecure client;
      client.setInsecure();
      HTTPClient http;
      http.setTimeout(5000); // TIMEOUT CẤP TỐC CHỐNG LƯU PHIÊN
      String url = "https://api.telegram.org/bot" + String(TELEGRAM_BOT_TOKEN) +
                   "/sendMessage?chat_id=" + String(TELEGRAM_CHAT_ID) +
                   "&text=" + message;
      http.begin(client, url);
      int code = http.GET();
      if (code > 0)
        Serial.printf("[Telegram] Gui canh bao (Code: %d)\n", code);
      else
        Serial.printf("[Telegram] Loi HTTP: %d\n", code);
      http.end();
      isTelegramSending = false;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Tiết kiệm đáng kể RAM bằng cách vô hiệu hóa Telegram Task (Vốn chiếm ~50KB
// RAM mỗi lần gọi)
void sendTelegramAlert(String message) {
  if (telegramQueue) {
    char msgBuf[512];
    strncpy(msgBuf, message.c_str(), 511);
    msgBuf[511] = '\0';
    xQueueSend(telegramQueue, &msgBuf,
               0); // Không chờ đợi, nhường RAM cho Web/Màn hình
  }
}

volatile bool triggerWifiConnect = false;
enum WifiState { WF_IDLE, WF_CONNECTING, WF_CONNECTED, WF_FAILED };
WifiState wfState = WF_IDLE;
uint32_t wfAttemptMs = 0;

void startWeb() {
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers",
                                       "Content-Type");
  webSrv.on("/", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send_P(200, "text/html", index_html);
  });
  webSrv.on("/data", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send(200, "application/json", sensorJson());
  });
  webSrv.on("/wifi", HTTP_POST, [](AsyncWebServerRequest *r) {
    if (r->hasParam("ssid", true)) {
      String newSSID = r->getParam("ssid", true)->value();
      String newPASS =
          r->hasParam("pass", true) ? r->getParam("pass", true)->value() : "";
      strncpy(wifiSSID, newSSID.c_str(), 63);
      wifiSSID[63] = '\0';
      strncpy(wifiPASS, newPASS.c_str(), 63);
      wifiPASS[63] = '\0';

      triggerWifiConnect = true;
      r->send(200, "text/plain", "Đang thử kết nối... Xem trạng thái ở trên!");
    } else {
      r->send(400, "text/plain", "Thiếu SSID!");
    }
  });

  webSrv.on("/wifi_status", HTTP_GET, [](AsyncWebServerRequest *r) {
    if (wfState == WF_CONNECTING)
      r->send(200, "text/plain", "connecting");
    else if (wfState == WF_FAILED)
      r->send(200, "text/plain", "failed");
    else if (wfState == WF_CONNECTED)
      r->send(200, "text/plain", "connected");
    else
      r->send(200, "text/plain", "idle");
  });

  webSrv.on("/scan", HTTP_GET, [](AsyncWebServerRequest *r) {
    // SỬA LỖI CRASH ESP32-S3: Chuyển lệnh quét WiFi ra vòng lặp loop() chính
    // Tuyệt đối không gọi WiFi.scanNetworks từ trong LwIP callback
    triggerWifiScan = true;
    r->send(200, "text/plain", "SCANNING");
  });
  webSrv.on("/scanresults", HTTP_GET, [](AsyncWebServerRequest *r) {
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) {
      r->send(200, "application/json", "{\"status\":\"scanning\"}");
      return;
    }
    if (n == WIFI_SCAN_FAILED || n < 0) {
      r->send(200, "application/json", "{\"status\":\"done\",\"list\":[]}");
      return;
    }
    String json = "{\"status\":\"done\",\"list\":[";
    for (int i = 0; i < n; i++) {
      if (i > 0)
        json += ",";
      json += "{\"ssid\":\"" + WiFi.SSID(i) +
              "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
    }
    json += "]}";
    WiFi.scanDelete();
    r->send(200, "application/json", json);
  });
  webSrv.begin();
}

void hl(int x, int y, int w, uint16_t c, int t = 1) {
  for (int i = 0; i < t; i++)
    tft.drawFastHLine(x, y + i, w, c);
}
void vl(int x, int y, int h2, uint16_t c, int t = 1) {
  for (int i = 0; i < t; i++)
    tft.drawFastVLine(x + i, y, h2, c);
}
void bar(int x, int y, int w, int h2, float p, uint16_t c) {
  tft.fillRect(x, y, w, h2, CDIM);
  int f = (int)(w * constrain(p, 0.f, 1.f));
  if (f > 0)
    tft.fillRect(x, y, f, h2, c);
}

struct Cache {
  float ph = -99, temp = -99, batV = -1, spdKt = -1, crs = -1, lat = -999;
  int tds = -1, ntu = -1, gpsFix = -1, batPct = -1, spd = -1;
  int cnEnow = -1, cnLora = -1, hm = -1;
  char dir = ' ';
} cache;

void drawHdrStatic() {
  tft.fillRect(0, 0, 320, HDR_H, CHDR);
  hl(0, 0, 320, CCYAN, 2);

  tft.setTextSize(2);
  tft.setTextColor(CCYAN, CHDR);
  tft.setCursor(5, 4);
  tft.print("MEI");

  tft.setTextSize(1);
  tft.setTextColor(CMUTED, CHDR);
  tft.setCursor(95, 4);
  tft.print("ENOW");

  tft.setCursor(132, 4);
  tft.print("LORA");

  tft.setCursor(170, 4);
  tft.print("GPS");

  tft.setCursor(200, 4);
  tft.print("HOME");

  tft.setCursor(240, 4);
  tft.print("BAT");

  tft.setCursor(275, 4);
  tft.print("WIFI");

  tft.setCursor(305, 4);
  tft.print("MIC");
}

void updHdr(bool cnEnow, bool cnLora, int gf, bool h, float bV, bool ap) {
  if (cnEnow != (bool)cache.cnEnow) {
    cache.cnEnow = cnEnow;
    tft.fillCircle(107, 14, 4, cnEnow ? CGREEN : CRED);
  }
  if (cnLora != (bool)cache.cnLora) {
    cache.cnLora = cnLora;
    tft.fillCircle(144, 14, 4, cnLora ? CGREEN : CRED);
  }
  if (gf != cache.gpsFix) {
    cache.gpsFix = gf;
    tft.fillCircle(179, 14, 4,
                   (gf >= 2) ? CGREEN : (gf == 1 ? CAMBER : CMUTED));
  }
  if (h != (bool)cache.hm) {
    cache.hm = h;
    tft.fillRect(205, 14, 14, 6, CHDR);
    if (h) {
      tft.fillRect(205, 14, 8, 6, CAMBER);
      tft.fillTriangle(204, 14, 209, 9, 214, 14, CAMBER);
    } else {
      tft.fillRect(205, 14, 8, 6, CDIM);
      tft.fillTriangle(204, 14, 209, 9, 214, 14, CDIM);
    }
  }
  int pct =
      (int)(constrain((bV - BAT_MIN) / (BAT_MAX - BAT_MIN), 0.f, 1.f) * 100);
  if (pct != cache.batPct) {
    cache.batPct = pct;
    uint16_t bc = pct > 50 ? CGREEN : pct > 20 ? CAMBER : CRED;
    tft.fillRect(238, 9, 20, 12, CHDR);
    tft.drawRect(238, 9, 18, 9, CMUTED);
    tft.fillRect(239, 10, 16, 7, CDIM);
    int fw = (int)(16.f * pct / 100);
    if (fw > 0)
      tft.fillRect(239, 10, fw, 7, bc);
  }

  // WiFI Connection Dot
  tft.fillCircle(287, 14, 4, wifiOnline ? CGREEN : CRED);

  // MIC Dot
  uint16_t mc = (aiState == AI_RECORDING)    ? CRED
                : (aiState == AI_PROCESSING) ? CAMBER
                                             : CMUTED;
  tft.fillCircle(314, 14, 4, mc);
  if (aiState == AI_RECORDING)
    tft.drawCircle(314, 14, 6, CRED);
}

const char *CLBL[] = {"pH", "TDS", "TEMP", "NTU"};
const char *UNIT[] = {"", "ppm",
                      "\xF8"
                      "C",
                      ""};
float CMAX[] = {14, 2000, 50, 3000};
uint16_t cardAccent(int i) {
  return (i == 0) ? CGREEN : (i == 1) ? CCYAN : (i == 2) ? CORANGE : CAMBER;
}

bool isWarn(int i, float v) {
  if (!connected())
    return false;
  if (i == 0)
    return v > 0 && v < 14 && (v < PH_LO || v > PH_HI);
  if (i == 1)
    return v > 0 && v < 50000 && v > TDS_HI;
  if (i == 2)
    return v > 0 && v < 60 && v > TEMP_HI;
  if (i == 3)
    return v > 0 && v < 10000 && v > NTU_HI;
  return false;
}

void drawCardBg(int i) {
  int y0 = HDR_H + i * CARD_H;
  uint16_t ac = cardAccent(i);
  tft.fillRect(0, y0, LW, CARD_H, CBG);
  tft.fillRect(0, y0 + 2, 3, CARD_H - 4, ac);
  tft.fillRect(3, y0 + 2, 1, CARD_H - 4,
               tft.color565(((ac >> 11) & 0x1F) * 4, ((ac >> 5) & 0x3F) * 2,
                            (ac & 0x1F) * 4));
  hl(0, y0 + CARD_H - 1, LW, CDIV);
  tft.setTextSize(1);
  tft.setTextColor(CMUTED, CBG);
  tft.setCursor(7, y0 + 4);
  tft.print(CLBL[i]);
}

void updCard(int i, float v) {
  int y0 = HDR_H + i * CARD_H;
  bool everReceived = (lastRxMs > 0 || lastRxLora > 0);
  bool inRange =
      (i == 0 && v >= 0 && v <= 14) || (i == 1 && v >= 0 && v < 50000) ||
      (i == 2 && v > -50 && v < 100) || (i == 3 && v >= 0 && v < 10000);
  bool valid = everReceived && inRange;
  bool w = valid && connected() && isWarn(i, v);
  uint16_t ac = cardAccent(i);
  uint16_t vc = w ? CRED : (valid ? CWHITE : CMUTED);
  uint16_t bc = w ? CRED : ac;

  tft.fillRect(4, y0 + 12, LW - 5, CARD_H - 19, CBG);
  tft.setTextSize(2);
  tft.setTextColor(vc, CBG);
  tft.setCursor(7, y0 + 14);
  if (!valid)
    tft.print("--");
  else {
    if (i == 0)
      tft.print(v, 2);
    else if (i == 2)
      tft.print(v, 1);
    else
      tft.print((int)v);
  }
  tft.setTextSize(1);
  tft.setTextColor(w ? CRED : (valid ? ac : CMUTED), CBG);
  tft.setCursor(70, y0 + 21);
  tft.print(UNIT[i]);
  bar(4, y0 + CARD_H - 6, LW - 5, 5, valid ? (v / CMAX[i]) : 0.f, bc);
  tft.fillRect(LW - 13, y0 + 3, 12, 9, CBG);
  if (w) {
    tft.setTextColor(CRED, CBG);
    tft.setTextSize(1);
    tft.setCursor(LW - 11, y0 + 4);
    tft.print("!");
  }
}

void drawCompassRing() {
  tft.drawCircle(CCX, CCY, CR + 6, CDIV);
  tft.drawCircle(CCX, CCY, CR + 4, CDIV);
  tft.drawCircle(CCX, CCY, CR + 1, CMUTED);
  for (int d = 0; d < 360; d += 45) {
    float r = d * DEG_TO_RAD;
    bool major = (d % 90 == 0);
    int r0 = CR - (major ? 9 : 5);
    tft.drawLine(CCX + (int)(r0 * sinf(r)), CCY - (int)(r0 * cosf(r)),
                 CCX + (int)(CR * sinf(r)), CCY - (int)(CR * cosf(r)),
                 major ? CCYAN : CMUTED);
  }
  tft.setTextSize(1);
  tft.setTextColor(CCYAN, CBG);
  tft.setCursor(CCX - 3, CCY - CR + 2);
  tft.print("N");
  tft.setTextColor(CMUTED, CBG);
  tft.setCursor(CCX - 3, CCY + CR - 10);
  tft.print("S");
  tft.setCursor(CCX - CR + 2, CCY - 4);
  tft.print("W");
  tft.setCursor(CCX + CR - 8, CCY - 4);
  tft.print("E");
}

void updCompassNeedle(float heading, char fallbackDir) {
  static float prevH = -999;
  float drawH = heading;
  if (drawH < 0 || drawH >= 360) {
    if (fallbackDir == 'W')
      drawH = 0.0f;
    else if (fallbackDir == 'D')
      drawH = 90.0f;
    else if (fallbackDir == 'S' || fallbackDir == 'L')
      drawH = 180.0f;
    else if (fallbackDir == 'A')
      drawH = 270.0f;
    else
      drawH = 0.0f;
  }

  if (fabsf(drawH - prevH) < 1.5f)
    return;
  prevH = drawH;

  tft.fillCircle(CCX, CCY, CR - 3, CBG);
  float rad = drawH * DEG_TO_RAD, sr = sinf(rad), cr2 = cosf(rad);
  int nx = CCX + (int)((CR - 6) * sr), ny = CCY - (int)((CR - 6) * cr2),
      tx = CCX - (int)((CR / 3) * sr), ty = CCY + (int)((CR / 3) * cr2);
  float pr = (drawH + 90) * DEG_TO_RAD;
  int lx = CCX + (int)(5 * sinf(pr)), ly = CCY - (int)(5 * cosf(pr)),
      rx2 = CCX - (int)(5 * sinf(pr)), ry2 = CCY + (int)(5 * cosf(pr));
  tft.fillTriangle(tx, ty, lx, ly, rx2, ry2, CWHITE);
  tft.fillTriangle(nx, ny, lx, ly, rx2, ry2, CRED);
  tft.fillCircle(CCX, CCY, 4, CAMBER);
  tft.fillCircle(CCX, CCY, 2, CBG);
  tft.fillRect(CCX - 14, CCY + CR - 16, 28, 10, CBG);
  tft.setTextColor(CCYAN, CBG);
  tft.setTextSize(1);
  tft.setCursor(CCX - 12, CCY + CR - 15);
  char hbuf[8];
  if (heading >= 0 && heading < 360)
    snprintf(hbuf, 8, "%3.0f\xF8", drawH);
  else
    snprintf(hbuf, 8, "---"); // Báo hiệu la bàn ảo
  tft.print(hbuf);
}

void updDir(char d, int s, bool h) {
  if (d == cache.dir && s == cache.spd && h == (bool)cache.hm)
    return;
  if (aiPanelOn)
    return;
  cache.dir = d;
  cache.spd = s;
  cache.hm = h;
  tft.fillRect(RX_S, DIR_Y, RW, DIR_H_, CBG);
  if (h) {
    tft.fillRoundRect(RX_S + 2, DIR_Y + 2, RW - 4, DIR_H_ - 4, 3, CAMBER);
    tft.setTextColor(CBG, CAMBER);
    tft.setTextSize(1);
    tft.setCursor(RX_S + 8, DIR_Y + 8);
    tft.print("\x10 RETURNING HOME \x11");
    return;
  }
  const char *DN[] = {"FORWARD", "REVERSE", " LEFT  ", " RIGHT ", "STOPPED"};
  int idx = 4;
  if (d == 'W')
    idx = 0;
  else if (d == 'L')
    idx = 1;
  else if (d == 'A')
    idx = 2;
  else if (d == 'D')
    idx = 3;
  uint16_t dc[] = {CGREEN, CRED, CAMBER, CAMBER, CMUTED};
  tft.setTextColor(dc[idx], CBG);
  tft.setTextSize(1);
  tft.setCursor(RX_S + 4, DIR_Y + 4);
  tft.print(DN[idx]);
  tft.setTextColor(CMUTED, CBG);
  tft.setCursor(RX_S + 4, DIR_Y + 14);
  tft.print("SPD");
  bar(RX_S + 22, DIR_Y + 15, 72, 5, s / 255.f, dc[idx]);
  tft.setTextColor(CWHITE, CBG);
  tft.setCursor(RX_S + 98, DIR_Y + 14);
  char b[5];
  snprintf(b, 5, "%-3d", s);
  tft.print(b);
}

void updGPS(float lat, float lon, int fix) {
  if (fix == cache.gpsFix && fabsf(lat - cache.lat) < 0.00001f)
    return;
  cache.lat = lat;
  cache.gpsFix = fix;
  if (aiPanelOn)
    return;
  tft.fillRect(RX_S, GPS_Y, RW, GPS_H_, CBG);
  hl(RX_S, GPS_Y, RW, CDIV);
  uint16_t fc = fix > 0 ? CGREEN : CRED;
  const char *fs = (fix == 2) ? "DGPS" : (fix == 1) ? " FIX" : "----";
  tft.fillRoundRect(RX_S + 2, GPS_Y + 2, 28, 9, 2, fc);
  tft.setTextColor(CBG, fc);
  tft.setTextSize(1);
  tft.setCursor(RX_S + 4, GPS_Y + 4);
  tft.print(fs);
  if (fix > 0) {
    char buf[12];
    tft.setTextColor(CMUTED, CBG);
    tft.setCursor(RX_S + 2, GPS_Y + 14);
    tft.print("LA:");
    tft.setTextColor(CWHITE, CBG);
    dtostrf(fabsf(lat), 8, 4, buf);
    tft.print(buf);
    tft.setTextColor(lat >= 0 ? CGREEN : CRED, CBG);
    tft.print(lat >= 0 ? " N" : " S");
    tft.setTextColor(CMUTED, CBG);
    tft.setCursor(RX_S + 2, GPS_Y + 23);
    tft.print("LO:");
    tft.setTextColor(CWHITE, CBG);
    dtostrf(fabsf(lon), 8, 4, buf);
    tft.print(buf);
    tft.setTextColor(lon >= 0 ? CGREEN : CRED, CBG);
    tft.print(lon >= 0 ? " E" : " W");
  } else {
    tft.setTextColor(CMUTED, CBG);
    tft.setCursor(RX_S + 2, GPS_Y + 14);
    tft.print("Searching GPS...   ");
  }
}

void updSpd(float kt, float crs) {
  if (fabsf(kt - cache.spdKt) < 0.05f && fabsf(crs - cache.crs) < 0.5f)
    return;
  cache.spdKt = kt;
  cache.crs = crs;
  if (aiPanelOn)
    return;
  tft.fillRect(RX_S, SPD_Y, RW, SPD_H_, CBG);
  hl(RX_S, SPD_Y, RW, CDIV);
  char b[10];
  tft.setTextColor(CMUTED, CBG);
  tft.setTextSize(1);
  tft.setCursor(RX_S + 2, SPD_Y + 3);
  tft.print("SPD");
  tft.setTextColor(CCYAN, CBG);
  tft.setTextSize(2);
  dtostrf(kt * 1.852f, 4, 1, b);
  char *p = b;
  while (*p == ' ')
    p++;
  tft.setCursor(RX_S + 24, SPD_Y + 2);
  tft.print(p);
  tft.setTextColor(CMUTED, CBG);
  tft.setTextSize(1);
  tft.setCursor(RX_S + 76, SPD_Y + 8);
  tft.print("km/h");
  tft.setTextColor(CAMBER, CBG);
  tft.setCursor(RX_S + 2, SPD_Y + 16);
  tft.print("CRS");
  tft.setTextColor(CWHITE, CBG);
  tft.setCursor(RX_S + 22, SPD_Y + 16);
  snprintf(b, 9, "%3.0f\xF8", crs);
  tft.print(b);
}

void updBat(float bV) {
  int pct =
      (int)(constrain((bV - BAT_MIN) / (BAT_MAX - BAT_MIN), 0.f, 1.f) * 100);
  if (pct == cache.batPct && fabsf(bV - cache.batV) < 0.05f)
    return;
  cache.batV = bV;
  cache.batPct = pct;
  if (aiPanelOn)
    return;
  tft.fillRect(RX_S, BAT_Y_, RW, BAT_H_, CBG);
  hl(RX_S, BAT_Y_, RW, CDIV);
  uint16_t bc = pct > 50 ? CGREEN : pct > 20 ? CAMBER : CRED;
  tft.setTextColor(CMUTED, CBG);
  tft.setTextSize(1);
  tft.setCursor(RX_S + 2, BAT_Y_ + 3);
  tft.print("BOAT BAT");
  int bx = RX_S + 2, bw = RW - 5;
  tft.drawRect(bx, BAT_Y_ + 13, bw, 11, CMUTED);
  tft.fillRect(bx + 1, BAT_Y_ + 14, bw - 2, 9, CDIM);
  int fw = (int)((bw - 2) * pct / 100);
  if (fw > 0)
    tft.fillRect(bx + 1, BAT_Y_ + 14, fw, 9, bc);
  char buf[6];
  snprintf(buf, 6, "%3d%%", pct);
  tft.setTextColor(pct > 20 ? CBG : bc, pct > 20 ? bc : CDIM);
  tft.setCursor(bx + (bw / 2) - 12, BAT_Y_ + 15);
  tft.print(buf);
  tft.setTextColor(CMUTED, CBG);
  tft.setCursor(RX_S + 2, BAT_Y_ + 26);
  if (bV > 0) {
    tft.print(bV, 1);
    tft.print("V  ");
  } else
    tft.print("--V");
}

void showPanel(AlertT t, const String &txt, bool fromPhone = false,
               String aiName = "System") {
  aiPanelOn = true;
  aiPanelMs = millis();
  uint16_t hc = aColor(t);
  tft.fillRect(RX_S, HDR_H, RW, 240 - HDR_H - 1, CBG);
  tft.fillRect(RX_S, HDR_H, RW, 14, hc);
  tft.setTextColor(CBG, hc);
  tft.setTextSize(1);
  tft.setCursor(RX_S + 3, HDR_H + 3);
  tft.print("! ");
  tft.print(aLbl(t));
  tft.setTextColor(fromPhone ? CPURPLE : CMUTED, CBG);
  tft.setCursor(RX_S + 3, HDR_H + 16);
  tft.print("[ " + aiName + " ]");
  tft.setTextColor(CWHITE, CBG);
  tft.setTextSize(1);
  int y = HDR_H + 27;
  const int mc = 18;
  int i = 0;
  while (i < (int)txt.length() && y < 226) {
    if (txt[i] == '\n') {
      i++;
      y += 9;
      continue;
    }
    int len = min((int)txt.length() - i, mc);
    for (int k = 0; k < len; k++)
      if (txt[i + k] == '\n') {
        len = k;
        break;
      }
    if (len == 0) {
      i++;
      y += 9;
      continue;
    }
    if (i + len < (int)txt.length() && txt[i + len] != ' ' &&
        txt[i + len] != '\n') {
      int c = len;
      while (c > 4 && txt[i + c - 1] != ' ')
        c--;
      if (c > 3)
        len = c;
    }
    tft.setCursor(RX_S + 3, y);
    tft.print(txt.substring(i, i + len));
    y += 9;
    i += len;
    if (i < (int)txt.length() && txt[i] == ' ')
      i++;
  }
  tft.setTextColor(CDIM, CBG);
  tft.setCursor(RX_S + 3, 229);
  tft.print("[BTN AI = dismiss]");
}

void dismissPanel() {
  if (!aiPanelOn)
    return;
  aiPanelOn = false;
  tft.fillRect(RX_S, HDR_H, RW, 240 - HDR_H - 1, CBG);
  hl(RX_S, COMP_Y, RW, CDIV);
  hl(RX_S, GPS_Y, RW, CDIV);
  hl(RX_S, SPD_Y, RW, CDIV);
  hl(RX_S, BAT_Y_, RW, CDIV);
  tft.setTextColor(CMUTED, CBG);
  tft.setTextSize(1);
  tft.setCursor(RX_S + 3, HDR_H + 2);
  tft.print("DIR    ");
  tft.setCursor(RX_S + 3, COMP_Y + 2);
  tft.print("HEADING");
  drawCompassRing();
  cache.dir = '?';
  cache.lat = -999;
  cache.spdKt = -1;
  cache.gpsFix = -1;
  cache.batV = -1;
  cache.batPct = -1;
  cache.hm = !homeMode;
}

void drawFrame() {
  tft.fillScreen(CBG);
  drawHdrStatic();
  vl(DIV_X, HDR_H, 240 - HDR_H, CDIV);
  vl(DIV_X + 1, HDR_H, 240 - HDR_H, CNAVY);
  for (int i = 0; i < N_CARDS; i++)
    drawCardBg(i);
  tft.setTextColor(CMUTED, CBG);
  tft.setTextSize(1);
  tft.setCursor(RX_S + 3, HDR_H + 2);
  tft.print("DIR");
  hl(RX_S, COMP_Y, RW, CDIV);
  tft.setCursor(RX_S + 3, COMP_Y + 2);
  tft.print("HEADING");
  hl(RX_S, GPS_Y, RW, CDIV);
  hl(RX_S, SPD_Y, RW, CDIV);
  hl(RX_S, BAT_Y_, RW, CDIV);
  hl(0, 239, 320, CDIV, 1);
  drawCompassRing();
}

static int current_qr_y_offset = 0;
void my_qrcode_display(esp_qrcode_handle_t qrcode) {
  int size = esp_qrcode_get_size(qrcode);
  int module_size = 3;
  if (size > 33)
    module_size = 2; // scale down if QR is too big

  int startX = RX_S + (RW - size * module_size) / 2;
  int startY = current_qr_y_offset;

  tft.fillRect(startX - 4, startY - 4, size * module_size + 8,
               size * module_size + 8, CWHITE);
  for (int y = 0; y < size; y++) {
    for (int x = 0; x < size; x++) {
      if (esp_qrcode_get_module(qrcode, x, y)) {
        tft.fillRect(startX + x * module_size, startY + y * module_size,
                     module_size, module_size, 0x0000);
      }
    }
  }
}

void showQRCodes() {
  aiPanelOn = true;
  aiPanelMs = millis();
  tft.fillRect(RX_S, HDR_H, RW, 240 - HDR_H - 1, CBG);

  tft.setTextColor(CCYAN, CBG);
  tft.setTextSize(1);

  tft.setCursor(RX_S + 20, HDR_H + 5);
  tft.print("WiFi: MEI-Controller");

  esp_qrcode_config_t cfg = ESP_QRCODE_CONFIG_DEFAULT();
  cfg.display_func = my_qrcode_display;
  cfg.max_qrcode_version = 4;
  cfg.qrcode_ecc_level = ESP_QRCODE_ECC_LOW;

  current_qr_y_offset = HDR_H + 18;
  esp_qrcode_generate(&cfg, "WIFI:S:MEI-Controller;T:WPA;P:mei12345;;");

  tft.setCursor(RX_S + 28, HDR_H + 112);
  tft.print("Web: 192.168.4.1");

  current_qr_y_offset = HDR_H + 125;
  esp_qrcode_generate(&cfg, "http://192.168.4.1");
}

void bootSplash() {
  tft.fillScreen(CBG);
  tft.drawRect(4, 4, 312, 232, CDIV);
  tft.setTextColor(CCYAN);
  tft.setTextSize(3);
  tft.setCursor(110, 36);
  tft.print("MEI");
  tft.setTextColor(CMUTED);
  tft.setTextSize(1);
  tft.setCursor(46, 75);
  tft.print("AI Water Monitor  v18.0  ESP32-S3");
  tft.setTextColor(CGREEN);
  tft.setCursor(34, 92);
  tft.print("WiFi AP : MEI-Controller");
  tft.setTextColor(CCYAN);
  tft.setCursor(34, 103);
  tft.print("Pass    : mei12345");
  tft.setTextColor(CAMBER);
  tft.setCursor(34, 114);
  tft.print("Browser : 192.168.4.1");
  tft.setTextColor(CPURPLE);
  tft.setCursor(34, 125);
  tft.print("AI      : BTN GPIO21 (x1/x2)");
  tft.setTextColor(CMUTED);
  tft.setCursor(34, 136);
  tft.print("HOME    : JOY_SW (GPIO6)");
  tft.drawRect(38, 150, 244, 12, CDIV);
  for (int i = 0; i <= 242; i += 3) {
    tft.fillRect(39, 151, i, 10, tft.color565(0, (uint8_t)(i * 0.84f), 255));
    delay(4);
  }
  tft.setTextColor(CGREEN);
  tft.setCursor(136, 168);
  tft.print("READY");
  delay(400);
}

void i2sInit() {
  i2s_config_t cfg = {.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
                      .sample_rate = SAMPLE_RATE,
                      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
                      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
                      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
                      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
                      .dma_buf_count = 4,
                      .dma_buf_len = 256,
                      .use_apll = false,
                      .tx_desc_auto_clear = false,
                      .fixed_mclk = 0};
  i2s_pin_config_t pins = {.bck_io_num = I2S_SCK_PIN,
                           .ws_io_num = I2S_WS_PIN,
                           .data_out_num = I2S_PIN_NO_CHANGE,
                           .data_in_num = I2S_SD_PIN};
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
}

void thucThiLenhLai(char cmd, String tenLenh, String aiName = "System",
                    int customSpd = -1) {
  int spd = (cmd == 'S' || cmd == 'H') ? 0 : 200;
  if (customSpd >= 0)
    spd = customSpd;
  homeMode = (cmd == 'H');
  txMsg.lenh = cmd;
  txMsg.tocDo = spd;
  esp_now_send(BOAT_MAC, (uint8_t *)&txMsg, sizeof(txMsg));
  showPanel(AN, "AI RA LENH:\n>> " + tenLenh, false, aiName);
  sendTelegramAlert("🚢 Thuyền nhận lệnh: " + tenLenh +
                    " (Tốc độ: " + String(spd) + ")");

  cache.dir = '?';
  char d = 'G';
  if (cmd == 'T')
    d = 'W';
  else if (cmd == 'A')
    d = 'A';
  else if (cmd == 'P')
    d = 'D';
  else if (cmd == 'L')
    d = 'L';
  curDir = d;
  curSpd = spd;
  updDir(d, spd, homeMode);
}

void traLoiThongSo(String loai, String aiName = "System") {
  String ans = "AI BAO CAO:\n";
  if (loai == "nhiet_do")
    ans += "Nhiet do:\n" + String(rx.temp, 1) +
           " \xF8"
           "C";
  else if (loai == "ph")
    ans += "Do pH:\n" + String(rx.ph, 2);
  else if (loai == "do_duc")
    ans += "Do duc:\n" + String(rx.ntu) + " NTU";
  else if (loai == "pin")
    ans += "Pin:\n" + String(rx.batVolt, 1) + "V (" +
           String((int)(constrain((rx.batVolt - BAT_MIN) / (BAT_MAX - BAT_MIN),
                                  0.f, 1.f) *
                        100)) +
           "%)";
  else if (loai == "chat_luong") {
    ans += "pH:" + String(rx.ph, 1) + " TDS:" + String(rx.tds) +
           "\nNhiet:" + String(rx.temp, 1) + "C\n";
    ans += (chkAlert() != AN) ? "CANH BAO: XAU" : "AN TOAN";
  } else if (loai == "bao_cao") {
    ans += "pH:" + String(rx.ph, 1) + " TDS:" + String(rx.tds) +
           "\nNhiet:" + String(rx.temp, 1) + "C Duc:" + String(rx.ntu) + "\n";
    AlertT alt = chkAlert();
    ans += "\nTINH HINH NUOC:\n";
    if (alt == AN) {
      ans += "- Nuoc tot, an toan.\n- Khong can gi them.";
    } else if (alt == APL) {
      ans += "- pH qua THAP!\n- Can tang pH ngay.";
    } else if (alt == APH) {
      ans += "- pH qua CAO!\n- Can giam pH ngay.";
    } else if (alt == ATDS) {
      ans += "- TDS NGUY HIEM.\n- Can thay/loc nuoc.";
    } else if (alt == ATEMP) {
      ans += "- NHIET DO CAO.\n- Kiem tra khu vuc.";
    } else if (alt == ANTU) {
      ans += "- DO DUC CAO.\n- Can loc nuoc sach.";
    } else if (alt == ABAT) {
      ans += "- PIN YEU!\n- Vui long cho ve bo!";
    }
  }
  showPanel(AN, ans, false, aiName);
}

void phanTichIntent(String intent, String aiName = "System") {
  if (intent == "lenh_tien")
    thucThiLenhLai('T', "DI THANG", aiName);
  else if (intent == "lenh_lui")
    thucThiLenhLai('L', "DI LUI", aiName);
  else if (intent == "lenh_trai")
    thucThiLenhLai('A', "RE TRAI", aiName);
  else if (intent == "lenh_phai")
    thucThiLenhLai('P', "RE PHAI", aiName);
  else if (intent == "lenh_dung")
    thucThiLenhLai('S', "DUNG LAI", aiName);
  else if (intent == "lenh_tro_ve")
    thucThiLenhLai('H', "VE NHA", aiName);
  else if (intent == "gui_vi_tri") {
    String msg =
        "Vi tri:\nLat: " + String(rx.lat, 6) + "\nLon: " + String(rx.lon, 6);
    showPanel(AN, msg, false, aiName);
    if (rx.gpsStatus > 0)
      sendTelegramAlert("📍 Vị trí thuyền: https://maps.google.com/?q=" +
                        String(rx.lat, 6) + "," + String(rx.lon, 6));
    else
      sendTelegramAlert("⚠️ Thuyền chưa có kết nối GPS! Không thể lấy tọa độ.");
  } else if (intent == "tang_toc") {
    int newSpd = curSpd + 50;
    if (newSpd > 255)
      newSpd = 255;
    char c = 'T';
    if (curDir == 'A')
      c = 'A';
    if (curDir == 'D')
      c = 'P';
    if (curDir == 'L')
      c = 'L';
    if (curDir == 'G' || curDir == 'S')
      c = 'T';
    thucThiLenhLai(c, "TANG TOC len " + String(newSpd), aiName, newSpd);
  } else if (intent == "giam_toc") {
    int newSpd = curSpd - 50;
    if (newSpd < 0)
      newSpd = 0;
    char c = 'T';
    if (curDir == 'A')
      c = 'A';
    if (curDir == 'D')
      c = 'P';
    if (curDir == 'L')
      c = 'L';
    if (curDir == 'G' || curDir == 'S')
      c = 'T';
    thucThiLenhLai(c, "GIAM TOC xuong " + String(newSpd), aiName, newSpd);
  } else if (intent == "chao_hoi") {
    showPanel(AN, "Chao thuyen truong!", false, aiName);
    sendTelegramAlert("👋 Chào thuyền trưởng! Hệ thống MEI đang trực tuyến và "
                      "sẵn sàng nhận lệnh.");
  } else if (intent == "hoi_nhiet_do")
    traLoiThongSo("nhiet_do", aiName);
  else if (intent == "hoi_ph")
    traLoiThongSo("ph", aiName);
  else if (intent == "hoi_do_duc")
    traLoiThongSo("do_duc", aiName);
  else if (intent == "hoi_pin")
    traLoiThongSo("pin", aiName);
  else if (intent == "hoi_chat_luong")
    traLoiThongSo("chat_luong", aiName);
  else if (intent == "bao_cao")
    traLoiThongSo("bao_cao", aiName);
  else
    showPanel(AN, "AI KHONG HIEU\n(" + intent + ")", false, aiName);
}

int microphone_audio_signal_get_data(size_t offset, size_t length,
                                     float *out_ptr) {
  numpy::int16_to_float((int16_t *)recBuf + offset, out_ptr, length);
  return 0;
}

void triggerLocalAI() {
  if (!recBuf)
    return;
  aiState = AI_RECORDING;
  updHdr(connected(), (millis() - lastRxLora < 3000), rx.gpsStatus,
         homeMode || (rx.isReturning == 1), rx.batVolt, true);

  showPanel(AN, "Local AI:\nDang nghe 2s...", false, "Local AI");
  size_t bytesRead = 0;
  i2s_read(I2S_PORT, recBuf, LOCAL_REC_BYTES, &bytesRead, portMAX_DELAY);

  aiState = AI_PROCESSING;
  updHdr(connected(), (millis() - lastRxLora < 3000), rx.gpsStatus,
         homeMode || (rx.isReturning == 1), rx.batVolt, true);
  showPanel(AN, "Dang xu ly...", false, "Local AI");

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  signal.get_data = &microphone_audio_signal_get_data;
  ei_impulse_result_t result = {0};
  EI_IMPULSE_ERROR r = run_classifier(&signal, &result, false);

  if (r != EI_IMPULSE_OK) {
    showPanel(AN, "Loi chay AI Local!", false, "Local AI");
    return;
  }

  int best_idx = -1;
  float best_score = 0.0;
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    if (result.classification[ix].value > best_score) {
      best_score = result.classification[ix].value;
      best_idx = ix;
    }
  }

  if (best_idx >= 0 && best_score > 0.70) {
    String label = String(result.classification[best_idx].label);
    Serial.printf("[Local AI] %s (%.2f)\n", label.c_str(), best_score);
    if (label == "tien")
      phanTichIntent("lenh_tien", "Local AI");
    else if (label == "lui")
      phanTichIntent("lenh_lui", "Local AI");
    else if (label == "trai")
      phanTichIntent("lenh_trai", "Local AI");
    else if (label == "phai")
      phanTichIntent("lenh_phai", "Local AI");
    else if (label == "dung")
      phanTichIntent("lenh_dung", "Local AI");
    else if (label == "tro_ve")
      phanTichIntent("lenh_tro_ve", "Local AI");
    else if (label == "nhiet_do")
      phanTichIntent("hoi_nhiet_do", "Local AI");
    else if (label == "ph")
      phanTichIntent("hoi_ph", "Local AI");
    else if (label == "do_duc")
      phanTichIntent("hoi_do_duc", "Local AI");
    else if (label == "pin")
      phanTichIntent("hoi_pin", "Local AI");
    else if (label == "chat_luong")
      phanTichIntent("hoi_chat_luong", "Local AI");
    else
      showPanel(AN, "Nhieu am/Khong hieu", false, "Local AI");
  } else {
    showPanel(AN, "Chua nghe ro.\nThu lai nhe!", false, "Local AI");
  }
  aiState = AI_SHOWING;
  aiPanelMs = millis();
}

// ================================================================
//  HÀM GEMINI AI (ĐÃ FIX API 400 - KHÔNG DẤU TIẾNG VIỆT)
// ================================================================
void triggerGeminiAI() {
  if (isTelegramSending) {
    showPanel(AN, "He thong dang ban\ngui Telegram.\nXin thu lai...", false,
              "System");
    return;
  }
  if (!recBuf)
    return;
  if (!wifiOnline || WiFi.status() != WL_CONNECTED) {
    showPanel(AN, "Loi: Khong co mang!\nKet noi WiFi de dung\nGemini AI.",
              false, "System");
    return;
  }

  aiState = AI_RECORDING;
  updHdr(connected(), (millis() - lastRxLora < 3000), rx.gpsStatus,
         homeMode || (rx.isReturning == 1), rx.batVolt, true);

  showPanel(AN, "Gemini AI:\nDang nghe 1.5s...", false, "Gemini AI");
  size_t bytesRead = 0;
  i2s_read(I2S_PORT, recBuf, GEMINI_REC_BYTES, &bytesRead, portMAX_DELAY);

  aiState = AI_PROCESSING;
  updHdr(connected(), (millis() - lastRxLora < 3000), rx.gpsStatus,
         homeMode || (rx.isReturning == 1), rx.batVolt, true);
  showPanel(AN, "Gemini: Dang\nsuy nghi...", false, "Gemini AI");

  uint8_t wavHeader[44];
  uint32_t dataSize = bytesRead;
  uint32_t fileSize = 36 + dataSize;
  uint32_t sampleRate = 16000;
  uint16_t numChannels = 1;
  uint16_t bitsPerSample = 16;
  uint32_t byteRate = sampleRate * 2;
  uint16_t blockAlign = 2;

  wavHeader[0] = 'R';
  wavHeader[1] = 'I';
  wavHeader[2] = 'F';
  wavHeader[3] = 'F';
  memcpy(wavHeader + 4, &fileSize, 4);
  wavHeader[8] = 'W';
  wavHeader[9] = 'A';
  wavHeader[10] = 'V';
  wavHeader[11] = 'E';
  wavHeader[12] = 'f';
  wavHeader[13] = 'm';
  wavHeader[14] = 't';
  wavHeader[15] = ' ';
  uint32_t fmtChunkSize = 16;
  memcpy(wavHeader + 16, &fmtChunkSize, 4);
  uint16_t audioFormat = 1;
  memcpy(wavHeader + 20, &audioFormat, 2);
  memcpy(wavHeader + 22, &numChannels, 2);
  memcpy(wavHeader + 24, &sampleRate, 4);
  memcpy(wavHeader + 28, &byteRate, 4);
  memcpy(wavHeader + 32, &blockAlign, 2);
  memcpy(wavHeader + 34, &bitsPerSample, 2);
  wavHeader[36] = 'd';
  wavHeader[37] = 'a';
  wavHeader[38] = 't';
  wavHeader[39] = 'a';
  memcpy(wavHeader + 40, &dataSize, 4);

  // ===================== TIẾN HÓA CỨU RAM CẤP TỐC =====================
  // Gán đè WAV Header ngay vào khoảng trống 44 byte chuẩn bị sẵn trước recBuf!
  memcpy(staticWavBuf, wavHeader, 44);
  size_t wavLen = 44 + bytesRead;

  // Tính kích thước Base64
  size_t b64Len = 0;
  mbedtls_base64_encode(NULL, 0, &b64Len, staticWavBuf, wavLen);

  const char *prefix =
      "{\"contents\":[{\"parts\":[{\"text\":\"Chi tra ve 1 tu: lenh_tien, "
      "lenh_lui, lenh_trai, lenh_phai, lenh_dung, lenh_tro_ve, gui_vi_tri, "
      "tang_toc, giam_toc, chao_hoi, hoi_nhiet_do, "
      "hoi_ph, hoi_do_duc, hoi_pin, hoi_chat_luong, bao_cao. Khac: "
      "khong_hieu.\"},{\"inlineData\":{\"mimeType\":\"audio/"
      "wav\",\"data\":\"";
  const char *suffix = "\"}}]}]}";
  size_t prefixLen = strlen(prefix);
  size_t suffixLen = strlen(suffix);
  size_t payloadLen = prefixLen + b64Len + suffixLen;

  if (payloadLen + 1 > sizeof(staticPayloadBuf)) {
    showPanel(AN, "Loi! JSON Payload \nvuot qua 46KB!", false, "System");
    return;
  }

  // Ghép JSON thẳng vào bộ đệm tĩnh (Tuyệt đối không phân mảnh heap)
  memcpy(staticPayloadBuf, prefix, prefixLen);
  mbedtls_base64_encode((unsigned char *)(staticPayloadBuf + prefixLen),
                        b64Len + 1, &b64Len, staticWavBuf, wavLen);
  memcpy(staticPayloadBuf + prefixLen + b64Len, suffix, suffixLen + 1);
  payloadLen = prefixLen + b64Len + suffixLen;

  WiFiClientSecure c;
  c.setInsecure();
  HTTPClient http;
  String url = "https://generativelanguage.googleapis.com/v1beta/models/"
               "gemini-2.5-flash:generateContent?key=" +
               String(GEMINI_API_KEY);
  http.begin(c, url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(25000);

  int code = http.POST((uint8_t *)staticPayloadBuf, payloadLen);

  if (code == 200) {
    DynamicJsonDocument doc(2048);
    deserializeJson(doc, http.getString());
    String resultText =
        doc["candidates"][0]["content"]["parts"][0]["text"].as<String>();
    resultText.trim();
    resultText.toLowerCase();
    Serial.println("[Gemini] " + resultText);

    if (resultText.indexOf("tien") >= 0)
      phanTichIntent("lenh_tien", "Gemini AI");
    else if (resultText.indexOf("lui") >= 0)
      phanTichIntent("lenh_lui", "Gemini AI");
    else if (resultText.indexOf("trai") >= 0)
      phanTichIntent("lenh_trai", "Gemini AI");
    else if (resultText.indexOf("phai") >= 0)
      phanTichIntent("lenh_phai", "Gemini AI");
    else if (resultText.indexOf("dung") >= 0)
      phanTichIntent("lenh_dung", "Gemini AI");
    else if (resultText.indexOf("tro_ve") >= 0)
      phanTichIntent("lenh_tro_ve", "Gemini AI");
    else if (resultText.indexOf("nhiet_do") >= 0)
      phanTichIntent("hoi_nhiet_do", "Gemini AI");
    else if (resultText.indexOf("ph") >= 0)
      phanTichIntent("hoi_ph", "Gemini AI");
    else if (resultText.indexOf("do_duc") >= 0)
      phanTichIntent("hoi_do_duc", "Gemini AI");
    else if (resultText.indexOf("pin") >= 0)
      phanTichIntent("hoi_pin", "Gemini AI");
    else if (resultText.indexOf("chat_luong") >= 0)
      phanTichIntent("hoi_chat_luong", "Gemini AI");
    else if (resultText.indexOf("bao_cao") >= 0 ||
             resultText.indexOf("b-io") >= 0 || resultText.indexOf("o_c") > 0)
      phanTichIntent("bao_cao", "Gemini AI");
    else if (resultText.indexOf("gui_vi") >= 0)
      phanTichIntent("gui_vi_tri", "Gemini AI");
    else if (resultText.indexOf("tang_") >= 0)
      phanTichIntent("tang_toc", "Gemini AI");
    else if (resultText.indexOf("giam_") >= 0)
      phanTichIntent("giam_toc", "Gemini AI");
    else if (resultText.indexOf("chao") >= 0)
      phanTichIntent("chao_hoi", "Gemini AI");
    else
      showPanel(AN, "Gemini khong hieu:\n" + resultText, false, "Gemini AI");
  } else {
    Serial.printf("[Gemini] Loi HTTP: %d\n", code);
    showPanel(AN, "Loi ket noi Gemini!\nMa loi: " + String(code), false,
              "Gemini AI");
  }
  http.end();

  aiState = AI_SHOWING;
  aiPanelMs = millis();
}

void setup() {
  Serial.begin(115200);

  pinMode(JOY_X, INPUT);
  pinMode(JOY_Y, INPUT);
  pinMode(JOY_SW, INPUT_PULLUP);
  pinMode(BTN_AI, INPUT_PULLUP);
  sharedSPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  tft.init(240, 320);
  tft.setRotation(3);
  tft.fillScreen(0);
  initColors();
  bootSplash();
  i2sInit();

  // ================================================================
  //  CHIẾN THUẬT WIFI MỚI: QUÉT TRƯỚC, KẾT NỐI SAU
  // ================================================================

  // KHỞI ĐỘNG DRIVER WIFI TRƯỚC TIÊN (NẾU KHÔNG GỌI DISCONNECT SẼ BỊ CRASH
  // BOOTLOOP!)
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false); // TẮT NGỦ WIFI ĐỂ JOYSTICK CHẠY MƯỢT MÀ 0MS DELAY

  // XÓA SẠCH BỘ NHỚ NVS CỦA WIFI ĐỂ KHÔNG BỊ LẠM DỤNG AUTO-RECONNECT
  WiFi.persistent(false);
  WiFi.disconnect(); // Ngắt an toàn
  delay(10);

  // Không dùng malloc cho recBuf nữa vì đã khai báo mảng tĩnh cực an toàn ở
  // trên.

  Serial.println("[WiFi] Dang quet kiem tra Hotspot...");
  int nScan =
      WiFi.scanNetworks(false, false, false, 300); // Quét nhanh 300ms/kênh
  bool hotspotFound = false;
  for (int i = 0; i < nScan; i++) {
    if (WiFi.SSID(i) == WIFI_SSID) {
      hotspotFound = true;
      Serial.printf("[WiFi] Tim thay '%s' o kenh %d\n", WIFI_SSID,
                    WiFi.channel(i));
      break;
    }
  }
  WiFi.scanDelete(); // Xóa kết quả scan giải phóng RAM

  if (hotspotFound) {
    // BƯỚC 2A: CÓ HOTSPOT -> Kết nối bình thường (AP_STA)
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("[WiFi] Connecting");
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
      delay(500);
      Serial.print(".");
    }
    wifiOnline = (WiFi.status() == WL_CONNECTED);
    if (wifiOnline) {
      Serial.println("\n[WiFi] DA KET NOI THANH CONG!");
      // BẮT BUỘC ÉP KHỚP KÊNH GIỮA AP VÀ STA NẾU KHÔNG ESP32 SẼ CHẾT LÂM SÀNG
      // VÌ NHẢY KÊNH LIÊN TỤC
      WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                        IPAddress(255, 255, 255, 0));
      WiFi.softAP(AP_SSID, AP_PASS,
                  WiFi.channel()); // TRUYỀN VÀO WiFi.channel() ĐỂ ĐỒNG BỘ 100%
    } else {
      // Tìm thấy nhưng kết nối thất bại -> Quay về Kênh 1
      WiFi.setAutoReconnect(false); // KHÔNG TỰ QUÉT LẠI ĐỂ TRÁNH LAG RADIO
      WiFi.disconnect(); // Ngắt dò tìm mạng nhưng KHÔNG TẮT module STA
      // KHÔNG ĐƯỢC CHUYỂN SANG WIFI_AP VÌ SẼ LÀM MẤT MAC ADDRESS CỦA STA
      // (khiến mạch bị điếc)
      WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                        IPAddress(255, 255, 255, 0));
      WiFi.softAP(AP_SSID, AP_PASS, 1);
      Serial.println("\n[WiFi] Ket noi that bai. Chuyen ve kenh 1.");
    }
  } else {
    // BƯỚC 2B: KHÔNG CÓ HOTSPOT -> KHÔNG GỌI WiFi.begin()!
    // Đây là bí quyết: không gọi WiFi.begin() = radio KHÔNG BAO GIỜ nhảy
    // kênh!
    WiFi.setAutoReconnect(false); // CHẮC CHẮN KHÔNG QUÉT LẠI
    WiFi.disconnect();            // Ngắt dò tìm thay vì tắt toàn bộ
    // KHÔNG ĐƯỢC CHUYỂN SANG WIFI_AP VÌ SẼ LÀM MẤT MAC ADDRESS CỦA STA
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                      IPAddress(255, 255, 255, 0));
    WiFi.softAP(AP_SSID, AP_PASS, 1); // Ép cứng kênh 1
    Serial.println("[WiFi] Khong co Hotspot. Kenh 1. ESP-NOW on dinh!");
  }

  Serial.printf("[AP] %s 192.168.4.1\n", AP_SSID);
  startWeb();
  LoRa.setSPI(sharedSPI);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (LoRa.begin(LORA_FREQ)) {
    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setTxPower(17);
    loraOk = true;
    Serial.println("[LoRa] OK");
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] ESP-NOW");
    return;
  }
  esp_now_register_recv_cb((esp_now_recv_cb_t)onRecv);
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, BOAT_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  // KÍCH HOẠT TIẾN TRÌNH TELEGRAM CÓ KIỂM SOÁT ĐỂ CHỐNG TRÀN RAM KHI DÙNG AI
  telegramQueue = xQueueCreate(10, 512);
  if (telegramQueue != NULL) {
    xTaskCreatePinnedToCore(telegramTask, "TeleTask", 8192, NULL, 1, NULL, 0);
  }

  drawFrame();
  for (int i = 0; i < N_CARDS; i++)
    updCard(i, 0);
  updDir('G', 0, false);
  updCompassNeedle(-1.0f, 'G');
  updGPS(0, 0, 0);
  updSpd(0, 0);
  updBat(0);
  updHdr(false, false, 0, false, 0, true);

  long sx = 0, sy = 0;
  for (int k = 0; k < 20; k++) {
    sx += analogRead(JOY_X);
    sy += analogRead(JOY_Y);
    delay(5);
  }
  joyXc = (int)(sx / 20);
  joyYc = (int)(sy / 20);
  if (joyXc < 100 || joyXc > 3990)
    joyXc = 2048;
  if (joyYc < 100 || joyYc > 3990)
    joyYc = 2048;
}

void loop() {
  unsigned long now = millis();

  // Chạy quét WiFi an toàn ở Core 1 để không bị Crash LwIP Watchdog
  if (triggerWifiScan) {
    triggerWifiScan = false;
    WiFi.scanNetworks(true);
  }

  // CHỐNG SẬP MẠCH KHI KẾT NỐI WIFI SAI MẬT KHẨU
  // Tách block HTTP_POST và giao phần xử lý kết nối cho Main Loop Non-blocking
  if (triggerWifiConnect) {
    triggerWifiConnect = false;
    WiFi.disconnect();
    WiFi.begin(wifiSSID, wifiPASS);
    wfState = WF_CONNECTING;
    wfAttemptMs = millis();
    Serial.printf("[WiFi] Thu ket noi '%s' ngam (khong chan)... \n", wifiSSID);
  }

  if (wfState == WF_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      wfState = WF_CONNECTED;
      wifiOnline = true;
      WiFi.softAP(AP_SSID, AP_PASS, WiFi.channel()); // Đồng bộ kênh AP an toàn
      Serial.printf("[WiFi] KET NOI THANH CONG! IP: %s\n",
                    WiFi.localIP().toString().c_str());
    } else if (millis() - wfAttemptMs > 12000) {
      // Nếu 12 giây mà vẫn chưa vào được -> Sai pass hoặc chết mạng
      wfState = WF_FAILED;
      wifiOnline = false;
      WiFi.setAutoReconnect(false);
      WiFi.disconnect();
      WiFi.softAP(AP_SSID, AP_PASS, 1); // Lui về kênh 1 bảo tồn sóng ESPNOW
      Serial.println("[WiFi] LOI! SAI MAT KHAU HOAC MAT MANG! HUY VA QUAY VE "
                     "KENH 1 AN TOAN.");
    }
  }

  static AlertT prevAlt = AN;

  static bool lastConnState = true;
  bool currentConnState = connected();
  if (lastConnState == true && currentConnState == false) {
    sendTelegramAlert(
        "CANH BAO: Thuyen vuot ngoai tam ket noi hoac dac biet mat song!");
  } else if (lastConnState == false && currentConnState == true) {
    sendTelegramAlert("Thong bao: DA KET NOI LAI voi thuyen an toan.");
  }
  lastConnState = currentConnState;

  if (loraOk) {
    int sz = LoRa.parsePacket();
    if (sz >= (int)sizeof(rx)) {
      uint8_t buf[sizeof(rx)];
      for (int i = 0; i < (int)sizeof(rx) && LoRa.available(); i++)
        buf[i] = LoRa.read();
      memcpy(&rx, buf, sizeof(rx));
      newData = true;
      lastRxLora = now;
    }
  }

  if (newData) {
    newData = false;
    bool c = connected();
    bool h = homeMode || (rx.isReturning == 1);
    if (fabsf(rx.ph - cache.ph) > 0.02f) {
      cache.ph = rx.ph;
      updCard(0, rx.ph);
    }
    if (rx.tds != cache.tds) {
      cache.tds = rx.tds;
      updCard(1, (float)rx.tds);
    }
    if (fabsf(rx.temp - cache.temp) > 0.1f) {
      cache.temp = rx.temp;
      updCard(2, rx.temp);
    }
    if (rx.ntu != cache.ntu) {
      cache.ntu = rx.ntu;
      updCard(3, (float)rx.ntu);
    }
    updHdr(c, (now - lastRxLora < 3000), rx.gpsStatus, h, rx.batVolt, true);
    if (!aiPanelOn) {
      updDir(curDir, curSpd, h);
      updCompassNeedle(rx.heading, curDir);
      updGPS(rx.lat, rx.lon, rx.gpsStatus);
      updSpd(rx.gpsSpeedKt, rx.gpsCourse);
      updBat(rx.batVolt);
    }
    AlertT alt = chkAlert();
    curAlert = alt;
    if (alt != AN && alt != prevAlt && (now - lastAlertMs > ALERT_CD_MS)) {
      lastAlertMs = now;
      prevAlt = alt;
      showPanel(alt, ruleMsg(alt));
      sendTelegramAlert("CANH BAO HE THONG:\n" + ruleMsg(alt));
    }
    if (alt == AN)
      prevAlt = AN;
  }

  if (hasNewAiResp) {
    hasNewAiResp = false;
    showPanel(curAlert, pendingAiResp, true);
    aiPanelMs = now;
  }

  static unsigned long lHdr = 0;
  if (now - lHdr > 1500) {
    lHdr = now;
    updHdr(connected(), (now - lastRxLora < 3000), rx.gpsStatus,
           homeMode || (rx.isReturning == 1), rx.batVolt, true);
  }

  static bool lAI = HIGH;
  static unsigned long aiPressTime = 0;
  static unsigned long aiReleaseTime = 0;
  static int clickCount = 0;
  static bool isPressingAI = false;
  const unsigned long doubleClickTimeout = 300;

  bool currentBtnAI = digitalRead(BTN_AI);

  if (currentBtnAI == LOW && !isPressingAI) {
    isPressingAI = true;
    aiPressTime = now;
  } else if (currentBtnAI == HIGH && isPressingAI) {
    isPressingAI = false;
    if (now - aiPressTime > 50) {
      clickCount++;
      aiReleaseTime = now;
    }
  }

  if (clickCount > 0 && (now - aiReleaseTime > doubleClickTimeout)) {
    if (aiPanelOn) {
      dismissPanel();
      aiState = AI_IDLE;
      lastAlertMs = now;
    } else {
      if (clickCount == 1) {
        Serial.println("[BTN_AI] 1 Click -> Local AI");
        triggerLocalAI();
      } else if (clickCount >= 2) {
        Serial.println("[BTN_AI] 2 Clicks -> Gemini AI");
        triggerGeminiAI();
      }
    }
    clickCount = 0;
  }

  static bool lSW = HIGH;
  static unsigned long swDownMs = 0;
  static unsigned long swUpMs = 0;
  static int swClickCnt = 0;

  bool sw = digitalRead(JOY_SW);
  if (sw == LOW && lSW == HIGH) {
    swDownMs = now;
  }
  if (sw == HIGH && lSW == LOW && (now - swDownMs) > 50) {
    swClickCnt++;
    swUpMs = now;
  }
  lSW = sw;

  if (swClickCnt > 0 && (now - swUpMs > 300)) {
    if (swClickCnt == 1) {
      if (aiPanelOn) {
        dismissPanel();
      } else {
        homeMode = !homeMode;
        txMsg.lenh = homeMode ? 'H' : 'S';
        txMsg.tocDo = 0;
        esp_now_send(BOAT_MAC, (uint8_t *)&txMsg, sizeof(txMsg));
        cache.dir = '?';
        updDir(curDir, curSpd, homeMode);
      }
    } else {
      if (!aiPanelOn) {
        showQRCodes();
      } else {
        dismissPanel();
      }
    }
    swClickCnt = 0;
  }

  // ================================================================
  //  JOYSTICK DI CHUYỂN: ĐÃ ĐẢO CHIỀU TRỤC Y & FIX LỖI LA BÀN
  // ================================================================
  static unsigned long lJ = 0;
  static char lC = 'S';
  static int lSpd = 0;

  if (!homeMode && aiState == AI_IDLE &&
      now - lJ > 40) { // Giảm delay từ 80ms xuống 40ms (Tốc độ quét tay lái
                       // 25 FPS siêu mượt)
    lJ = now;
    int x = analogRead(JOY_X);
    int y = analogRead(JOY_Y);

    int dX = x - joyXc;
    int dY = y - joyYc;

    const int D = 800;
    char cmd = 'S';
    int spd = 0;
    char dir = 'G';

    if (abs(dY) > abs(dX) && abs(dY) > D) {
      if (dY < 0) {
        cmd = 'T';
        dir = 'W';
        float percent = (float)(abs(dY) - D) / max(1, (joyYc - D));
        spd = 160 + (int)(percent * (255 - 160));
      } else {
        cmd = 'L';
        dir = 'L';
        float percent = (float)(abs(dY) - D) / max(1, (4095 - joyYc - D));
        spd = 160 + (int)(percent * (255 - 160));
      }
    } else if (abs(dX) >= abs(dY) && abs(dX) > D) {
      if (dX < 0) {
        cmd = 'A';
        dir = 'A';
        float percent = (float)(abs(dX) - D) / max(1, (joyXc - D));
        spd = 160 + (int)(percent * (255 - 160));
      } else {
        cmd = 'P';
        dir = 'D';
        float percent = (float)(abs(dX) - D) / max(1, (4095 - joyXc - D));
        spd = 160 + (int)(percent * (255 - 160));
      }
    }

    spd = constrain(spd, 0, 255);

    txMsg.lenh = cmd;
    txMsg.tocDo = spd;
    esp_now_send(BOAT_MAC, (uint8_t *)&txMsg, sizeof(txMsg));

    if (cmd != lC || abs(spd - lSpd) > 10) {
      lC = cmd;
      lSpd = spd;
      curDir = dir;
      curSpd = spd;

      //  ĐÃ SỬA LẠI ĐÚNG HÀM LA BÀN CỦA BẢN MỚI
      updCompassNeedle(rx.heading, dir);

      cache.dir = '?';
      updDir(dir, spd, false);
    }
  }
}