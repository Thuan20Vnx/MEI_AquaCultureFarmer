#include <Adafruit_INA219.h>
#include <DallasTemperature.h>
#include <LoRa.h>
#include <MPU9250_WE.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>

// ── Chân motor ────────────────────────────────────────────────
#define IN1 15
#define IN2 23
#define IN3 22
#define IN4 21

// ── Chân cảm biến ─────────────────────────────────────────────
#define GPS_RX_PIN 6
#define GPS_TX_PIN 7
#define PH_PIN 0
#define TDS_PIN 1
#define TURB_PIN 2
#define TEMP_PIN 19

// ── I2C ───────────────────────────────────────────────────────
#define I2C_SDA 4
#define I2C_SCL 5

// ── Chân LoRa (Theo đúng bạn cắm) ─────────────────────────────
#define LORA_SCK 18
#define LORA_MISO 20
#define LORA_MOSI 11
#define LORA_CS 13
#define LORA_DIO0 12
#define LORA_RST 3
#define LORA_FREQ 433E6

bool loraReady = false;

uint8_t diaChiBo[] = {0x3C, 0x0F, 0x02,
                      0xD6, 0x3B, 0x30}; // Địa chỉ MAC của Tay Cầm

// ── Struct dữ liệu (Đồng bộ 100% với mạch Tay Cầm S3) ─────────
typedef struct {
  char lenh;
  int tocDo;
} RemoteCommand;

typedef struct {
  float ph;
  int tds;
  float temp;
  int ntu;
  float heading, pitch, roll;
  int gpsStatus;
  float lat, lon;
  float gpsSpeedKt, gpsCourse;
  float batVolt;
  int isReturning;
  uint32_t boatPing;
} SensorData;

SensorData duLieuGui;
RemoteCommand lenhMoi;
esp_now_peer_info_t thongTinKetNoi;

// ── Cờ báo lệnh ───────────────────────────────────────────────
volatile char pendingLenh = 'S';
volatile int pendingTocDo = 0;

void thucThiLenh(char lenh, int tocDo); // Khai báo trước hàm Motor

void khiNhanDuocLenh(const esp_now_recv_info *info, const uint8_t *data,
                     int len) {
  if (len < (int)sizeof(RemoteCommand))
    return;
  memcpy(&lenhMoi, data, sizeof(RemoteCommand));

  // SỬA LỖI DELAY (TRỄ LÁI): CHẠY THẲNG HÀM MOTOR NGAY TẠI CA CA CỦA CORTEX-M4!
  // Đoạn code này được kích hoạt bởi ngắt (Interrupt) của nhân WiFi (Core 0),
  // Nó sẽ chạy tức thời bỏ qua bất kỳ thứ gì đang cản đường ở vòng Loop (Dù là
  // LoRa hay Nhiệt độ)!
  thucThiLenh(lenhMoi.lenh, lenhMoi.tocDo);
}

// ── Khởi tạo thiết bị ─────────────────────────────────────────
OneWire oneWire(TEMP_PIN);
DallasTemperature sensorsTemp(&oneWire);
MPU9250_WE mpu = MPU9250_WE(0x68);
Adafruit_INA219 ina219;
TinyGPSPlus gps;

float ph = 0, temp_ = 0, batVolt = 0;
float heading = 0, pitch = 0, roll = 0;
float lat = 0, lon = 0, gpsSpeedKt = 0, gpsCourse = 0;
int tds = 0, ntu = 0, gpsStatus = 0;
int isReturning = 0;
uint32_t boatPing = 0;
double homeLat = 0, homeLon = 0;
bool homeSet = false;

// Biến lưu trữ cho bộ lọc Pin chống "Parkinson"
static float smoothedBatVolt = -1.0f;

// ── Điều khiển Motor ──────────────────────────────────────────
// (ĐÃ ĐẢO CHIỀU: Thực tế chân quạt lắp ngược nên đổi Tiến <-> Lùi)
void diThang(int s) {
  analogWrite(IN1, 0);
  analogWrite(IN2, s);
  analogWrite(IN3, 0);
  analogWrite(IN4, s);
}
void diLui(int s) {
  analogWrite(IN1, s);
  analogWrite(IN2, 0);
  analogWrite(IN3, s);
  analogWrite(IN4, 0);
}
void reTrai(int s) {
  analogWrite(IN1, 0);
  analogWrite(IN2, s);
  analogWrite(IN3, s);
  analogWrite(IN4, 0);
}
void rePhai(int s) {
  analogWrite(IN1, s);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, s);
}
void dungLai() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}

void thucThiLenh(char lenh, int tocDo) {
  if (isReturning == 1) {
    if (lenh != 'S')
      isReturning = 0; // Hủy tự động quay về nếu tay cầm có tác động
    else
      return;
  }
  switch (lenh) {
  case 'T':
    diThang(tocDo);
    break;
  case 'L':
    diLui(tocDo);
    break;
  case 'A':
    reTrai(tocDo);
    break;
  case 'P':
    rePhai(tocDo);
    break;
  case 'H':
    isReturning = 1;
    dungLai();
    break;
  case 'K':
    break;
  default:
    dungLai();
    isReturning = 0;
    break;
  }
}

// ── Bộ lọc Số (Median + Mean Filter) Khử nhiễu Cảm Biến ─────────
int getStableADC(int pin) {
  int buf[15];
  for (int i = 0; i < 15; i++) {
    buf[i] = analogRead(pin);
  }
  for (int i = 0; i < 14; i++) {
    for (int j = i + 1; j < 15; j++) {
      if (buf[i] > buf[j]) {
        int t = buf[i];
        buf[i] = buf[j];
        buf[j] = t;
      }
    }
  }
  int sum = 0;
  for (int i = 5; i < 10; i++)
    sum += buf[i]; // Lấy 5 giá trị ở giữa để tính trung bình
  return sum / 5;
}

// ── Đọc toàn bộ cảm biến ──────────────────────────────────────
void docCamBien() {
  // 1. Đọc Pin có bộ lọc EMA siêu mượt
  float currentBat = ina219.getBusVoltage_V();
  if (smoothedBatVolt < 0) {
    smoothedBatVolt = currentBat;
  } else {
    smoothedBatVolt = (0.05f * currentBat) + (0.95f * smoothedBatVolt);
  }
  batVolt = smoothedBatVolt;

  // 2. Đọc các cảm biến nước (ĐÃ ÁP DỤNG LỌC NHIỄU VÀ CALIB)
  int adcPH = getStableADC(PH_PIN);
  float vPH = adcPH * (3.3f / 4095.0f);
  // Đảo chiều hệ số góc pH (Điện áp tăng -> pH tăng) do module đo bị ngược
  ph = 7.00f + 5.70f * (vPH - 1.65f);
  if (ph < 0)
    ph = 0;
  if (ph > 14)
    ph = 14;

  int adcTDS = getStableADC(TDS_PIN);
  float vTDS = adcTDS * (3.3f / 4095.0f);
  // Bù trừ sai số nhiệt độ để TDS không bị nhảy số (Compensation Coefficient)
  float tempK = temp_ > 0 ? temp_ : 25.0f;
  float compK = 1.0f + 0.02f * (tempK - 25.0f);
  if (compK < 0.1f)
    compK = 0.1f;
  float compVoltage = vTDS / compK;
  int rawTds =
      (int)((133.42f * compVoltage * compVoltage * compVoltage -
             255.86f * compVoltage * compVoltage + 857.39f * compVoltage) *
            0.5f);
  // Chống nhiễu đỉnh (Spike filter) cho TDS
  static int lastTds = 0;
  tds = (rawTds * 0.4f) + (lastTds * 0.6f); // Tăng độ phản hồi nhạy của TDS
  lastTds = tds;

  int adcNTU = getStableADC(TURB_PIN);
  // Trả về chiều thuận cho Cảm biến Độ Đục (Module xuất Volt thấp = sạch, Volt
  // cao = đục)
  ntu = map(adcNTU, 0, 4095, 0, 3000);
  if (ntu < 0)
    ntu = 0;
  if (ntu > 3000)
    ntu = 3000;

  // 3. Đọc nhiệt độ (Bật lại Delay 94ms để hỗ trợ cáp cảm biến Parasite Power)
  sensorsTemp.requestTemperatures();
  float t = sensorsTemp.getTempCByIndex(0);
  temp_ = (t <= -100.0f || t == 85.0f) ? -127.0f : t;
}

// ── Cài đặt ban đầu ───────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // SỬA LỖI GPS: Tăng bộ đệm Serial lên 1024 bytes để chống tràn dữ liệu gây
  // mất/sai tọa độ (Vì thư viện LoRa và Nhiệt độ chiếm dụng CPU làm quá trình
  // đọc GPS bị gián đoạn ~250ms)
  Serial1.setRxBufferSize(1024);
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  dungLai();

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // Khởi động INA219
  if (!ina219.begin())
    Serial.println("[WARN] INA219 khong tim thay");

  // Khởi động MPU9250
  if (!mpu.init()) {
    Serial.println("[ERR] Khong the ket noi den MPU9250_WE o dia chi 0x68");
  } else {
    Serial.println("[OK] Khoi dong thanh cong La Ban MPU9250_WE (MPU6500)!");
    mpu.autoOffsets();

    if (!mpu.initMagnetometer()) {
      Serial.println(
          "[ERR] Khong the ket noi voi Chip La ban Tu Truong (AK8963)!");
    } else {
      Serial.println("[OK] La ban AK8963 da san sang dong bo!");
      mpu.setMagOpMode(AK8963_CONT_MODE_100HZ);
    }
  }

  // Khởi động Cảm biến nhiệt độ
  sensorsTemp.begin();
  sensorsTemp.setWaitForConversion(true);
  sensorsTemp.setResolution(9);

  // XÓA SẠCH BỘ NHỚ NVS CỦA WIFI ESP32!
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(100);

  // Khởi động ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  // ĐỒNG BỘ KÊNH WI-FI CHUẨN XÁC VÀ TỰ ĐỘNG 100%:
  // Kết nối thẳng vào AP của Tay Cầm. Wi-Fi driver sẽ tự động bám theo Kênh của
  // Tay Cầm mãi mãi!
  Serial.println("Dang ket noi vao MEI-Controller de dong bo Kenh...");
  WiFi.begin("MEI-Controller", "mei12345");

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] ESP-NOW init fail");
    return;
  }
  esp_now_register_recv_cb(khiNhanDuocLenh);

  memset(&thongTinKetNoi, 0, sizeof(thongTinKetNoi));
  memcpy(thongTinKetNoi.peer_addr, diaChiBo, 6);
  thongTinKetNoi.channel = 0; // 0 = Tự động bám theo kênh của WiFi.begin()
  thongTinKetNoi.encrypt = false;
  esp_now_add_peer(&thongTinKetNoi);

  // Khởi động LoRa (Backup)
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (LoRa.begin(LORA_FREQ)) {
    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setTxPower(20);
    loraReady = true;
    Serial.println("[LoRa] OK");
  } else {
    Serial.println("[ERR] LoRa Fail");
  }

  Serial.println("[OK] Thuyen ESP32-C6 San Sang!");
}

// ── Vòng lặp chính ────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // (Đã chuyển hàm Motor sang thẳng hàm ngắt ESP-NOW để giảm delay tuyệt đối)

  // 1.1 Tính năng Tự Động Quay Về (Auto Return To Home) qua GPS
  static unsigned long lastNav = 0;
  if (isReturning == 1 && (now - lastNav > 200)) {
    lastNav = now;
    if (!homeSet || gpsStatus < 2) {
      dungLai();
    } else {
      double dist = TinyGPSPlus::distanceBetween(lat, lon, homeLat, homeLon);
      if (dist < 3.0) {
        dungLai();
        isReturning = 0;
      } else {
        double target = TinyGPSPlus::courseTo(lat, lon, homeLat, homeLon);
        double current = heading;
        double diff = target - current;
        if (diff < -180.0)
          diff += 360.0;
        if (diff > 180.0)
          diff -= 360.0;

        if (gpsSpeedKt < 0.5 && heading == 0.0f) {
          diThang(200);
        } else {
          if (diff > 25.0) {
            analogWrite(IN1, 200);
            analogWrite(IN2, 0);
            analogWrite(IN3, 0);
            analogWrite(IN4, 0);
          } else if (diff < -25.0) {
            analogWrite(IN1, 0);
            analogWrite(IN2, 0);
            analogWrite(IN3, 200);
            analogWrite(IN4, 0);
          } else {
            diThang(220);
          }
        }
      }
    }
  }

  // 1.5. Cập nhật dữ liệu từ MPU9250_WE
  static unsigned long lastMpu = 0;
  if (now - lastMpu >= 100) {
    lastMpu = now;
    xyzFloat angle = mpu.getAngles();
    pitch = angle.x;
    roll = angle.y;

    xyzFloat mag = mpu.getMagValues();
    if (isnan(mag.x) || isnan(mag.y) || isnan(mag.z) ||
        (mag.x == 0 && mag.y == 0 && mag.z == 0)) {
      if (gpsSpeedKt > 0.5) {
        heading = gpsCourse;
      } else if (heading < 0 || isnan(heading)) {
        heading = 0.0f;
      }
    } else {
      float pRad = pitch * DEG_TO_RAD;
      float rRad = roll * DEG_TO_RAD;
      float magXcomp = mag.x * cos(pRad) + mag.z * sin(pRad);
      float magYcomp = mag.x * sin(rRad) * sin(pRad) + mag.y * cos(rRad) -
                       mag.z * sin(rRad) * cos(pRad);

      heading = atan2(-magYcomp, magXcomp) * RAD_TO_DEG;
      if (heading < 0)
        heading += 360.0f;
    }
  }

  // 2. Cập nhật GPS liên tục bằng TinyGPSPlus
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      if (gps.location.isValid()) {
        lat = gps.location.lat();
        lon = gps.location.lng();
        gpsStatus = 2;
        if (!homeSet && lat != 0.0 && lon != 0.0) {
          homeLat = lat;
          homeLon = lon;
          homeSet = true;
        }
      }
      if (gps.speed.isValid()) {
        gpsSpeedKt = gps.speed.knots();
        if (gpsSpeedKt < 1.5f)
          gpsSpeedKt = 0.0f; // Lọc bỏ tốc độ ảo (Drift) khi đứng im
      }
      if (gps.course.isValid())
        gpsCourse = gps.course.deg();
    }
  }

  // 3. Đo cảm biến và gửi về Tay Cầm mỗi 200ms
  static unsigned long lastRead = 0;
  if (now - lastRead >= 200) {
    lastRead = now;

    docCamBien();

    duLieuGui.ph = ph;
    duLieuGui.tds = tds;
    duLieuGui.temp = temp_;
    duLieuGui.ntu = ntu;
    duLieuGui.heading = heading;
    duLieuGui.pitch = pitch;
    duLieuGui.roll = roll;
    duLieuGui.gpsStatus = gpsStatus;
    duLieuGui.lat = lat;
    duLieuGui.lon = lon;
    duLieuGui.gpsSpeedKt = gpsSpeedKt;
    duLieuGui.gpsCourse = gpsCourse;
    duLieuGui.batVolt = batVolt;
    duLieuGui.isReturning = isReturning;
    duLieuGui.boatPing = ++boatPing;

    // Bắn qua ESP-NOW
    esp_now_send(diaChiBo, (uint8_t *)&duLieuGui, sizeof(duLieuGui));

    // Bắn qua LoRa
    if (loraReady) {
      LoRa.beginPacket();
      LoRa.write((uint8_t *)&duLieuGui, sizeof(duLieuGui));
      LoRa.endPacket();
    }
  }
}
