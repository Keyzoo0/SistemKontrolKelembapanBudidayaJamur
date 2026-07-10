/*
 * ================================================================
 *  SISTEM KONTROL KELEMBAPAN BUDIDAYA JAMUR — ESP32
 * ================================================================
 *  Board     : ESP32 DevKit V1 (flash 4MB)
 *  Partisi   : Huge APP (3MB No OTA / 1MB SPIFFS)  -> LittleFS
 *  Core      : esp32 3.x (API LEDC baru)
 *
 *  WIRING
 *  ------------------------------------------------
 *  SHT31 (I2C)        : SDA=21, SCL=22, addr 0x44
 *  LCD 16x2 (I2C)     : SDA=21, SCL=22, addr 0x27 / 0x3F (auto-deteksi)
 *  Mist maker (PWM)   : GPIO 25, 100 Hz, 10-bit (0..1023)
 *  Sensor air penuh   : GPIO 26 (INPUT_PULLUP, HIGH = air penuh)
 *  Relay valve air    : GPIO 19 (HIGH = buka kran)
 *  Relay fan kabut    : GPIO 18 (HIGH = fan nyala)
 *
 *  PRINSIP KONTROL (AI PREDICTIVE CONTROL)
 *  ------------------------------------------------
 *  1. SHT31 dibaca tiap 2 detik -> log 3 menit ke belakang (90 titik).
 *  2. Model peramalan Holt (double exponential smoothing) memprediksi
 *     kelembapan sampai 5 menit ke depan.
 *  3. Kontroler prediktif: PWM mist dihitung dari SELISIH prediksi
 *     terhadap target 80%, ditambah feedforward cuaca luar
 *     (Open-Meteo: makin panas & kering udara luar, base PWM naik).
 *  4. Aturan override (selalu menang atas AI):
 *       RH < 70%        -> kabut 100%
 *       70..80%         -> keluaran AI prediktif
 *       80..90%         -> PWM diperlambat linier (cap menuju 0)
 *       RH >= 90%       -> kabut MATI (berlaku juga saat mode manual)
 *  5. Fan mengikuti mist: PWM > 0 -> fan ON (bisa manual dari web).
 *  6. Air: pelampung LOW (kurang) -> valve buka sampai penuh,
 *     dengan pengaman timeout isi 5 menit (alarm anti banjir).
 *
 *  WEB  : dashboard LittleFS (data/index.html), API /api/data & /api/control
 *  ================================================================
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <FS.h>
#include <LittleFS.h>
#include <Wire.h>
#include <time.h>
#include <ArduinoJson.h>
#include <Adafruit_SHT31.h>
#include <LiquidCrystal_I2C.h>

// ================== KONFIGURASI WIFI ==================
const char* WIFI_SSID = "robot";
const char* WIFI_PASS = "12345678";
const char* AP_SSID   = "JamurControl";   // fallback bila WiFi gagal
const char* AP_PASS   = "jamur12345";

// ================== KONFIGURASI PIN ==================
#define PIN_MIST    25
#define PIN_FLOAT   26
#define PIN_VALVE   19
#define PIN_FAN     18
#define I2C_SDA     21
#define I2C_SCL     22
#define SHT31_ADDR  0x44

// PWM mist maker
#define PWM_FREQ    100
#define PWM_RES     10
#define PWM_MAX     1023

// Level logika (ubah bila modul relay aktif-LOW)
#define RELAY_ON    HIGH
#define RELAY_OFF   LOW
#define FLOAT_FULL  HIGH   // pelampung HIGH = air penuh, LOW = air kurang

// ================== PARAMETER KONTROL ==================
const float RH_LOW    = 70.0f;   // di bawah ini kabut penuh
const float RH_TARGET = 80.0f;   // target tengah pita ideal
const float RH_TAPER  = 80.0f;   // mulai perlambat PWM
const float RH_CUT    = 90.0f;   // kabut mati total
const float KP        = 8.0f;    // gain prediktif (% PWM per %RH)
const float MIST_SLEW = 10.0f;   // maks kenaikan PWM per siklus (%)

// Peramalan Holt (double exponential smoothing), sampel per 2 dtk
const float HOLT_ALPHA = 0.30f;
const float HOLT_BETA  = 0.05f;
const float H_CTRL_S   = 120.0f; // horizon prediksi utk kontrol (dtk)

// Interval tugas (ms)
const unsigned long SENSOR_MS    = 2000;
const unsigned long WATER_MS     = 200;
const unsigned long LCD_MS       = 500;
const unsigned long WEATHER_MS   = 300000;  // cuaca tiap 5 menit
const unsigned long WIFI_CHK_MS  = 10000;
const unsigned long FILL_TIMEOUT_MS = 300000; // 5 menit isi terus = alarm

// Cuaca luar: Sukosewu, Bojonegoro (Open-Meteo)
const char* WEATHER_URL =
  "https://api.open-meteo.com/v1/forecast"
  "?latitude=-7.16&longitude=111.89"
  "&current=temperature_2m,relative_humidity_2m,weather_code"
  "&timezone=Asia%2FJakarta";

// ================== OBJEK GLOBAL ==================
Adafruit_SHT31    sht;
LiquidCrystal_I2C* lcd = nullptr;
WebServer         server(80);

// ================== STATE SENSOR & LOG ==================
const int HIST_N = 90;            // 90 titik x 2 dtk = 3 menit
float histT[HIST_N], histH[HIST_N];
int   histHead = 0, histCount = 0;

float curT = NAN, curH = NAN;
bool  readingValid = false;
bool  shtOk = false;
int   shtFails = 0;

// Model Holt
float holtLevel = NAN, holtTrend = 0.0f;

// ================== STATE AKTUATOR ==================
enum CtrlMode { MODE_AUTO, MODE_MANUAL };
CtrlMode mistMode  = MODE_AUTO;
CtrlMode fanMode   = MODE_AUTO;
CtrlMode valveMode = MODE_AUTO;

float mistManualPct = 0.0f;
bool  fanManualOn   = false;
bool  valveManualOn = false;

float mistPct  = 0.0f;   // PWM terpasang saat ini (%)
bool  fanOn    = false;
bool  valveOn  = false;
bool  safetyCut = false;
String aiTxt   = "Menunggu data sensor...";

// Air
bool waterFull = false;
bool fillAlarm = false;
unsigned long fillStartMs = 0;

// ================== STATE CUACA ==================
bool   wOk = false;
float  wT = NAN, wRH = NAN;
int    wCode = -1;
String wDesc = "-";
unsigned long wAtMs = 0;

// ================== STATE JARINGAN ==================
bool staOk = false;
bool apOn  = false;
unsigned long staLostMs = 0;

// Timer loop
unsigned long tSensor = 0, tWater = 0, tLcd = 0, tWeather = 0, tWifi = 0;

// ================== UTIL ==================
static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static inline float round1(float v) {
  return isnan(v) ? v : roundf(v * 10.0f) / 10.0f;
}

String artiKodeCuaca(int code) {
  switch (code) {
    case 0:  return "Cerah";
    case 1:  return "Sebagian Cerah";
    case 2:  return "Berawan Sebagian";
    case 3:  return "Mendung";
    case 45: return "Berkabut";
    case 48: return "Kabut Embun Beku";
    case 51: return "Gerimis Ringan";
    case 53: return "Gerimis Sedang";
    case 55: return "Gerimis Lebat";
    case 61: return "Hujan Ringan";
    case 63: return "Hujan Sedang";
    case 65: return "Hujan Lebat";
    case 71: return "Salju Ringan";
    case 73: return "Salju Sedang";
    case 75: return "Salju Lebat";
    case 80: return "Hujan Lokal Ringan";
    case 81: return "Hujan Lokal Sedang";
    case 82: return "Hujan Lokal Lebat";
    case 95: return "Badai Petir";
    case 96: return "Badai Petir + Es Ringan";
    case 99: return "Badai Petir + Es Lebat";
    default: return "Tidak Diketahui (" + String(code) + ")";
  }
}

// ================== LCD ==================
void lcdLine(uint8_t row, String s) {
  if (!lcd) return;
  if (s.length() > 16) s = s.substring(0, 16);
  while (s.length() < 16) s += ' ';
  lcd->setCursor(0, row);
  lcd->print(s);
}

void lcdBoot(const String& msg) {
  Serial.println("[BOOT] " + msg);
  lcdLine(1, msg);
}

bool i2cAda(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void initLcd() {
  uint8_t addr = 0;
  if (i2cAda(0x27)) addr = 0x27;
  else if (i2cAda(0x3F)) addr = 0x3F;
  if (addr) {
    lcd = new LiquidCrystal_I2C(addr, 16, 2);
    lcd->init();
    lcd->backlight();
    Serial.printf("[BOOT] LCD ditemukan di 0x%02X\n", addr);
  } else {
    Serial.println("[BOOT] LCD tidak ditemukan (lanjut tanpa LCD)");
  }
}

// Halaman LCD saat runtime
void lcdRuntime() {
  static uint8_t tick = 0, page = 0;
  static bool blink = false;
  tick++;
  blink = !blink;
  if (tick >= 7) { tick = 0; page = (page + 1) % 4; }   // ganti halaman ~3.5 dtk

  // Halaman alarm punya prioritas
  if (fillAlarm || safetyCut) {
    if (blink) {
      lcdLine(0, fillAlarm ? "!! ALARM AIR !!" : "!! RH >= 90% !!");
      lcdLine(1, fillAlarm ? "Cek kran/plmpung" : "Kabut dimatikan");
    } else {
      lcdLine(0, "");
      lcdLine(1, fillAlarm ? "Reset lewat web" : "Tunggu RH turun");
    }
    return;
  }

  char l1[20], l2[20];
  switch (page) {
    case 0: { // sensor + aktuator
      if (readingValid) snprintf(l1, sizeof(l1), "T:%.1fC H:%.1f%%", curT, curH);
      else              snprintf(l1, sizeof(l1), "T:--.-C H:--.-%%");
      snprintf(l2, sizeof(l2), "Kabut:%3d%% F:%s", (int)mistPct, fanOn ? "ON" : "OFF");
      break;
    }
    case 1: { // air + prediksi
      snprintf(l1, sizeof(l1), "Air:%s V:%s", waterFull ? "PENUH " : "KURANG", valveOn ? "ON" : "OFF");
      float p5 = isnan(holtLevel) ? NAN : clampf(holtLevel + holtTrend * (300.0f / 2.0f), 0, 100);
      if (isnan(p5)) snprintf(l2, sizeof(l2), "Prediksi5m: --%%");
      else           snprintf(l2, sizeof(l2), "Prediksi5m:%.1f%%", p5);
      break;
    }
    case 2: { // cuaca luar
      if (wOk) {
        snprintf(l1, sizeof(l1), "Luar:%.1fC %d%%", wT, (int)wRH);
        strncpy(l2, wDesc.c_str(), sizeof(l2) - 1); l2[sizeof(l2)-1] = 0;
      } else {
        snprintf(l1, sizeof(l1), "Cuaca luar: -");
        snprintf(l2, sizeof(l2), "belum ada data");
      }
      break;
    }
    default: { // jaringan + jam
      IPAddress ip = staOk ? WiFi.localIP() : WiFi.softAPIP();
      snprintf(l1, sizeof(l1), "%s", ip.toString().c_str());
      time_t now = time(nullptr);
      if (now > 1600000000) {
        struct tm tmInfo;
        localtime_r(&now, &tmInfo);
        snprintf(l2, sizeof(l2), "%02d:%02d:%02d %s", tmInfo.tm_hour, tmInfo.tm_min, tmInfo.tm_sec,
                 mistMode == MODE_AUTO ? "AUTO" : "MANUAL");
      } else {
        snprintf(l2, sizeof(l2), "--:--:-- %s", mistMode == MODE_AUTO ? "AUTO" : "MANUAL");
      }
      break;
    }
  }
  lcdLine(0, String(l1));
  lcdLine(1, String(l2));
}

// ================== MODEL PERAMALAN (AI) ==================
void holtUpdate(float h) {
  if (isnan(holtLevel)) { holtLevel = h; holtTrend = 0; return; }
  float prev = holtLevel;
  holtLevel = HOLT_ALPHA * h + (1.0f - HOLT_ALPHA) * (holtLevel + holtTrend);
  holtTrend = HOLT_BETA * (holtLevel - prev) + (1.0f - HOLT_BETA) * holtTrend;
  // batasi tren agar ramalan tidak liar (maks ~4.5 %RH per menit)
  holtTrend = clampf(holtTrend, -0.15f, 0.15f);
}

float forecastRH(float detik) {
  if (isnan(holtLevel)) return NAN;
  return clampf(holtLevel + holtTrend * (detik / 2.0f), 0.0f, 100.0f);
}

// ================== LOG RIWAYAT ==================
void pushHist(float t, float h) {
  histT[histHead] = t;
  histH[histHead] = h;
  histHead = (histHead + 1) % HIST_N;
  if (histCount < HIST_N) histCount++;
}

// ================== KONTROL MIST + FAN ==================
void computeMist() {
  float pct = 0;
  String txt;

  if (mistMode == MODE_MANUAL) {
    pct = mistManualPct;
    txt = "MANUAL: kabut " + String((int)pct) + "%";
  } else if (!readingValid) {
    pct = 0;
    txt = "Sensor SHT31 gagal - kabut OFF (fail-safe)";
  } else {
    float rh     = curH;
    float rhPred = forecastRH(H_CTRL_S);
    if (isnan(rhPred)) rhPred = rh;

    // feedforward cuaca luar: makin kering & panas, base PWM makin besar
    float ff = 0;
    if (wOk && !isnan(wRH)) {
      float kering = clampf((85.0f - wRH) / 60.0f, 0, 1);
      float panas  = clampf((wT - 24.0f) / 12.0f, 0, 1);
      ff = 15.0f * kering + 10.0f * panas;
    }

    float ai = clampf(ff + KP * (RH_TARGET - rhPred), 0.0f, 100.0f);

    if (rh < RH_LOW) {
      pct = 100;
      txt = "RH " + String(rh, 1) + "% < 70% - kabut PENUH";
    } else if (rh <= RH_TAPER) {
      pct = ai;
      txt = "AI: prediksi " + String(rhPred, 1) + "% (target 80%) - kabut " + String((int)ai) + "%";
    } else if (rh < RH_CUT) {
      float cap = (RH_CUT - rh) / (RH_CUT - RH_TAPER) * 100.0f;
      pct = fminf(ai, cap);
      txt = "RH " + String(rh, 1) + "% > 80% - kabut diperlambat (maks " + String((int)cap) + "%)";
    } else {
      pct = 0;
    }

    // slew-rate: kenaikan dibatasi agar halus, penurunan bebas (aman)
    if (pct > mistPct + MIST_SLEW) pct = mistPct + MIST_SLEW;
  }

  // SAFETY: RH >= 90% mematikan kabut di SEMUA mode
  safetyCut = false;
  if (readingValid && curH >= RH_CUT) {
    pct = 0;
    safetyCut = true;
    txt = "RH " + String(curH, 1) + "% >= 90% - kabut DIMATIKAN (safety)";
  }

  mistPct = clampf(pct, 0.0f, 100.0f);
  ledcWrite(PIN_MIST, (uint32_t)(mistPct * PWM_MAX / 100.0f + 0.5f));

  // fan mengikuti mist (atau manual)
  fanOn = (fanMode == MODE_AUTO) ? (mistPct > 0.5f) : fanManualOn;
  digitalWrite(PIN_FAN, fanOn ? RELAY_ON : RELAY_OFF);

  aiTxt = txt;
}

// ================== SENSOR TIAP 2 DETIK ==================
void tickSensor() {
  float t = sht.readTemperature();
  float h = sht.readHumidity();
  bool good = !isnan(t) && !isnan(h) && h >= 0.0f && h <= 100.0f;

  if (good) {
    curT = t; curH = h;
    readingValid = true;
    shtFails = 0;
    holtUpdate(h);
  } else {
    shtFails++;
    if (shtFails >= 3) readingValid = false;
    if (shtFails % 15 == 0) {          // coba init ulang tiap ~30 dtk
      shtOk = sht.begin(SHT31_ADDR);
      Serial.printf("[SENSOR] Re-init SHT31: %s\n", shtOk ? "OK" : "gagal");
    }
  }

  pushHist(good ? t : NAN, good ? h : NAN);
  computeMist();
}

// ================== KONTROL AIR (VALVE) ==================
void tickWater() {
  static bool rawLast = false;
  static uint8_t stableCnt = 0;
  static bool inited = false;

  bool raw = (digitalRead(PIN_FLOAT) == FLOAT_FULL);
  if (!inited) { inited = true; rawLast = raw; waterFull = raw; }
  if (raw == rawLast) {
    if (stableCnt < 5) stableCnt++;
  } else {
    rawLast = raw;
    stableCnt = 0;
  }
  if (stableCnt >= 5) waterFull = raw;   // debounce 1 detik

  if (waterFull) fillAlarm = false;      // air penuh -> alarm hilang

  bool want = (valveMode == MODE_MANUAL) ? valveManualOn
                                         : (!waterFull);
  if (fillAlarm) want = false;           // alarm memaksa kran tutup

  if (want && !valveOn) fillStartMs = millis();
  if (want && (millis() - fillStartMs > FILL_TIMEOUT_MS)) {
    fillAlarm = true;
    want = false;
    Serial.println("[ALARM] Pengisian air > 5 menit! Kran ditutup paksa (anti banjir).");
  }

  if (want != valveOn) {
    valveOn = want;
    Serial.printf("[AIR] Valve %s (air %s)\n", valveOn ? "BUKA" : "TUTUP", waterFull ? "penuh" : "kurang");
  }
  digitalWrite(PIN_VALVE, valveOn ? RELAY_ON : RELAY_OFF);
}

// ================== CUACA (OPEN-METEO) ==================
void fetchWeather() {
  if (WiFi.status() != WL_CONNECTED) return;
  Serial.println("[CUACA] Mengambil data Open-Meteo...");

  WiFiClientSecure client;
  client.setInsecure();               // API publik, tanpa verifikasi sertifikat
  HTTPClient http;
  http.setTimeout(7000);
  if (!http.begin(client, WEATHER_URL)) return;

  int code = http.GET();
  if (code == HTTP_CODE_OK) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, http.getString());
    if (!err) {
      wT    = doc["current"]["temperature_2m"] | NAN;
      wRH   = doc["current"]["relative_humidity_2m"] | NAN;
      wCode = doc["current"]["weather_code"] | -1;
      wDesc = artiKodeCuaca(wCode);
      wOk   = true;
      wAtMs = millis();
      Serial.printf("[CUACA] Sukosewu: %.1fC %.0f%% - %s\n", wT, wRH, wDesc.c_str());
    } else {
      Serial.printf("[CUACA] Parsing gagal: %s\n", err.c_str());
    }
  } else {
    Serial.printf("[CUACA] HTTP gagal, kode: %d\n", code);
  }
  http.end();
}

// ================== WEB SERVER ==================
void handleIndex() {
  File f = LittleFS.open("/index.html", "r");
  if (!f) {
    server.send(500, "text/plain", "index.html tidak ada - upload image LittleFS dulu");
    return;
  }
  server.streamFile(f, "text/html");
  f.close();
}

void handleData() {
  JsonDocument doc;

  doc["t"]  = round1(curT);
  doc["rh"] = round1(curH);
  doc["ok"] = readingValid;

  JsonObject m = doc["mist"].to<JsonObject>();
  m["mode"] = mistMode == MODE_AUTO ? "auto" : "manual";
  m["pct"]  = round1(mistPct);
  m["man"]  = round1(mistManualPct);

  JsonObject fn = doc["fan"].to<JsonObject>();
  fn["mode"] = fanMode == MODE_AUTO ? "auto" : "manual";
  fn["on"]   = fanOn;
  fn["man"]  = fanManualOn;

  JsonObject vl = doc["valve"].to<JsonObject>();
  vl["mode"] = valveMode == MODE_AUTO ? "auto" : "manual";
  vl["on"]   = valveOn;
  vl["man"]  = valveManualOn;

  doc["waterFull"] = waterFull;
  doc["fillAlarm"] = fillAlarm;
  doc["safety"]    = safetyCut;
  doc["ai"]        = aiTxt;

  doc["histStep"] = 2;
  JsonArray hT = doc["histT"].to<JsonArray>();
  JsonArray hH = doc["histH"].to<JsonArray>();
  for (int i = 0; i < histCount; i++) {
    int idx = (histHead - histCount + i + 2 * HIST_N) % HIST_N;
    hT.add(round1(histT[idx]));
    hH.add(round1(histH[idx]));
  }

  doc["fcStep"] = 10;
  JsonArray fc = doc["fc"].to<JsonArray>();
  for (int s = 10; s <= 300; s += 10) fc.add(round1(forecastRH((float)s)));

  JsonObject w = doc["w"].to<JsonObject>();
  w["ok"]   = wOk;
  w["t"]    = round1(wT);
  w["rh"]   = round1(wRH);
  w["code"] = wCode;
  w["desc"] = wDesc;
  w["ageS"] = wOk ? (uint32_t)((millis() - wAtMs) / 1000) : 0;

  JsonObject sys = doc["sys"].to<JsonObject>();
  sys["ip"]   = staOk ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  sys["rssi"] = staOk ? WiFi.RSSI() : 0;
  sys["heap"] = (uint32_t)ESP.getFreeHeap();
  sys["upS"]  = (uint32_t)(millis() / 1000);
  time_t nowT = time(nullptr);
  sys["time"] = (nowT > 1600000000) ? (uint32_t)nowT : 0;
  sys["ap"]   = apOn;

  String out;
  out.reserve(4096);
  serializeJson(doc, out);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

void handleControl() {
  String dev = server.arg("dev");

  if (dev == "mist") {
    if (server.hasArg("mode")) mistMode = server.arg("mode") == "manual" ? MODE_MANUAL : MODE_AUTO;
    if (server.hasArg("val"))  mistManualPct = clampf(server.arg("val").toFloat(), 0, 100);
  } else if (dev == "fan") {
    if (server.hasArg("mode")) fanMode = server.arg("mode") == "manual" ? MODE_MANUAL : MODE_AUTO;
    if (server.hasArg("val"))  fanManualOn = server.arg("val") == "1";
  } else if (dev == "valve") {
    if (server.hasArg("mode")) valveMode = server.arg("mode") == "manual" ? MODE_MANUAL : MODE_AUTO;
    if (server.hasArg("val"))  valveManualOn = server.arg("val") == "1";
  } else if (dev == "alarm") {
    fillAlarm = false;
    Serial.println("[AIR] Alarm pengisian direset lewat web");
  } else {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }

  Serial.printf("[WEB] Kontrol: %s\n", server.uri().c_str());
  computeMist();   // terapkan langsung
  server.send(200, "application/json", "{\"ok\":true}");
}

// ================== WIFI ==================
void wifiWatchdog() {
  bool conn = (WiFi.status() == WL_CONNECTED);
  if (conn != staOk) {
    staOk = conn;
    if (conn) {
      Serial.print("[WIFI] Terhubung kembali, IP: ");
      Serial.println(WiFi.localIP());
      staLostMs = 0;
    } else {
      Serial.println("[WIFI] Koneksi terputus, mencoba menghubungkan ulang...");
      staLostMs = millis();
    }
  }
  // >60 dtk tanpa WiFi -> nyalakan AP fallback agar web tetap bisa diakses
  if (!staOk && !apOn && staLostMs && (millis() - staLostMs > 60000)) {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(AP_SSID, AP_PASS);
    apOn = true;
    Serial.print("[WIFI] AP fallback aktif: ");
    Serial.print(AP_SSID);
    Serial.print(" | IP: ");
    Serial.println(WiFi.softAPIP());
  }
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n================================================");
  Serial.println("  SISTEM KONTROL KELEMBAPAN BUDIDAYA JAMUR");
  Serial.println("================================================");

  // Aktuator ke kondisi aman SEBELUM apapun
  pinMode(PIN_VALVE, OUTPUT); digitalWrite(PIN_VALVE, RELAY_OFF);
  pinMode(PIN_FAN,   OUTPUT); digitalWrite(PIN_FAN,   RELAY_OFF);
  pinMode(PIN_FLOAT, INPUT_PULLUP);
  ledcAttach(PIN_MIST, PWM_FREQ, PWM_RES);
  ledcWrite(PIN_MIST, 0);

  // I2C + LCD
  Wire.begin(I2C_SDA, I2C_SCL);
  initLcd();
  lcdLine(0, "Budidaya Jamur");
  lcdBoot("Memulai sistem");
  delay(600);

  // Sensor SHT31
  shtOk = sht.begin(SHT31_ADDR);
  lcdBoot(shtOk ? "SHT31: OK" : "SHT31: GAGAL!");
  delay(600);

  // LittleFS
  bool fsOk = LittleFS.begin(false);
  if (!fsOk) fsOk = LittleFS.begin(true);
  lcdBoot(fsOk ? "LittleFS: OK" : "LittleFS: GAGAL");
  delay(400);

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[BOOT] Menghubungkan ke WiFi \"%s\"", WIFI_SSID);
  lcdBoot("WiFi: " + String(WIFI_SSID));
  unsigned long t0 = millis();
  uint8_t dots = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(250);
    Serial.print(".");
    if (++dots % 2 == 0) lcdLine(1, "WiFi" + String("....").substring(0, (dots / 2) % 5));
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    staOk = true;
    WiFi.setAutoReconnect(true);
    Serial.print("[BOOT] WiFi terhubung! IP: ");
    Serial.println(WiFi.localIP());
    lcdBoot(WiFi.localIP().toString());
    configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com"); // WIB
  } else {
    Serial.println("[BOOT] WiFi gagal -> mode AP fallback");
    WiFi.mode(WIFI_AP_STA);          // STA tetap mencoba di latar belakang
    WiFi.softAP(AP_SSID, AP_PASS);
    apOn = true;
    Serial.print("[BOOT] AP: ");
    Serial.print(AP_SSID);
    Serial.print(" | IP: ");
    Serial.println(WiFi.softAPIP());
    lcdBoot("AP:" + WiFi.softAPIP().toString());
  }
  delay(800);

  // Web server
  server.on("/", HTTP_GET, handleIndex);
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/control", HTTP_POST, handleControl);
  server.onNotFound([]() { server.send(404, "text/plain", "404"); });
  server.begin();
  lcdBoot("Web server: OK");
  Serial.println("[BOOT] Web server aktif di port 80");
  delay(600);

  // Data awal
  if (staOk) fetchWeather();
  tickSensor();

  tSensor = tWater = tLcd = tWifi = millis();
  tWeather = millis();
  Serial.println("[BOOT] Sistem siap.\n");
}

// ================== LOOP (NON-BLOCKING) ==================
void loop() {
  server.handleClient();
  unsigned long now = millis();

  if (now - tSensor >= SENSOR_MS)  { tSensor = now; tickSensor(); }
  if (now - tWater  >= WATER_MS)   { tWater  = now; tickWater(); }
  if (now - tLcd    >= LCD_MS)     { tLcd    = now; lcdRuntime(); }
  if (now - tWifi   >= WIFI_CHK_MS){ tWifi   = now; wifiWatchdog(); }
  if (staOk && now - tWeather >= WEATHER_MS) { tWeather = now; fetchWeather(); }

  delay(2);
}
