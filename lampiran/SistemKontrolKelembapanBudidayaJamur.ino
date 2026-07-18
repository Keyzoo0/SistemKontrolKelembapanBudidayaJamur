#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <FS.h>
#include <LittleFS.h>
#include <Wire.h>
#include <time.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Adafruit_SHT31.h>
#include <LiquidCrystal_I2C.h>

#include "secrets.h"
const char* MDNS_NAME = "bagus";

#define PIN_MIST    25
#define PIN_FLOAT   26
#define PIN_VALVE   19
#define PIN_FAN     18
#define I2C_SDA     21
#define I2C_SCL     22
#define SHT31_ADDR  0x44

#define PWM_FREQ    100
#define PWM_RES     10
#define PWM_MAX     1023

#define RELAY_ON    HIGH
#define RELAY_OFF   LOW
#define FLOAT_FULL  HIGH

const float RH_LOW    = 70.0f;
const float RH_TARGET = 80.0f;
const float RH_TAPER  = 80.0f;
const float RH_CUT    = 90.0f;
const float KP        = 8.0f;
const float MIST_SLEW = 10.0f;

const float HOLT_ALPHA = 0.30f;
const float HOLT_BETA  = 0.05f;
const float H_CTRL_S   = 120.0f;

const unsigned long SENSOR_MS    = 2000;
const unsigned long WATER_MS     = 200;
const unsigned long LCD_MS       = 500;
const unsigned long WEATHER_MS   = 300000;
const unsigned long WIFI_CHK_MS  = 10000;
const unsigned long FILL_TIMEOUT_MS = 300000;

struct Lokasi { const char* nama; float lat; float lon; };
const Lokasi LOKASI[] = {
  { "Sukosewu, Bojonegoro",     -7.160f, 111.890f },
  { "Politeknik Negeri Malang", -7.946f, 112.615f },
};
const int N_LOKASI = sizeof(LOKASI) / sizeof(LOKASI[0]);

const char* WEATHER_URL_FMT =
  "https://api.open-meteo.com/v1/forecast"
  "?latitude=%.3f&longitude=%.3f"
  "&current=temperature_2m,relative_humidity_2m,weather_code"
  "&timezone=Asia%%2FJakarta";

Adafruit_SHT31    sht;
LiquidCrystal_I2C* lcd = nullptr;
WebServer         server(80);
Preferences       prefs;

const int HIST_N = 90;
float histT[HIST_N], histH[HIST_N];
int   histHead = 0, histCount = 0;

float curT = NAN, curH = NAN;
bool  readingValid = false;
bool  shtOk = false;
int   shtFails = 0;

float holtLevel = NAN, holtTrend = 0.0f;

enum CtrlMode { MODE_AUTO, MODE_MANUAL };
CtrlMode mistMode  = MODE_AUTO;
CtrlMode fanMode   = MODE_AUTO;
CtrlMode valveMode = MODE_AUTO;

float mistManualPct = 0.0f;
bool  fanManualOn   = false;
bool  valveManualOn = false;

float mistPct  = 0.0f;
bool  fanOn    = false;
bool  valveOn  = false;
bool  safetyCut = false;
String aiTxt   = "Menunggu data sensor...";

bool waterFull = false;
bool fillAlarm = false;
unsigned long fillStartMs = 0;

bool   wOk = false;
float  wT = NAN, wRH = NAN;
int    wCode = -1;
String wDesc = "-";
unsigned long wAtMs = 0;
int    lokasiIdx = 0;
bool   wantFetch = false;

bool staOk  = false;
bool ntpSet = false;
bool mdnsOn = false;

unsigned long tSensor = 0, tWater = 0, tLcd = 0, tWeather = 0, tWifi = 0;

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

void lcdRuntime() {
  static uint8_t tick = 0, page = 0;
  static bool blink = false;
  tick++;
  blink = !blink;
  if (tick >= 7) { tick = 0; page = (page + 1) % 4; }

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
    case 0: {
      if (readingValid) snprintf(l1, sizeof(l1), "T:%.1fC H:%.1f%%", curT, curH);
      else              snprintf(l1, sizeof(l1), "T:--.-C H:--.-%%");
      snprintf(l2, sizeof(l2), "Kabut:%3d%% F:%s", (int)mistPct, fanOn ? "ON" : "OFF");
      break;
    }
    case 1: {
      snprintf(l1, sizeof(l1), "Air:%s V:%s", waterFull ? "PENUH " : "KURANG", valveOn ? "ON" : "OFF");
      float p5 = isnan(holtLevel) ? NAN : clampf(holtLevel + holtTrend * (300.0f / 2.0f), 0, 100);
      if (isnan(p5)) snprintf(l2, sizeof(l2), "Prediksi5m: --%%");
      else           snprintf(l2, sizeof(l2), "Prediksi5m:%.1f%%", p5);
      break;
    }
    case 2: {
      if (wOk) {
        snprintf(l1, sizeof(l1), "Luar:%.1fC %d%%", wT, (int)wRH);
        strncpy(l2, wDesc.c_str(), sizeof(l2) - 1); l2[sizeof(l2)-1] = 0;
      } else {
        snprintf(l1, sizeof(l1), "Cuaca luar: -");
        snprintf(l2, sizeof(l2), "belum ada data");
      }
      break;
    }
    default: {
      static bool showMdns = false;
      showMdns = !showMdns;
      if (showMdns) {
        snprintf(l1, sizeof(l1), "bagus.local");
      } else if (staOk) {
        snprintf(l1, sizeof(l1), "%s", WiFi.localIP().toString().c_str());
      } else {
        snprintf(l1, sizeof(l1), "WiFi: mencoba...");
      }
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

void holtUpdate(float h) {
  if (isnan(holtLevel)) { holtLevel = h; holtTrend = 0; return; }
  float prev = holtLevel;
  holtLevel = HOLT_ALPHA * h + (1.0f - HOLT_ALPHA) * (holtLevel + holtTrend);
  holtTrend = HOLT_BETA * (holtLevel - prev) + (1.0f - HOLT_BETA) * holtTrend;
  holtTrend = clampf(holtTrend, -0.15f, 0.15f);
}

float forecastRH(float detik) {
  if (isnan(holtLevel)) return NAN;
  return clampf(holtLevel + holtTrend * (detik / 2.0f), 0.0f, 100.0f);
}

void pushHist(float t, float h) {
  histT[histHead] = t;
  histH[histHead] = h;
  histHead = (histHead + 1) % HIST_N;
  if (histCount < HIST_N) histCount++;
}

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

    if (pct > mistPct + MIST_SLEW) pct = mistPct + MIST_SLEW;
  }

  safetyCut = false;
  if (readingValid && curH >= RH_CUT) {
    pct = 0;
    safetyCut = true;
    txt = "RH " + String(curH, 1) + "% >= 90% - kabut DIMATIKAN (safety)";
  }

  mistPct = clampf(pct, 0.0f, 100.0f);
  ledcWrite(PIN_MIST, (uint32_t)(mistPct * PWM_MAX / 100.0f + 0.5f));

  fanOn = (fanMode == MODE_AUTO) ? (mistPct > 0.5f) : fanManualOn;
  digitalWrite(PIN_FAN, fanOn ? RELAY_ON : RELAY_OFF);

  aiTxt = txt;
}

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
    if (shtFails % 15 == 0) {
      shtOk = sht.begin(SHT31_ADDR);
      Serial.printf("[SENSOR] Re-init SHT31: %s\n", shtOk ? "OK" : "gagal");
    }
  }

  pushHist(good ? t : NAN, good ? h : NAN);
  computeMist();
}

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
  if (stableCnt >= 5) waterFull = raw;

  if (waterFull) fillAlarm = false;

  bool want = (valveMode == MODE_MANUAL) ? valveManualOn
                                         : (!waterFull);
  if (fillAlarm) want = false;

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

void fetchWeather() {
  if (WiFi.status() != WL_CONNECTED) return;
  char url[224];
  snprintf(url, sizeof(url), WEATHER_URL_FMT, LOKASI[lokasiIdx].lat, LOKASI[lokasiIdx].lon);
  Serial.printf("[CUACA] Mengambil data %s...\n", LOKASI[lokasiIdx].nama);

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(7000);
  if (!http.begin(client, url)) return;

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
      Serial.printf("[CUACA] %s: %.1fC %.0f%% - %s\n", LOKASI[lokasiIdx].nama, wT, wRH, wDesc.c_str());
    } else {
      Serial.printf("[CUACA] Parsing gagal: %s\n", err.c_str());
    }
  } else {
    Serial.printf("[CUACA] HTTP gagal, kode: %d\n", code);
  }
  http.end();
}

String tipeKonten(const String& path) {
  if (path.endsWith(".html")) return "text/html";
  if (path.endsWith(".css"))  return "text/css";
  if (path.endsWith(".js"))   return "application/javascript";
  if (path.endsWith(".json")) return "application/json";
  if (path.endsWith(".svg"))  return "image/svg+xml";
  if (path.endsWith(".png"))  return "image/png";
  if (path.endsWith(".ico"))  return "image/x-icon";
  return "text/plain";
}

bool kirimFile(String path) {
  if (path.endsWith("/")) path += "index.html";
  if (!LittleFS.exists(path)) return false;
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  server.streamFile(f, tipeKonten(path));
  f.close();
  return true;
}

void handleIndex() {
  if (!kirimFile("/index.html"))
    server.send(500, "text/plain", "index.html tidak ada - upload image LittleFS dulu");
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
  w["lok"]  = lokasiIdx;
  w["nama"] = LOKASI[lokasiIdx].nama;

  JsonObject sys = doc["sys"].to<JsonObject>();
  sys["ip"]   = WiFi.localIP().toString();
  sys["rssi"] = staOk ? WiFi.RSSI() : 0;
  sys["heap"] = (uint32_t)ESP.getFreeHeap();
  sys["upS"]  = (uint32_t)(millis() / 1000);
  time_t nowT = time(nullptr);
  sys["time"] = (nowT > 1600000000) ? (uint32_t)nowT : 0;

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
  } else if (dev == "lokasi") {
    int v = server.arg("val").toInt();
    if (v >= 0 && v < N_LOKASI && v != lokasiIdx) {
      lokasiIdx = v;
      prefs.putInt("lokasi", lokasiIdx);
      wOk = false;
      wantFetch = true;
      Serial.printf("[CUACA] Lokasi ramalan diganti: %s\n", LOKASI[lokasiIdx].nama);
    }
  } else if (dev == "alarm") {
    fillAlarm = false;
    Serial.println("[AIR] Alarm pengisian direset lewat web");
  } else {
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }

  Serial.printf("[WEB] Kontrol: %s\n", server.uri().c_str());
  computeMist();
  server.send(200, "application/json", "{\"ok\":true}");
}

void mulaiLayananJaringan() {
  if (!ntpSet) {
    configTime(7 * 3600, 0, "216.239.35.0", "216.239.35.4");
    ntpSet = true;
  }
  if (!mdnsOn) {
    if (MDNS.begin(MDNS_NAME)) {
      MDNS.addService("http", "tcp", 80);
      mdnsOn = true;
      Serial.println("[WIFI] mDNS aktif: http://bagus.local");
    } else {
      Serial.println("[WIFI] mDNS gagal dimulai");
    }
  }
}

void wifiWatchdog() {
  bool conn = (WiFi.status() == WL_CONNECTED);
  if (conn != staOk) {
    staOk = conn;
    if (conn) {
      Serial.print("[WIFI] Tersambung! IP: ");
      Serial.println(WiFi.localIP());
      mulaiLayananJaringan();
    } else {
      Serial.println("[WIFI] Terputus - menghubungkan ulang...");
    }
  }
  static unsigned long lastKick = 0;
  if (!conn && millis() - lastKick > 30000) {
    lastKick = millis();
    Serial.println("[WIFI] Mencoba menghubungkan ulang...");
    WiFi.reconnect();
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n================================================");
  Serial.println("  SISTEM KONTROL KELEMBAPAN BUDIDAYA JAMUR");
  Serial.println("================================================");

  pinMode(PIN_VALVE, OUTPUT); digitalWrite(PIN_VALVE, RELAY_OFF);
  pinMode(PIN_FAN,   OUTPUT); digitalWrite(PIN_FAN,   RELAY_OFF);
  pinMode(PIN_FLOAT, INPUT_PULLUP);
  ledcAttach(PIN_MIST, PWM_FREQ, PWM_RES);
  ledcWrite(PIN_MIST, 0);

  Wire.begin(I2C_SDA, I2C_SCL);
  initLcd();
  lcdLine(0, "Budidaya Jamur");
  lcdBoot("Memulai sistem");
  delay(600);

  shtOk = sht.begin(SHT31_ADDR);
  lcdBoot(shtOk ? "SHT31: OK" : "SHT31: GAGAL!");
  delay(600);

  bool fsOk = LittleFS.begin(false);
  if (!fsOk) fsOk = LittleFS.begin(true);
  lcdBoot(fsOk ? "LittleFS: OK" : "LittleFS: GAGAL");
  delay(400);

  prefs.begin("jamur", false);
  lokasiIdx = constrain(prefs.getInt("lokasi", 0), 0, N_LOKASI - 1);
  Serial.printf("[BOOT] Lokasi ramalan: %s\n", LOKASI[lokasiIdx].nama);

  WiFi.setHostname(MDNS_NAME);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
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
    Serial.print("[BOOT] WiFi terhubung! IP: ");
    Serial.println(WiFi.localIP());
    lcdBoot(WiFi.localIP().toString());
  } else {
    Serial.println("[BOOT] WiFi belum tersambung - terus mencoba di latar belakang");
    lcdBoot("WiFi: mencoba...");
  }
  delay(800);

  server.on("/", HTTP_GET, handleIndex);
  server.on("/api/data", HTTP_GET, handleData);
  server.on("/api/control", HTTP_POST, handleControl);
  server.onNotFound([]() {
    if (!kirimFile(server.uri())) server.send(404, "text/plain", "404");
  });
  server.begin();
  lcdBoot("Web server: OK");
  Serial.println("[BOOT] Web server aktif di port 80");
  delay(600);

  if (staOk) {
    mulaiLayananJaringan();
    lcdBoot("bagus.local");
    delay(600);
  }

  if (staOk) fetchWeather();
  tickSensor();

  tSensor = tWater = tLcd = tWifi = millis();
  tWeather = millis();
  Serial.println("[BOOT] Sistem siap.\n");
}

void loop() {
  server.handleClient();
  unsigned long now = millis();

  if (now - tSensor >= SENSOR_MS)  { tSensor = now; tickSensor(); }
  if (now - tWater  >= WATER_MS)   { tWater  = now; tickWater(); }
  if (now - tLcd    >= LCD_MS)     { tLcd    = now; lcdRuntime(); }
  if (now - tWifi   >= WIFI_CHK_MS){ tWifi   = now; wifiWatchdog(); }
  if (staOk && (wantFetch || now - tWeather >= WEATHER_MS)) { tWeather = now; wantFetch = false; fetchWeather(); }

  delay(2);
}
