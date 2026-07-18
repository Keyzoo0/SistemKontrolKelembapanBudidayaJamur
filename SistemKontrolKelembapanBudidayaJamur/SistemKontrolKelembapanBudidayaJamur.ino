/*
 * ================================================================
 *  SISTEM KONTROL KELEMBAPAN BUDIDAYA JAMUR — ESP32
 * ================================================================
 *  Board     : ESP32 DevKit V1 (flash 4MB)
 *  Partisi   : Huge APP (3MB No OTA / 1MB SPIFFS)  -> LittleFS
 *  Core      : esp32 3.x (API LEDC baru)
 *
 *  WIRING (SAMBUNGAN KABEL)
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
 *
 *  ----------------------------------------------------------------
 *  PETUNJUK MEMBACA PROGRAM INI (untuk yang belum paham pemrograman)
 *  ----------------------------------------------------------------
 *  - Baris yang diawali "//" atau diapit "/* ... *" + "/" adalah KOMENTAR:
 *    catatan untuk manusia, TIDAK dijalankan oleh ESP32.
 *  - "Variabel" = tempat menyimpan nilai (angka/teks) yang bisa berubah.
 *    Contoh tipe: int (bilangan bulat), float (bilangan desimal),
 *    bool (hanya true/false), String (teks).
 *  - "Fungsi" = kumpulan perintah yang diberi nama, supaya bisa
 *    dipanggil berulang kali. Contoh: computeMist() menghitung PWM kabut.
 *  - Program Arduino selalu punya 2 fungsi utama:
 *      setup()  -> dijalankan SEKALI saat alat baru menyala
 *      loop()   -> dijalankan BERULANG terus-menerus setelah setup()
 *  - "millis()" = stopwatch internal ESP32: jumlah milidetik sejak
 *    alat menyala. Dipakai untuk menjadwal tugas tanpa menghentikan
 *    program (non-blocking).
 * ================================================================
 */

/* ============ PUSTAKA (LIBRARY) YANG DIPAKAI ============
 * "#include" artinya memasukkan kode siap-pakai buatan orang lain,
 * supaya kita tidak menulis semuanya dari nol. */
#include <WiFi.h>               // koneksi WiFi ESP32
#include <WiFiClientSecure.h>   // koneksi HTTPS (untuk API cuaca)
#include <HTTPClient.h>         // mengirim permintaan HTTP GET ke internet
#include <WebServer.h>          // membuat web server di ESP32 (port 80)
#include <ESPmDNS.h>            // alamat lokal http://bagus.local
#include <FS.h>                 // dasar sistem berkas (file system)
#include <LittleFS.h>           // penyimpanan file web di flash ESP32
#include <Wire.h>               // komunikasi I2C (sensor SHT31 + LCD)
#include <time.h>               // jam waktu nyata (disinkron lewat NTP)
#include <Preferences.h>        // memori permanen NVS (simpan pilihan lokasi)
#include <ArduinoJson.h>        // membuat/membaca data format JSON
#include <Adafruit_SHT31.h>     // driver sensor suhu-kelembapan SHT31
#include <LiquidCrystal_I2C.h>  // driver LCD 16x2 lewat I2C

// ================== KONFIGURASI WIFI ==================
// Nama WiFi & password ditaruh di file terpisah "secrets.h" supaya
// tidak ikut ter-upload ke GitHub (keamanan). Salin dari secrets.h.example.
#include "secrets.h"   // berisi WIFI_SSID & WIFI_PASS
const char* MDNS_NAME = "bagus";          // dashboard: http://bagus.local

// ================== KONFIGURASI PIN ==================
// "#define NAMA nilai" = memberi nama pada sebuah angka supaya mudah dibaca.
// Angka di bawah adalah nomor kaki (GPIO) ESP32 tempat kabel disambung.
#define PIN_MIST    25    // keluaran PWM ke driver mist maker
#define PIN_FLOAT   26    // masukan sensor pelampung air
#define PIN_VALVE   19    // keluaran relay kran (valve) air
#define PIN_FAN     18    // keluaran relay kipas penyebar kabut
#define I2C_SDA     21    // jalur data I2C (bersama SHT31 + LCD)
#define I2C_SCL     22    // jalur clock I2C
#define SHT31_ADDR  0x44  // alamat I2C sensor SHT31 (heksadesimal)

// PWM mist maker.
// PWM = Pulse Width Modulation: menyala-matikan pin sangat cepat sehingga
// daya rata-rata bisa diatur (mirip "gas" untuk mist maker).
#define PWM_FREQ    100   // frekuensi 100 Hz (100x nyala-mati per detik)
#define PWM_RES     10    // resolusi 10-bit -> nilai duty 0..1023
#define PWM_MAX     1023  // nilai duty maksimum (kabut penuh)

// Level logika (ubah bila modul relay aktif-LOW)
#define RELAY_ON    HIGH  // tegangan yang MENYALAKAN relay
#define RELAY_OFF   LOW   // tegangan yang MEMATIKAN relay
#define FLOAT_FULL  HIGH  // pelampung HIGH = air penuh, LOW = air kurang

// ================== PARAMETER KONTROL ==================
// "const float" = angka desimal yang nilainya TETAP (tidak bisa berubah).
// Angka-angka ini adalah "aturan main" pengendali kelembapan.
const float RH_LOW    = 70.0f;   // di bawah ini kabut penuh (100%)
const float RH_TARGET = 80.0f;   // target tengah pita ideal 70-90%
const float RH_TAPER  = 80.0f;   // mulai perlambat PWM di atas nilai ini
const float RH_CUT    = 90.0f;   // kabut mati total di atas nilai ini
const float KP        = 8.0f;    // gain prediktif: tiap selisih 1 %RH -> 8% PWM
const float MIST_SLEW = 10.0f;   // maks KENAIKAN PWM per siklus (%), agar halus

// Peramalan Holt (double exponential smoothing), sampel per 2 dtk.
// Alpha = seberapa cepat model mengikuti nilai baru (0..1).
// Beta  = seberapa cepat model mengikuti perubahan tren (0..1).
const float HOLT_ALPHA = 0.30f;
const float HOLT_BETA  = 0.05f;
const float H_CTRL_S   = 120.0f; // horizon prediksi utk kontrol: +120 dtk (2 mnt)

// Interval tugas (ms). 1000 ms = 1 detik.
const unsigned long SENSOR_MS    = 2000;    // baca sensor tiap 2 detik
const unsigned long WATER_MS     = 200;     // cek pelampung tiap 0,2 detik
const unsigned long LCD_MS       = 500;     // segarkan LCD tiap 0,5 detik
const unsigned long WEATHER_MS   = 300000;  // ambil cuaca tiap 5 menit
const unsigned long WIFI_CHK_MS  = 10000;   // cek status WiFi tiap 10 detik
const unsigned long FILL_TIMEOUT_MS = 300000; // 5 menit isi terus = alarm

// Lokasi ramalan cuaca (Open-Meteo) — dipilih dari web, tersimpan di NVS.
// "struct" = tipe data buatan sendiri yang menggabungkan beberapa nilai;
// di sini satu Lokasi berisi: nama + garis lintang (lat) + garis bujur (lon).
struct Lokasi { const char* nama; float lat; float lon; };
const Lokasi LOKASI[] = {                       // daftar (array) pilihan lokasi
  { "Sukosewu, Bojonegoro",     -7.160f, 111.890f },
  { "Politeknik Negeri Malang", -7.946f, 112.615f },
};
// Jumlah anggota daftar dihitung otomatis: ukuran total / ukuran 1 anggota.
const int N_LOKASI = sizeof(LOKASI) / sizeof(LOKASI[0]);

// Cetakan (template) alamat API cuaca. "%.3f" akan diganti angka
// koordinat oleh snprintf() di fungsi fetchWeather().
const char* WEATHER_URL_FMT =
  "https://api.open-meteo.com/v1/forecast"
  "?latitude=%.3f&longitude=%.3f"
  "&current=temperature_2m,relative_humidity_2m,weather_code"
  "&timezone=Asia%%2FJakarta";

// ================== OBJEK GLOBAL ==================
// "Objek" = variabel pintar yang punya kemampuan (fungsi) sendiri.
Adafruit_SHT31    sht;             // objek sensor SHT31
LiquidCrystal_I2C* lcd = nullptr;  // penunjuk ke LCD; nullptr = belum ada/tidak terpasang
WebServer         server(80);      // web server di port 80 (port standar browser)
Preferences       prefs;           // akses memori permanen (NVS)

// ================== STATE SENSOR & LOG ==================
// "State" = kumpulan variabel yang menyimpan kondisi terkini sistem.
const int HIST_N = 90;            // 90 titik x 2 dtk = riwayat 3 menit
float histT[HIST_N], histH[HIST_N]; // array riwayat suhu & kelembapan
int   histHead = 0, histCount = 0;  // posisi tulis berikutnya & jumlah data terisi
// (riwayat memakai teknik "ring buffer": kalau penuh, data TERTUA ditimpa)

float curT = NAN, curH = NAN;     // suhu & RH terkini; NAN = belum ada angka
bool  readingValid = false;       // true bila pembacaan sensor sah
bool  shtOk = false;              // true bila sensor berhasil diinisialisasi
int   shtFails = 0;               // hitungan gagal baca beruntun

// Model Holt: "level" = perkiraan nilai saat ini, "trend" = arah perubahan.
float holtLevel = NAN, holtTrend = 0.0f;

// ================== STATE AKTUATOR ==================
// "enum" = daftar pilihan bernama. Mode tiap aktuator hanya boleh
// salah satu dari: MODE_AUTO (dikendalikan program) / MODE_MANUAL (dari web).
enum CtrlMode { MODE_AUTO, MODE_MANUAL };
CtrlMode mistMode  = MODE_AUTO;
CtrlMode fanMode   = MODE_AUTO;
CtrlMode valveMode = MODE_AUTO;

// Nilai yang diminta pengguna saat mode MANUAL:
float mistManualPct = 0.0f;   // PWM kabut manual (0..100 %)
bool  fanManualOn   = false;  // kipas manual nyala?
bool  valveManualOn = false;  // kran manual buka?

// Kondisi aktuator yang BENAR-BENAR terpasang saat ini:
float mistPct  = 0.0f;   // PWM terpasang saat ini (%)
bool  fanOn    = false;  // kipas sedang nyala?
bool  valveOn  = false;  // kran sedang terbuka?
bool  safetyCut = false; // true bila kabut dipaksa mati karena RH >= 90%
String aiTxt   = "Menunggu data sensor..."; // kalimat penjelasan keputusan AI (tampil di web)

// Air
bool waterFull = false;          // hasil baca pelampung (sudah di-debounce)
bool fillAlarm = false;          // alarm: pengisian terlalu lama (anti banjir)
unsigned long fillStartMs = 0;   // kapan kran mulai dibuka (utk hitung timeout)

// ================== STATE CUACA ==================
bool   wOk = false;          // sudah punya data cuaca yang sah?
float  wT = NAN, wRH = NAN;  // suhu & kelembapan luar ruangan
int    wCode = -1;           // kode cuaca WMO dari Open-Meteo
String wDesc = "-";          // arti kode cuaca dalam bahasa Indonesia
unsigned long wAtMs = 0;     // kapan data cuaca terakhir diterima (millis)
int    lokasiIdx = 0;        // indeks LOKASI[] aktif (0 = Bojonegoro, 1 = Polinema)
bool   wantFetch = false;    // minta ambil cuaca segera (setelah ganti lokasi)

// ================== STATE JARINGAN ==================
bool staOk  = false;  // WiFi sedang tersambung?
bool ntpSet = false;  // sinkron jam NTP sudah dijalankan?
bool mdnsOn = false;  // mDNS (bagus.local) sudah aktif?

// Timer loop: mencatat kapan tiap tugas terakhir dijalankan (millis).
unsigned long tSensor = 0, tWater = 0, tLcd = 0, tWeather = 0, tWifi = 0;

// ================== UTIL (FUNGSI BANTU KECIL) ==================
// clampf: membatasi nilai v agar tidak keluar rentang lo..hi.
// Contoh: clampf(120, 0, 100) menghasilkan 100.
static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
// round1: membulatkan ke 1 angka di belakang koma (76.4321 -> 76.4).
static inline float round1(float v) {
  return isnan(v) ? v : roundf(v * 10.0f) / 10.0f;
}

// Menerjemahkan kode cuaca WMO (angka dari Open-Meteo) menjadi teks.
// "switch" = memilih satu cabang sesuai nilai variabel code.
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
// lcdLine: menulis satu baris teks ke LCD (baris 0 = atas, 1 = bawah).
// Teks dipotong/diberi spasi agar pas 16 kolom, supaya sisa tulisan
// lama tidak tertinggal di layar.
void lcdLine(uint8_t row, String s) {
  if (!lcd) return;                          // LCD tidak terpasang -> lewati
  if (s.length() > 16) s = s.substring(0, 16); // potong bila lebih dari 16 huruf
  while (s.length() < 16) s += ' ';            // tambah spasi sampai penuh 16
  lcd->setCursor(0, row);                    // pindah kursor ke awal baris
  lcd->print(s);
}

// lcdBoot: menampilkan pesan tahap booting di LCD sekaligus ke Serial
// Monitor (layar teks di komputer, kecepatan 115200).
void lcdBoot(const String& msg) {
  Serial.println("[BOOT] " + msg);
  lcdLine(1, msg);
}

// i2cAda: mengetes apakah ada perangkat I2C di alamat tertentu.
// Caranya "mengetuk pintu" alamat itu; balasan 0 = ada yang menjawab.
bool i2cAda(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// initLcd: mencari LCD di dua alamat umum (0x27 atau 0x3F), lalu
// menyalakannya. Bila tidak ketemu, sistem tetap jalan tanpa LCD.
void initLcd() {
  uint8_t addr = 0;
  if (i2cAda(0x27)) addr = 0x27;
  else if (i2cAda(0x3F)) addr = 0x3F;
  if (addr) {
    lcd = new LiquidCrystal_I2C(addr, 16, 2);  // buat objek LCD 16 kolom x 2 baris
    lcd->init();
    lcd->backlight();                          // nyalakan lampu latar
    Serial.printf("[BOOT] LCD ditemukan di 0x%02X\n", addr);
  } else {
    Serial.println("[BOOT] LCD tidak ditemukan (lanjut tanpa LCD)");
  }
}

// lcdRuntime: tampilan LCD saat sistem sudah berjalan normal.
// Ada 4 halaman info yang berganti otomatis tiap ~3,5 detik.
// "static" di dalam fungsi = variabel yang nilainya AWET antar-pemanggilan.
void lcdRuntime() {
  static uint8_t tick = 0, page = 0;
  static bool blink = false;
  tick++;
  blink = !blink;                                       // pengedip teks alarm
  if (tick >= 7) { tick = 0; page = (page + 1) % 4; }   // ganti halaman ~3.5 dtk

  // Halaman alarm punya prioritas: bila ada alarm air / safety cut,
  // LCD hanya menampilkan peringatan berkedip.
  if (fillAlarm || safetyCut) {
    if (blink) {
      lcdLine(0, fillAlarm ? "!! ALARM AIR !!" : "!! RH >= 90% !!");
      lcdLine(1, fillAlarm ? "Cek kran/plmpung" : "Kabut dimatikan");
    } else {
      lcdLine(0, "");
      lcdLine(1, fillAlarm ? "Reset lewat web" : "Tunggu RH turun");
    }
    return;   // selesai; jangan tampilkan halaman biasa
  }

  // snprintf = menyusun teks ke dalam variabel (l1/l2) dengan format.
  // %.1f = angka desimal 1 digit koma, %d = bilangan bulat, %s = teks, %% = tanda %.
  char l1[20], l2[20];
  switch (page) {
    case 0: { // halaman 1: sensor + aktuator
      if (readingValid) snprintf(l1, sizeof(l1), "T:%.1fC H:%.1f%%", curT, curH);
      else              snprintf(l1, sizeof(l1), "T:--.-C H:--.-%%");
      snprintf(l2, sizeof(l2), "Kabut:%3d%% F:%s", (int)mistPct, fanOn ? "ON" : "OFF");
      break;
    }
    case 1: { // halaman 2: air + prediksi 5 menit
      snprintf(l1, sizeof(l1), "Air:%s V:%s", waterFull ? "PENUH " : "KURANG", valveOn ? "ON" : "OFF");
      // prediksi 5 menit = level + tren x (300 dtk / 2 dtk per langkah)
      float p5 = isnan(holtLevel) ? NAN : clampf(holtLevel + holtTrend * (300.0f / 2.0f), 0, 100);
      if (isnan(p5)) snprintf(l2, sizeof(l2), "Prediksi5m: --%%");
      else           snprintf(l2, sizeof(l2), "Prediksi5m:%.1f%%", p5);
      break;
    }
    case 2: { // halaman 3: cuaca luar ruangan
      if (wOk) {
        snprintf(l1, sizeof(l1), "Luar:%.1fC %d%%", wT, (int)wRH);
        strncpy(l2, wDesc.c_str(), sizeof(l2) - 1); l2[sizeof(l2)-1] = 0;
      } else {
        snprintf(l1, sizeof(l1), "Cuaca luar: -");
        snprintf(l2, sizeof(l2), "belum ada data");
      }
      break;
    }
    default: { // halaman 4: jaringan + jam (IP bergantian dengan bagus.local)
      static bool showMdns = false;
      showMdns = !showMdns;
      if (showMdns) {
        snprintf(l1, sizeof(l1), "bagus.local");
      } else if (staOk) {
        snprintf(l1, sizeof(l1), "%s", WiFi.localIP().toString().c_str());
      } else {
        snprintf(l1, sizeof(l1), "WiFi: mencoba...");
      }
      // time(nullptr) = detik sejak 1 Jan 1970; > 1600000000 berarti jam
      // sudah pernah sinkron NTP (bukan masih tahun 1970).
      time_t now = time(nullptr);
      if (now > 1600000000) {
        struct tm tmInfo;
        localtime_r(&now, &tmInfo);   // pecah menjadi jam:menit:detik lokal
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
// holtUpdate: memperbarui model Holt setiap ada data kelembapan baru (h).
// Ide dasarnya:
//   level = perkiraan "nilai sebenarnya" saat ini (rata-rata halus),
//   trend = perkiraan kecepatan naik/turun per langkah (per 2 detik).
// Rumus Holt mencampur data baru dengan perkiraan lama memakai bobot
// alpha & beta, sehingga hasilnya halus tapi tetap mengikuti perubahan.
void holtUpdate(float h) {
  if (isnan(holtLevel)) { holtLevel = h; holtTrend = 0; return; }  // data pertama
  float prev = holtLevel;
  holtLevel = HOLT_ALPHA * h + (1.0f - HOLT_ALPHA) * (holtLevel + holtTrend);
  holtTrend = HOLT_BETA * (holtLevel - prev) + (1.0f - HOLT_BETA) * holtTrend;
  // batasi tren agar ramalan tidak liar (maks ~4.5 %RH per menit)
  holtTrend = clampf(holtTrend, -0.15f, 0.15f);
}

// forecastRH: menghitung ramalan kelembapan "detik" ke depan.
// Ramalan = level sekarang + tren x jumlah langkah (1 langkah = 2 detik).
// Hasil dikunci di 0..100 karena kelembapan tidak mungkin di luar itu.
float forecastRH(float detik) {
  if (isnan(holtLevel)) return NAN;   // model belum punya data
  return clampf(holtLevel + holtTrend * (detik / 2.0f), 0.0f, 100.0f);
}

// ================== LOG RIWAYAT ==================
// pushHist: menyimpan 1 titik data (suhu, kelembapan) ke ring buffer.
// histHead menunjuk kotak berikutnya; setelah kotak ke-90 kembali ke
// kotak 0 (operator % = sisa pembagian), menimpa data tertua.
void pushHist(float t, float h) {
  histT[histHead] = t;
  histH[histHead] = h;
  histHead = (histHead + 1) % HIST_N;
  if (histCount < HIST_N) histCount++;
}

// ================== KONTROL MIST + FAN ==================
// computeMist: JANTUNG sistem — menentukan berapa persen PWM mist maker.
// Urutan keputusan:
//   1. Mode MANUAL  -> pakai nilai dari pengguna web.
//   2. Sensor rusak -> kabut mati (fail-safe, mencegah kelebihan kabut).
//   3. Mode AUTO    -> hitung AI prediktif + terapkan aturan override.
//   4. Paling akhir -> SAFETY: RH >= 90% mematikan kabut apapun modenya.
void computeMist() {
  float pct = 0;     // hasil akhir: persen PWM yang akan dipasang
  String txt;        // penjelasan keputusan (ditampilkan di web)

  if (mistMode == MODE_MANUAL) {
    pct = mistManualPct;
    txt = "MANUAL: kabut " + String((int)pct) + "%";
  } else if (!readingValid) {
    pct = 0;
    txt = "Sensor SHT31 gagal - kabut OFF (fail-safe)";
  } else {
    float rh     = curH;                  // kelembapan saat ini
    float rhPred = forecastRH(H_CTRL_S);  // prediksi 2 menit ke depan
    if (isnan(rhPred)) rhPred = rh;       // model belum siap? pakai nilai kini

    // feedforward cuaca luar: makin kering & panas, base PWM makin besar.
    // "Feedforward" = antisipasi dari kondisi luar SEBELUM efeknya terasa
    // di dalam kumbung (udara luar kering menyedot kelembapan kumbung).
    float ff = 0;
    if (wOk && !isnan(wRH)) {
      float kering = clampf((85.0f - wRH) / 60.0f, 0, 1);  // 0..1 makin kering
      float panas  = clampf((wT - 24.0f) / 12.0f, 0, 1);   // 0..1 makin panas
      ff = 15.0f * kering + 10.0f * panas;                 // sumbangan maks 25%
    }

    // Inti kontrol prediktif: PWM sebanding selisih target dengan PREDIKSI.
    // Kalau prediksi < 80% (bakal kering), PWM naik; kalau lebih, turun.
    float ai = clampf(ff + KP * (RH_TARGET - rhPred), 0.0f, 100.0f);

    // Aturan override berdasarkan kelembapan SAAT INI:
    if (rh < RH_LOW) {                    // terlalu kering -> gas penuh
      pct = 100;
      txt = "RH " + String(rh, 1) + "% < 70% - kabut PENUH";
    } else if (rh <= RH_TAPER) {          // zona nyaman -> ikuti AI
      pct = ai;
      txt = "AI: prediksi " + String(rhPred, 1) + "% (target 80%) - kabut " + String((int)ai) + "%";
    } else if (rh < RH_CUT) {             // 80..90%: batasi PWM secara linier
      // cap = plafon PWM; makin dekat 90% makin kecil (90% -> plafon 0)
      float cap = (RH_CUT - rh) / (RH_CUT - RH_TAPER) * 100.0f;
      pct = fminf(ai, cap);
      txt = "RH " + String(rh, 1) + "% > 80% - kabut diperlambat (maks " + String((int)cap) + "%)";
    } else {                              // >= 90% (ditangani safety di bawah)
      pct = 0;
    }

    // slew-rate: kenaikan dibatasi agar halus, penurunan bebas (aman)
    if (pct > mistPct + MIST_SLEW) pct = mistPct + MIST_SLEW;
  }

  // SAFETY: RH >= 90% mematikan kabut di SEMUA mode (termasuk manual).
  safetyCut = false;
  if (readingValid && curH >= RH_CUT) {
    pct = 0;
    safetyCut = true;
    txt = "RH " + String(curH, 1) + "% >= 90% - kabut DIMATIKAN (safety)";
  }

  // Pasang hasil ke perangkat keras: persen diubah ke duty 0..1023.
  // (+0.5 sebelum dibulatkan ke bawah = trik pembulatan terdekat)
  mistPct = clampf(pct, 0.0f, 100.0f);
  ledcWrite(PIN_MIST, (uint32_t)(mistPct * PWM_MAX / 100.0f + 0.5f));

  // fan mengikuti mist (atau manual): PWM > 0.5% dianggap kabut hidup.
  fanOn = (fanMode == MODE_AUTO) ? (mistPct > 0.5f) : fanManualOn;
  digitalWrite(PIN_FAN, fanOn ? RELAY_ON : RELAY_OFF);

  aiTxt = txt;   // simpan penjelasan untuk ditampilkan di dashboard
}

// ================== SENSOR TIAP 2 DETIK ==================
// tickSensor: membaca SHT31, memvalidasi hasilnya, memperbarui model AI,
// mencatat riwayat, lalu menghitung ulang kontrol kabut.
void tickSensor() {
  float t = sht.readTemperature();
  float h = sht.readHumidity();
  // Pembacaan dianggap sah bila bukan NAN dan kelembapan masuk akal (0-100).
  bool good = !isnan(t) && !isnan(h) && h >= 0.0f && h <= 100.0f;

  if (good) {
    curT = t; curH = h;
    readingValid = true;
    shtFails = 0;          // reset hitungan gagal
    holtUpdate(h);         // beri data baru ke model peramalan
  } else {
    shtFails++;
    if (shtFails >= 3) readingValid = false;  // 3x gagal beruntun = tidak valid
    if (shtFails % 15 == 0) {          // coba init ulang sensor tiap ~30 dtk
      shtOk = sht.begin(SHT31_ADDR);
      Serial.printf("[SENSOR] Re-init SHT31: %s\n", shtOk ? "OK" : "gagal");
    }
  }

  pushHist(good ? t : NAN, good ? h : NAN);  // catat ke riwayat (gagal = NAN)
  computeMist();                             // hitung ulang keputusan kabut
}

// ================== KONTROL AIR (VALVE) ==================
// tickWater: menjaga tandon air mist maker selalu terisi.
// - Pelampung dibaca dengan "debounce": nilai baru dipercaya setelah
//   stabil 5x berturut-turut (1 detik), supaya riak air tidak menipu.
// - AUTO: air kurang -> kran buka; penuh -> tutup.
// - Anti banjir: mengisi > 5 menit tanpa penuh = ada masalah
//   (selang lepas / pelampung macet) -> kran ditutup paksa + alarm.
void tickWater() {
  static bool rawLast = false;    // nilai mentah pembacaan sebelumnya
  static uint8_t stableCnt = 0;   // berapa kali berturut-turut nilainya sama
  static bool inited = false;     // penanda pembacaan pertama

  bool raw = (digitalRead(PIN_FLOAT) == FLOAT_FULL);
  if (!inited) { inited = true; rawLast = raw; waterFull = raw; }
  if (raw == rawLast) {
    if (stableCnt < 5) stableCnt++;
  } else {
    rawLast = raw;      // nilai berubah -> mulai hitung stabil dari nol
    stableCnt = 0;
  }
  if (stableCnt >= 5) waterFull = raw;   // debounce 1 detik (5 x 200 ms)

  if (waterFull) fillAlarm = false;      // air penuh -> alarm otomatis hilang

  // Tentukan kran seharusnya buka atau tutup:
  bool want = (valveMode == MODE_MANUAL) ? valveManualOn
                                         : (!waterFull);
  if (fillAlarm) want = false;           // alarm memaksa kran tutup

  if (want && !valveOn) fillStartMs = millis();   // mulai stopwatch pengisian
  if (want && (millis() - fillStartMs > FILL_TIMEOUT_MS)) {
    fillAlarm = true;
    want = false;
    Serial.println("[ALARM] Pengisian air > 5 menit! Kran ditutup paksa (anti banjir).");
  }

  if (want != valveOn) {                 // hanya lapor saat kondisi BERUBAH
    valveOn = want;
    Serial.printf("[AIR] Valve %s (air %s)\n", valveOn ? "BUKA" : "TUTUP", waterFull ? "penuh" : "kurang");
  }
  digitalWrite(PIN_VALVE, valveOn ? RELAY_ON : RELAY_OFF);
}

// ================== CUACA (OPEN-METEO) ==================
// fetchWeather: mengambil data cuaca terkini dari internet (API Open-Meteo,
// gratis tanpa API key) untuk lokasi yang sedang dipilih, lalu menyimpan
// suhu/kelembapan/kode cuaca luar ke variabel state.
void fetchWeather() {
  if (WiFi.status() != WL_CONNECTED) return;   // tidak ada internet -> batal
  // Susun URL: masukkan koordinat lokasi terpilih ke cetakan URL.
  char url[224];
  snprintf(url, sizeof(url), WEATHER_URL_FMT, LOKASI[lokasiIdx].lat, LOKASI[lokasiIdx].lon);
  Serial.printf("[CUACA] Mengambil data %s...\n", LOKASI[lokasiIdx].nama);

  WiFiClientSecure client;
  client.setInsecure();               // API publik, tanpa verifikasi sertifikat
  HTTPClient http;
  http.setTimeout(7000);              // menyerah bila 7 detik tidak ada jawaban
  if (!http.begin(client, url)) return;

  int code = http.GET();              // kirim permintaan; jawaban 200 = sukses
  if (code == HTTP_CODE_OK) {
    // Jawaban berupa teks JSON; diurai dengan ArduinoJson agar bisa
    // diambil nilainya per bagian. Operator "| NAN" = nilai cadangan
    // bila bagian itu tidak ada di jawaban.
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
  http.end();   // tutup koneksi, bebaskan memori
}

// ================== WEB SERVER ==================
// tipeKonten: menebak jenis file dari akhiran namanya, supaya browser
// tahu cara menampilkannya (Content-Type).
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

// kirimFile: menyajikan file statis dari LittleFS (index.html, style.css,
// app.js, ...) ke browser. Mengembalikan false bila file tidak ada.
bool kirimFile(String path) {
  if (path.endsWith("/")) path += "index.html";   // "/" berarti halaman utama
  if (!LittleFS.exists(path)) return false;
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  server.streamFile(f, tipeKonten(path));  // kirim isi file ke browser
  f.close();
  return true;
}

// handleIndex: dipanggil saat browser membuka alamat "/" (halaman utama).
void handleIndex() {
  if (!kirimFile("/index.html"))
    server.send(500, "text/plain", "index.html tidak ada - upload image LittleFS dulu");
}

// handleData: dipanggil saat dashboard meminta /api/data (tiap 2 detik).
// Semua kondisi sistem dikemas menjadi satu paket JSON lalu dikirim.
// JSON = format teks {"nama":nilai,...} yang mudah dibaca JavaScript.
void handleData() {
  JsonDocument doc;

  // --- sensor dalam kumbung ---
  doc["t"]  = round1(curT);
  doc["rh"] = round1(curH);
  doc["ok"] = readingValid;

  // --- status tiap aktuator: mode, kondisi nyata, nilai manualnya ---
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

  // --- air, alarm, safety, penjelasan AI ---
  doc["waterFull"] = waterFull;
  doc["fillAlarm"] = fillAlarm;
  doc["safety"]    = safetyCut;
  doc["ai"]        = aiTxt;

  // --- riwayat 3 menit (untuk sisi kiri grafik) ---
  // Ring buffer dibaca urut dari data TERTUA ke TERBARU.
  doc["histStep"] = 2;
  JsonArray hT = doc["histT"].to<JsonArray>();
  JsonArray hH = doc["histH"].to<JsonArray>();
  for (int i = 0; i < histCount; i++) {
    int idx = (histHead - histCount + i + 2 * HIST_N) % HIST_N;
    hT.add(round1(histT[idx]));
    hH.add(round1(histH[idx]));
  }

  // --- ramalan AI 5 menit (untuk sisi kanan grafik): tiap 10 dtk ---
  doc["fcStep"] = 10;
  JsonArray fc = doc["fc"].to<JsonArray>();
  for (int s = 10; s <= 300; s += 10) fc.add(round1(forecastRH((float)s)));

  // --- cuaca luar ruangan ---
  JsonObject w = doc["w"].to<JsonObject>();
  w["ok"]   = wOk;
  w["t"]    = round1(wT);
  w["rh"]   = round1(wRH);
  w["code"] = wCode;
  w["desc"] = wDesc;
  w["ageS"] = wOk ? (uint32_t)((millis() - wAtMs) / 1000) : 0;  // umur data (dtk)
  w["lok"]  = lokasiIdx;
  w["nama"] = LOKASI[lokasiIdx].nama;

  // --- info sistem (ditampilkan di footer dashboard) ---
  JsonObject sys = doc["sys"].to<JsonObject>();
  sys["ip"]   = WiFi.localIP().toString();
  sys["rssi"] = staOk ? WiFi.RSSI() : 0;          // kekuatan sinyal WiFi (dBm)
  sys["heap"] = (uint32_t)ESP.getFreeHeap();      // sisa memori RAM (byte)
  sys["upS"]  = (uint32_t)(millis() / 1000);      // lama menyala (detik)
  time_t nowT = time(nullptr);
  sys["time"] = (nowT > 1600000000) ? (uint32_t)nowT : 0;

  // Ubah dokumen JSON menjadi teks, lalu kirim ke browser.
  String out;
  out.reserve(4096);   // pesan memori dulu agar tidak terfragmentasi
  serializeJson(doc, out);
  server.sendHeader("Cache-Control", "no-store");  // jangan di-cache browser
  server.send(200, "application/json", out);
}

// handleControl: dipanggil saat pengguna menekan tombol/saklar di web
// (POST /api/control). Parameter "dev" menentukan perangkat yang diatur:
//   dev=mist|fan|valve : ganti mode auto/manual dan/atau nilai manual
//   dev=lokasi         : ganti lokasi ramalan cuaca (disimpan permanen)
//   dev=alarm          : reset alarm pengisian air
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
    if (v >= 0 && v < N_LOKASI && v != lokasiIdx) {   // hanya bila pilihan sah & berubah
      lokasiIdx = v;
      prefs.putInt("lokasi", lokasiIdx);  // simpan ke NVS: awet walau mati listrik
      wOk = false;              // data lama tidak berlaku utk lokasi baru
      wantFetch = true;         // ambil segera di loop berikutnya
      Serial.printf("[CUACA] Lokasi ramalan diganti: %s\n", LOKASI[lokasiIdx].nama);
    }
  } else if (dev == "alarm") {
    fillAlarm = false;
    Serial.println("[AIR] Alarm pengisian direset lewat web");
  } else {
    // dev tidak dikenal -> balas kode 400 (permintaan salah)
    server.send(400, "application/json", "{\"ok\":false}");
    return;
  }

  Serial.printf("[WEB] Kontrol: %s\n", server.uri().c_str());
  computeMist();   // terapkan perubahan langsung, tanpa menunggu 2 detik
  server.send(200, "application/json", "{\"ok\":true}");
}

// ================== WIFI ==================
// mulaiLayananJaringan: dipanggil sekali setelah WiFi tersambung
// (saat boot atau saat reconnect). Menyalakan 2 layanan:
//   1. NTP  = sinkron jam lewat internet (untuk jam di LCD & web).
//   2. mDNS = supaya dashboard bisa dibuka lewat nama http://bagus.local.
void mulaiLayananJaringan() {
  if (!ntpSet) {
    // NTP pakai IP langsung (time.google.com, anycast stabil): nama host di sini
    // memicu bug core 3.3.x — callback DNS SNTP bentrok dgn hostByName()
    // (dns_clear_cache tanpa lock TCPIP) -> assert udp_new_ip_type -> reboot loop.
    configTime(7 * 3600, 0, "216.239.35.0", "216.239.35.4"); // 7*3600 = zona WIB (UTC+7)
    ntpSet = true;
  }
  if (!mdnsOn) {
    if (MDNS.begin(MDNS_NAME)) {
      MDNS.addService("http", "tcp", 80);   // umumkan "ada web server di port 80"
      mdnsOn = true;
      Serial.println("[WIFI] mDNS aktif: http://bagus.local");
    } else {
      Serial.println("[WIFI] mDNS gagal dimulai");
    }
  }
}

// wifiWatchdog: "penjaga" koneksi WiFi, dicek tiap 10 detik dari loop().
// - Mencatat perubahan status (tersambung <-> terputus) ke Serial.
// - Saat tersambung kembali: menyalakan layanan jaringan bila belum.
// - Saat terputus: mendorong WiFi.reconnect() tiap 30 detik, karena
//   fitur autoReconnect bawaan kadang berhenti mencoba sendiri.
void wifiWatchdog() {
  bool conn = (WiFi.status() == WL_CONNECTED);
  if (conn != staOk) {                    // status BERUBAH sejak cek terakhir
    staOk = conn;
    if (conn) {
      Serial.print("[WIFI] Tersambung! IP: ");
      Serial.println(WiFi.localIP());
      mulaiLayananJaringan();
    } else {
      Serial.println("[WIFI] Terputus - menghubungkan ulang...");
    }
  }
  // autoReconnect kadang berhenti sendiri; dorong ulang tiap 30 dtk selama putus
  static unsigned long lastKick = 0;
  if (!conn && millis() - lastKick > 30000) {
    lastKick = millis();
    Serial.println("[WIFI] Mencoba menghubungkan ulang...");
    WiFi.reconnect();
  }
}

// ================== SETUP ==================
// setup() dijalankan SEKALI setiap alat menyala / di-reset.
// Urutan: amankan aktuator -> LCD -> sensor -> file system -> preferensi
// -> WiFi -> web server -> NTP/mDNS -> data awal.
void setup() {
  Serial.begin(115200);   // buka jalur Serial Monitor kecepatan 115200
  delay(300);             // beri waktu 0,3 dtk agar Serial siap
  Serial.println("\n================================================");
  Serial.println("  SISTEM KONTROL KELEMBAPAN BUDIDAYA JAMUR");
  Serial.println("================================================");

  // Aktuator ke kondisi aman SEBELUM apapun: semua relay mati, PWM 0.
  // Penting supaya kran/kipas/kabut tidak menyala sendiri saat boot.
  pinMode(PIN_VALVE, OUTPUT); digitalWrite(PIN_VALVE, RELAY_OFF);
  pinMode(PIN_FAN,   OUTPUT); digitalWrite(PIN_FAN,   RELAY_OFF);
  pinMode(PIN_FLOAT, INPUT_PULLUP);        // pelampung pakai resistor pull-up internal
  ledcAttach(PIN_MIST, PWM_FREQ, PWM_RES); // siapkan pin 25 sebagai keluaran PWM
  ledcWrite(PIN_MIST, 0);                  // kabut mati dulu

  // I2C + LCD
  Wire.begin(I2C_SDA, I2C_SCL);   // nyalakan jalur I2C di pin 21 & 22
  initLcd();
  lcdLine(0, "Budidaya Jamur");
  lcdBoot("Memulai sistem");
  delay(600);                     // jeda agar pesan sempat terbaca di LCD

  // Sensor SHT31
  shtOk = sht.begin(SHT31_ADDR);
  lcdBoot(shtOk ? "SHT31: OK" : "SHT31: GAGAL!");
  delay(600);

  // LittleFS: tempat file dashboard web tersimpan di flash.
  // begin(false) = coba pasang; gagal? begin(true) = format dulu lalu pasang.
  bool fsOk = LittleFS.begin(false);
  if (!fsOk) fsOk = LittleFS.begin(true);
  lcdBoot(fsOk ? "LittleFS: OK" : "LittleFS: GAGAL");
  delay(400);

  // Preferensi tersimpan (lokasi ramalan cuaca) dibaca dari memori NVS.
  // constrain menjaga nilainya tetap sah walau isi memori aneh.
  prefs.begin("jamur", false);
  lokasiIdx = constrain(prefs.getInt("lokasi", 0), 0, N_LOKASI - 1);
  Serial.printf("[BOOT] Lokasi ramalan: %s\n", LOKASI[lokasiIdx].nama);

  // WiFi — hanya mode station (ikut hotspot); bila gagal, terus dicoba dari loop
  WiFi.setHostname(MDNS_NAME);   // nama alat di router (DHCP)
  WiFi.mode(WIFI_STA);           // mode station = ESP32 jadi "klien" WiFi
  WiFi.setSleep(false);          // matikan mode hemat daya (respon web lebih cepat)
  WiFi.setAutoReconnect(true);   // minta core menyambung ulang otomatis
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[BOOT] Menghubungkan ke WiFi \"%s\"", WIFI_SSID);
  lcdBoot("WiFi: " + String(WIFI_SSID));
  // Tunggu maksimal 15 detik sambil menampilkan titik-titik animasi.
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
    // Tidak apa-apa: kontrol kelembapan tetap jalan; wifiWatchdog()
    // akan terus mencoba menyambung di belakang layar.
    Serial.println("[BOOT] WiFi belum tersambung - terus mencoba di latar belakang");
    lcdBoot("WiFi: mencoba...");
  }
  delay(800);

  // Web server: daftarkan alamat-alamat (route) yang dilayani.
  server.on("/", HTTP_GET, handleIndex);              // halaman utama
  server.on("/api/data", HTTP_GET, handleData);       // data JSON utk dashboard
  server.on("/api/control", HTTP_POST, handleControl);// perintah dari dashboard
  server.onNotFound([]() {
    // Alamat lain (mis. /style.css, /app.js): coba cari filenya di LittleFS.
    if (!kirimFile(server.uri())) server.send(404, "text/plain", "404");
  });
  server.begin();
  lcdBoot("Web server: OK");
  Serial.println("[BOOT] Web server aktif di port 80");
  delay(600);

  // NTP + mDNS (bagus.local) — dijalankan begitu WiFi tersambung
  if (staOk) {
    mulaiLayananJaringan();
    lcdBoot("bagus.local");
    delay(600);
  }

  // Data awal: cuaca (bila online) + pembacaan sensor pertama.
  if (staOk) fetchWeather();
  tickSensor();

  // Set semua stopwatch tugas ke "sekarang".
  tSensor = tWater = tLcd = tWifi = millis();
  tWeather = millis();
  Serial.println("[BOOT] Sistem siap.\n");
}

// ================== LOOP (NON-BLOCKING) ==================
// loop() diulang terus-menerus, ribuan kali per detik. Setiap tugas punya
// jadwalnya sendiri: "kalau sudah lewat X ms sejak terakhir, kerjakan lagi".
// Dengan cara ini tidak ada tugas yang saling menunggu (non-blocking).
void loop() {
  server.handleClient();       // layani permintaan browser (harus sering dipanggil)
  unsigned long now = millis();

  if (now - tSensor >= SENSOR_MS)  { tSensor = now; tickSensor(); }   // tiap 2 dtk
  if (now - tWater  >= WATER_MS)   { tWater  = now; tickWater(); }    // tiap 0,2 dtk
  if (now - tLcd    >= LCD_MS)     { tLcd    = now; lcdRuntime(); }   // tiap 0,5 dtk
  if (now - tWifi   >= WIFI_CHK_MS){ tWifi   = now; wifiWatchdog(); } // tiap 10 dtk
  // Cuaca: tiap 5 menit, ATAU segera bila baru ganti lokasi (wantFetch).
  if (staOk && (wantFetch || now - tWeather >= WEATHER_MS)) { tWeather = now; wantFetch = false; fetchWeather(); }

  delay(2);   // istirahat 2 ms: beri napas untuk proses WiFi internal
}
