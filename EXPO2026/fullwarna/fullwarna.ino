// Define DEBUG to enable debugging; comment it out to disable
//#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DEBUG_BEGIN(baud) Serial.begin(baud)
#else
  #define DEBUG_PRINT(...)    // Do nothing
  #define DEBUG_PRINTLN(...)  // Do nothing
  #define DEBUG_BEGIN(baud)   // Do nothing
#endif

// Line follower + mapping + gripper. Pinout & primitive ada di mapping.h/servo_config.h.

#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <Adafruit_TiCoServo.h>
#include "servo_config.h"  // pin & sudut servo gripper
// mapping.h di-include PALING BAWAH (bukan di sini) - isinya butuh
// strip/setMotor()/BASE_SPEED/dst yang baru didefinisikan di bawah sini.
void executeMapStep(unsigned int simpangKe, bool &actionDone);
void servoSweep(Adafruit_TiCoServo &s, int &posVar, int target);

// ================== PIN DEFINITION ==================
const uint8_t sensorPin[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

const uint8_t BTN_CALIB = 2;
const uint8_t BTN_START = 4;
const uint8_t BTN_STOP  = 12;

// kiri-kanan dibalik dari tesMotor.ino karena roda fisiknya kebalik
const uint8_t IN1 = 6;   // motor KANAN maju
const uint8_t IN2 = 11;  // motor KANAN mundur
const uint8_t IN3 = 3;   // motor KIRI maju
const uint8_t IN4 = 5;   // motor KIRI mundur

const uint8_t LED_PIN   = 13;
const uint8_t LED_COUNT = 8;

// ================== TUNING ==================
float KP = 0.5;
float KD = 0.3;

int BASE_SPEED = 140;     // kecepatan dasar saat lurus (0-255)
int MAX_SPEED  = 200;

int LINE_LOST_THRESHOLD = 400; // kalau semua sensor normalized-nya di bawah ini = line hilang
int PIVOT_SPEED = 150;         // kecepatan pivot saat nyari line

int SENSOR_ON_THRESHOLD = 500;         // >ini = dianggap "kena hitam" (dipakai LED, simpang, deteksi diangkat)
                                        // ini nilai normalized (0-1000) HASIL KALIBRASI, bukan ADC mentah
const unsigned long INTERSECTION_FLASH_MS = 200; // durasi LED kuning pas simpang kedeteksi

uint8_t LED_BRIGHTNESS = 20;   // 0-255, atur terang-redup semua LED sekaligus

const unsigned long DEBUG_PRINT_INTERVAL = 100;   // ms, biar serial ga banjir
const unsigned long CALIB_DURATION_MS   = 5000;   // 5 detik
const unsigned long FINISH_DURATION_MS  = 2000;   // durasi LED hijau di state FINISH sebelum auto balik IDLE

// ================== EEPROM (persist kalibrasi biar ga ilang tiap reset) ==================
const int   EEPROM_ADDR_MAGIC = 0;      // 1 byte penanda "sudah pernah kalibrasi"
const int   EEPROM_ADDR_DATA  = 1;      // calibMin[8] (16 byte) + calibMax[8] (16 byte)
const uint8_t EEPROM_MAGIC_VALUE = 0xA5;

// ================== STATE MACHINE ==================
// ST_FINISH = rute mapping selesai dilewati semua (bukan STOP manual dari
// tombol) - LED hijau blink sesaat, lalu OTOMATIS balik ke ST_IDLE lagi,
// state/counter di-reset kaya abis baru boot, siap buat run berikutnya.
// dipakai checkCalibButton() di bawah - didefinisikan di sini (bukan deket
// fungsinya) biar aman dari isu urutan compile auto-prototype Arduino,
// SAMA persis alasannya kayak enum Action/TurnAction sebelumnya.
enum PressType { PRESS_NONE, PRESS_SHORT, PRESS_LONG };

enum RobotState { ST_IDLE, ST_CALIBRATING, ST_RUNNING, ST_STOPPED, ST_FINISH };
RobotState currentState = ST_IDLE;
unsigned long finishStateEnteredAt = 0;

// ================== VARIABEL GLOBAL ==================
int calibMin[8];
int calibMax[8];
int normalized[8]; // 0 (putih) - 1000 (hitam)

int lastError = 0;
int lastErrorSign = 1; // buat nentuin arah pivot pas line hilang

bool intersectionActive = false;        // lagi di atas persimpangan (edge-trigger guard)
unsigned long intersectionFlashUntil = 0;
unsigned int intersectionCount = 0;

bool robotLifted = false;               // edge-trigger guard buat log "robot diangkat"

// ---- multi-plan mapping ----
// planNum dipilih user pas ST_IDLE (sebelum START), lewat short-press
// CALIBRATE (naik) / short-press STOP (turun). Dipakai sebagai outer
// switch di executeMapStep() (mapping.h) - lihat komentar di sana.
unsigned int planNum = 1;
const uint8_t PLAN_COUNT = 20; // plan 1 s/d 8

const unsigned long CALIB_LONG_PRESS_MS = 600; // ambang "tahan lama" buat CALIBRATE
unsigned long calibPressStart = 0;
bool calibLongFired = false;
unsigned long lastCalibChangeAt = 0;

unsigned long lastDebugPrint = 0;

bool lastReadCalib = HIGH;
bool lastReadStart = HIGH;
bool lastReadStop  = HIGH;
unsigned long lastStartChangeAt = 0;
unsigned long lastStopChangeAt  = 0;
bool startPressed;
bool stopPressed;

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

Adafruit_TiCoServo jepitServo;
Adafruit_TiCoServo angkatServo;
// posisi TERAKHIR yang di-write ke tiap servo - dipakai servoSweep() buat
// tau harus gerak dari mana ke target berikutnya (dipakai mid-route).
// Nilai awal = NAIK+BUKA, sama kayak posisi boot (lihat setup()).
int jepitPos  = JEPIT_BUKA_ANGLE;
int angkatPos = ANGKAT_NAIK_ANGLE;

void maju(unsigned long ms);
// ================== SETUP ==================
void setup() {
  DEBUG_BEGIN(115200);
  delay(200);

  DEBUG_PRINTLN();
  DEBUG_PRINTLN(F("========================================"));
  DEBUG_PRINTLN(F("   LINE FOLLOWER - SETUP DIMULAI"));
  DEBUG_PRINTLN(F("========================================"));

  for (uint8_t i = 0; i < 8; i++) pinMode(sensorPin[i], INPUT);
  DEBUG_PRINTLN(F("[OK] Sensor A0-A7 diinisialisasi"));

  // default range aman kalau user belum sempat kalibrasi
  for (uint8_t i = 0; i < 8; i++) {
    calibMin[i] = 0;
    calibMax[i] = 1023;
  }

  if (loadCalibrationFromEEPROM()) {
    DEBUG_PRINTLN(F("[OK] Kalibrasi lama ditemukan di EEPROM, langsung dipakai:"));
    for (uint8_t i = 0; i < 8; i++) {
      DEBUG_PRINT(F("  S")); DEBUG_PRINT(i);
      DEBUG_PRINT(F(": ")); DEBUG_PRINT(calibMin[i]);
      DEBUG_PRINT(F(" - ")); DEBUG_PRINTLN(calibMax[i]);
    }
  } else {
    DEBUG_PRINTLN(F("[INFO] Belum ada kalibrasi tersimpan, pakai default 0-1023"));
  }

  pinMode(BTN_CALIB, INPUT_PULLUP);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);
  DEBUG_PRINTLN(F("[OK] Tombol D2(CALIBRATE) D4(START) D12(STOP) diinisialisasi"));

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  stopMotor();
  DEBUG_PRINTLN(F("[OK] Motor IN1/IN2/IN3/IN4 diinisialisasi, motor OFF"));

  strip.begin();
  strip.setBrightness(LED_BRIGHTNESS);
  strip.clear();
  strip.show();
  DEBUG_PRINTLN(F("[OK] LED WS2812 (D13) diinisialisasi"));

  jepitServo.attach(SERVO_JEPIT_PIN);
  angkatServo.attach(SERVO_ANGKAT_PIN);
  angkatServo.write(ANGKAT_NAIK_ANGLE);
  jepitServo.write(JEPIT_BUKA_ANGLE);
  DEBUG_PRINTLN(F("[OK] Servo jepit(D9) & angkat(D10) langsung di posisi NAIK+BUKA"));

  DEBUG_PRINTLN(F("========================================"));
  DEBUG_PRINTLN(F("SETUP SELESAI - IDLE, tombol bebas ditekan kapan aja:"));
  DEBUG_PRINTLN(F("  D2 CALIBRATE | D4 START | D12 STOP"));
  DEBUG_PRINTLN(F("========================================"));

  currentState = ST_IDLE;
}

// ================== LOOP (STATE MACHINE BEBAS) ==================
void loop() {
  PressType calibPress = checkCalibButton();
  startPressed = checkButton(BTN_START, lastReadStart, lastStartChangeAt);
  stopPressed  = checkButton(BTN_STOP, lastReadStop, lastStopChangeAt);

  // CALIBRATE: tahan lama = kalibrasi beneran, tekan sebentar = plan naik
  if (calibPress == PRESS_LONG) {
    stopMotor();
    DEBUG_PRINTLN(F("[BTN] CALIBRATE ditahan lama -> mulai kalibrasi"));
    currentState = ST_CALIBRATING;
    doCalibration();              // blocking, ada log di dalam
    DEBUG_PRINTLN(F("[STATE] -> IDLE (tekan START buat jalan)"));
    currentState = ST_IDLE;
    return;
  }
  if (calibPress == PRESS_SHORT) {
    if (currentState == ST_IDLE) {
      planNum = (planNum >= PLAN_COUNT) ? 1 : (planNum + 1);
      DEBUG_PRINT(F("[BTN] CALIBRATE ditekan sebentar -> plan naik jadi: "));
      DEBUG_PRINTLN(planNum);
    } else {
      DEBUG_PRINTLN(F("[BTN] CALIBRATE ditekan sebentar diabaikan (bukan lagi IDLE)"));
    }
  }

  if (startPressed && currentState != ST_RUNNING) {
    maju(500);
    DEBUG_PRINTLN(F("[BTN] START ditekan -> RUNNING"));
    DEBUG_PRINT(F("[BTN] Pakai plan: "));
    DEBUG_PRINTLN(planNum);
    lastError = 0;
    currentState = ST_RUNNING;
  }

  // STOP: kalau lagi RUNNING (atau state lain yang jalan) = berhenti normal,
  // kalau lagi ST_IDLE (belum start) = plan turun
  if (stopPressed) {
    if (currentState == ST_IDLE) {
      planNum = (planNum <= 1) ? PLAN_COUNT : (planNum - 1);
      DEBUG_PRINT(F("[BTN] STOP ditekan sebentar (idle) -> plan turun jadi: "));
      DEBUG_PRINTLN(planNum);
    } else if (currentState != ST_STOPPED) {
      stopMotor();
      solidRed();
      DEBUG_PRINTLN(F("[BTN] STOP ditekan -> motor berhenti"));
      currentState = ST_STOPPED;
    }
  }

  switch (currentState) {
    case ST_IDLE:
      rainbowIdle();
      break;
    case ST_RUNNING:
      runLineFollowing();
      break;
    case ST_STOPPED:
      rainbowIdle();
      break;
    case ST_FINISH:
      indicateFinish();
      if (millis() - finishStateEnteredAt > FINISH_DURATION_MS) {
        DEBUG_PRINTLN(F("[STATE] FINISH selesai -> balik ke IDLE, siap run berikutnya"));
        intersectionCount = 0;
        intersectionActive = false;
        currentState = ST_IDLE;
      }
      break;
    case ST_CALIBRATING:
      // ga akan sampai sini karena doCalibration() blocking di atas
      break;
  }
}

// ================== LINE FOLLOWING (dipanggil tiap loop saat RUNNING) ==================
void runLineFollowing() {
  readAndNormalize();

  // ---- cek robot diangkat: SEMUA sensor gelap barengan (ga ada permukaan buat mantul) ----
  bool allBlack = true;
  for (uint8_t i = 0; i < 8; i++) {
    if (normalized[i] <= SENSOR_ON_THRESHOLD) { allBlack = false; break; }
  }
  if (allBlack) {
    stopMotor();
    indicateLifted();
    if (!robotLifted) {
      robotLifted = true;
      DEBUG_PRINTLN(F("[ANGKAT] Semua sensor gelap barengan - robot kemungkinan diangkat, motor dimatikan"));
    }
    return; // skip semua logic line-following/simpang selama diangkat
  } else if (robotLifted) {
    robotLifted = false;
    DEBUG_PRINTLN(F("[ANGKAT] Robot balik ke track, lanjut jalan normal"));
  }

  long weightedSum = 0;
  long sumVal = 0;
  int maxVal = 0;
  for (uint8_t i = 0; i < 8; i++) {
    weightedSum += (long)normalized[i] * (i * 1000L);
    sumVal += normalized[i];
    if (normalized[i] > maxVal) maxVal = normalized[i];
  }

  bool lineDetected = (maxVal >= LINE_LOST_THRESHOLD);

  if (!lineDetected) {
    indicateLineLost();
    if (lastErrorSign >= 0) {
      setMotor(PIVOT_SPEED, -PIVOT_SPEED);   // pivot kanan
    } else {
      setMotor(-PIVOT_SPEED, PIVOT_SPEED);   // pivot kiri
    }
    if (millis() - lastDebugPrint > DEBUG_PRINT_INTERVAL) {
      lastDebugPrint = millis();
      DEBUG_PRINT(F("[RUNNING] LINE HILANG (maxVal:"));
      DEBUG_PRINT(maxVal);
      DEBUG_PRINT(F(") - pivot ke "));
      DEBUG_PRINTLN(lastErrorSign >= 0 ? F("KANAN") : F("KIRI"));
    }
    return;
  }

  int position = weightedSum / sumVal;   // 0 (paling kiri) .. 7000 (paling kanan)
  int error = position - 3500;           // 3500 = titik tengah 8 sensor

  lastErrorSign = (error >= 0) ? 1 : -1;

  int derivative = error - lastError;
  lastError = error;

  int correction = (int)(KP * error + KD * derivative);

  int leftSpeed  = constrain(BASE_SPEED + correction, 0, MAX_SPEED);
  int rightSpeed = constrain(BASE_SPEED - correction, 0, MAX_SPEED);

  setMotor(leftSpeed, rightSpeed);

  // ---- deteksi persimpangan: S0 & S7 hitam, TAPI masih ada sensor tengah yang putih ----
  // (syarat "ada putih di tengah" ini yang bedain simpang asli vs kondisi diangkat/full black)
  bool anyMiddleWhite = false;  
  for (uint8_t i = 0; i < 8; i++) {
    if (i == 1 || i == 6) continue;
    if (normalized[i] <= SENSOR_ON_THRESHOLD) { anyMiddleWhite = true; break; }
  }
  
  bool onIntersectionNow = (normalized[1] > SENSOR_ON_THRESHOLD) &&
                           (normalized[6] > SENSOR_ON_THRESHOLD) &&
                            anyMiddleWhite;
                            
  if (onIntersectionNow && !intersectionActive) {
    intersectionActive = true;
    intersectionCount++;
    intersectionFlashUntil = millis() + INTERSECTION_FLASH_MS;
    DEBUG_PRINT(F("[SIMPANG] Persimpangan terdeteksi! Total: "));
    DEBUG_PRINTLN(intersectionCount);

    // ---- cek mapping.h: ada script buat simpang ini? ----
    bool actionDone = false;
    executeMapStep(intersectionCount, actionDone);
    if (actionDone) {
      return; // simpang ini udah dihandle manual di mapping.h, skip sisa loop iterasi ini
    }
    // kalau actionDone masih false (case kosong/belum di-map di mapping.h)
    // -> lanjut ke behavior default LURUS di bawah, ga ada yang berubah sama sekali
  } else if (!onIntersectionNow && intersectionActive) {
    intersectionActive = false; // robot udah lewat, siap deteksi simpang berikutnya
  }

  if (millis() < intersectionFlashUntil) {
    indicateIntersection();
  } else {
    indicateSensorBar();
  }

  if (millis() - lastDebugPrint > DEBUG_PRINT_INTERVAL) {
    lastDebugPrint = millis();
    DEBUG_PRINT(F("[RUNNING] sensor:"));
    for (uint8_t i = 0; i < 8; i++) {
      DEBUG_PRINT(normalized[i]);
      DEBUG_PRINT(i < 7 ? "," : " ");
    }
    DEBUG_PRINT(F("pos:")); DEBUG_PRINT(position);
    DEBUG_PRINT(F(" err:")); DEBUG_PRINT(error);
    DEBUG_PRINT(F(" L:")); DEBUG_PRINT(leftSpeed);
    DEBUG_PRINT(F(" R:")); DEBUG_PRINT(rightSpeed);
    DEBUG_PRINT(F(" simpang:")); DEBUG_PRINTLN(intersectionCount);
  }
}

// ================== SENSOR ==================
void readAndNormalize() {
  for (uint8_t i = 0; i < 8; i++) {
    int raw = analogRead(sensorPin[i]);
    int val = map(raw, calibMin[i], calibMax[i], 0, 1000);
    normalized[i] = constrain(val, 0, 1000);
  }
}

void doCalibration() {
  for (uint8_t i = 0; i < 8; i++) {
    calibMin[i] = 1023;
    calibMax[i] = 0;
  }

  DEBUG_PRINTLN(F("----------------------------------------"));
  DEBUG_PRINT(F("[KALIBRASI] DIMULAI - geser robot manual ngelewatin garis selama "));
  DEBUG_PRINT(CALIB_DURATION_MS / 1000);
  DEBUG_PRINTLN(F(" detik!"));

  unsigned long t0 = millis();
  unsigned long lastTick = 0;
  while (millis() - t0 < CALIB_DURATION_MS) {
    for (uint8_t i = 0; i < 8; i++) {
      int raw = analogRead(sensorPin[i]);
      if (raw < calibMin[i]) calibMin[i] = raw;
      if (raw > calibMax[i]) calibMax[i] = raw;
    }
    calibratingBlink();

    if (millis() - lastTick > 500) {
      lastTick = millis();    
      DEBUG_PRINT(F("[KALIBRASI] sisa waktu: "));
      DEBUG_PRINT((CALIB_DURATION_MS - (millis() - t0)) / 1000.0, 1);
      DEBUG_PRINTLN(F(" detik"));
    }
    if (!digitalRead(BTN_STOP)) break;
  }

  for (uint8_t i = 0; i < 8; i++) {
    if (calibMax[i] - calibMin[i] < 20) calibMax[i] = calibMin[i] + 20;
  }

  DEBUG_PRINTLN(F("[KALIBRASI] SELESAI. Hasil per sensor (min - max):"));
  for (uint8_t i = 0; i < 8; i++) {
    DEBUG_PRINT(F("  S")); DEBUG_PRINT(i);
    DEBUG_PRINT(F(": ")); DEBUG_PRINT(calibMin[i]);
    DEBUG_PRINT(F(" - ")); DEBUG_PRINTLN(calibMax[i]);
  }
  DEBUG_PRINTLN(F("----------------------------------------"));

  saveCalibrationToEEPROM();
  DEBUG_PRINTLN(F("[KALIBRASI] Tersimpan ke EEPROM - ga perlu diulang tiap reset"));

  strip.clear();
  strip.show();
  flashGreen();
}

// simpan/baca calibMin & calibMax dari EEPROM, ditandai 1 byte magic biar
// EEPROM kosong (chip baru) ga kebaca sebagai data valid
void saveCalibrationToEEPROM() {
  EEPROM.put(EEPROM_ADDR_DATA, calibMin);
  EEPROM.put(EEPROM_ADDR_DATA + sizeof(calibMin), calibMax);
  EEPROM.update(EEPROM_ADDR_MAGIC, EEPROM_MAGIC_VALUE);
}

bool loadCalibrationFromEEPROM() {
  if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC_VALUE) return false;
  EEPROM.get(EEPROM_ADDR_DATA, calibMin);
  EEPROM.get(EEPROM_ADDR_DATA + sizeof(calibMin), calibMax);
  return true;
}

// ================== TOMBOL ==================
// deteksi falling edge (HIGH -> LOW), debounce NON-BLOCKING berbasis waktu
// (bukan delay()+recheck - itu bikin tap CEPET ke-drop kalau tombolnya udah
// kelepas sebelum recheck-nya, makanya kadang "ga kehitung")
bool checkButton(uint8_t pin, bool &lastReading, unsigned long &lastChangeAt) {
  bool reading = digitalRead(pin);
  if (!reading) solidRed();
  bool triggered = false;
  if (reading != lastReading && millis() - lastChangeAt > 30) {
    lastChangeAt = millis();
    if (lastReading == HIGH && reading == LOW) triggered = true;
    lastReading = reading;
  }
  return triggered;
}

// khusus BTN_CALIB: bedain short-press (ganti plan) vs long-press (mulai
// kalibrasi beneran). Long-press LANGSUNG trigger begitu ambang kelewat
// (ga perlu nunggu lepas tombol dulu).
PressType checkCalibButton() {
  bool reading = digitalRead(BTN_CALIB);
  if (!reading) solidRed(); // efek flash sama kayak checkButton()
  PressType result = PRESS_NONE;

  if (reading != lastReadCalib && millis() - lastCalibChangeAt > 30) {
    lastCalibChangeAt = millis();
    if (lastReadCalib == HIGH && reading == LOW) {
      // baru ditekan (udah lolos debounce, LANGSUNG dianggap valid - ga
      // perlu nunggu/recheck lagi, itu yang bikin tap cepet ke-drop dulu)
      calibPressStart = millis();
      calibLongFired = false;
    } else if (lastReadCalib == LOW && reading == HIGH) {
      // baru dilepas
      if (!calibLongFired) result = PRESS_SHORT;
    }
    lastReadCalib = reading;
  } else if (lastReadCalib == LOW && reading == LOW && !calibLongFired) {
    // masih ditahan, cek udah lewat ambang long-press belum
    if (millis() - calibPressStart >= CALIB_LONG_PRESS_MS) {
      calibLongFired = true;
      result = PRESS_LONG;
    }
  }

  return result;
}

// ================== MOTOR ==================
// speed: -255..255, negatif = mundur
void setMotor(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed >= 0) {
    analogWrite(IN3, leftSpeed);
    analogWrite(IN4, 0);
  } else {
    analogWrite(IN3, 0);
    analogWrite(IN4, -leftSpeed);
  }

  if (rightSpeed >= 0) {
    analogWrite(IN1, rightSpeed);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, -rightSpeed);
  }
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ================== INDIKATOR WS2812 ==================
// tiap LED mewakili 1 sensor fisik: nyala merah kalau sensor itu kena garis hitam
void indicateSensorBar() {
  for (uint8_t i = 0; i < 8; i++) {
    if (normalized[i] > SENSOR_ON_THRESHOLD) {
      strip.setPixelColor(i, strip.Color(255, 0, 0)); // merah = kena garis
    } else {
      strip.setPixelColor(i, strip.Color(0, 0, 60));  // biru redup = area kosong
    }
  }
  strip.show();
}

// putih blink cepat, nandain "robot lagi diangkat, motor mati"
void indicateLifted() {
  static bool on = false;
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 100) {
    on = !on;
    lastBlink = millis();
    uint32_t c = on ? strip.Color(80, 80, 80) : strip.Color(0, 0, 0);
    for (uint8_t i = 0; i < 8; i++) strip.setPixelColor(i, c);
    strip.show();
  }
}

// kuning nyala penuh sesaat, nandain "persimpangan baru aja kedeteksi"
void indicateIntersection() {
  for (uint8_t i = 0; i < 8; i++) {
    strip.setPixelColor(i, strip.Color(255, 150, 0));
  }
  strip.show();
}

void indicateLineLost() {
  static bool on = false;
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 100) {
    on = !on;
    lastBlink = millis();
    uint32_t c = on ? strip.Color(255, 0, 0) : strip.Color(0, 0, 0);
    for (uint8_t i = 0; i < 8; i++) strip.setPixelColor(i, c);
    strip.show();
  }
}

void calibratingBlink() {
  static unsigned long lastUpdate = 0;
  static int8_t pos = 0;
  static int8_t direction = 1;  // 1 = kanan, -1 = kiri

  if (millis() - lastUpdate > 120) {
    lastUpdate = millis();
    strip.clear();
    strip.setPixelColor(pos, strip.Color(255, 255, 255));
    strip.show();
    pos += direction;
    // Balik arah saat mencapai ujung
    if (pos >= 7) {
      pos = 7;
      direction = -1;
    } 
    else if (pos <= 0) {
      pos = 0;
      direction = 1;
    }
  }
}

// rainbow tetep kalem jalan di semua LED; 1 LED doang (index == planNum,
// 1-7) yang blink putih di atasnya, biar plan yang lagi kepilih keliatan
// tanpa ganggu animasi rainbow-nya. Dipakai di ST_IDLE MAUPUN ST_STOPPED
// (sama-sama manggil fungsi ini), jadi otomatis konsisten di 2 state itu.
void rainbowIdle() {
  static unsigned long lastUpdate = 0;
  static uint16_t hue = 0;
  static unsigned long lastBlink = 0;
  static bool blinkOn = false;
  const unsigned long BLINK_MS = 150;
  const uint8_t RAINBOW_VALUE = 90; // rainbow SENGAJA redup ("kalem") biar blink pixel kontras jelas

  if (millis() - lastBlink > BLINK_MS) {
    lastBlink = millis();
    blinkOn = !blinkOn;
  }

  if (millis() - lastUpdate > 20) {
    lastUpdate = millis();
    for (uint8_t i = 0; i < 8; i++) {
      uint32_t c = strip.gamma32(strip.ColorHSV(hue + i * 8192, 255, RAINBOW_VALUE));
      strip.setPixelColor(i, c);
    }
    // index 1-7 = planNum langsung; plan 8 kebagian index 0 (biar semua 8 LED kepakai)
//    uint8_t ledIndex = (planNum == 9) ? 0 : planNum - 1;
  uint8_t ledIndex = (planNum == 9) ? 0 : planNum - 1;
    if (blinkOn && ledIndex < 8) {
      strip.setPixelColor(ledIndex, strip.Color(255, 255, 255)); // full white, kontras sama rainbow yang diredupin
//      DEBUG_PRINT("idx led:"); DEBUG_PRINTLN(ledIndex);
    }
    strip.show();
    hue += 400;
  }
}

// merah nyala terus, nandain "robot lagi STOP" - dipanggil terus tiap loop
// selama currentState == ST_STOPPED, jadi ga bakal balik sendiri ke rainbow
void solidRed() {
  for (uint8_t i = 0; i < 8; i++) {
    strip.setPixelColor(i, strip.Color(255, 0, 0));
  }
  strip.setBrightness(70);
  strip.show();
  strip.setBrightness(LED_BRIGHTNESS);
}

// blink hijau, nandain "rute mapping selesai (FINISH)" - dipanggil terus
// tiap loop selama currentState == ST_FINISH, sebelum auto balik ke IDLE
void indicateFinish() {
  static bool on = false;
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 120) {
    on = !on;
    lastBlink = millis();
    uint32_t c = on ? strip.Color(0, 255, 0) : strip.Color(0, 0, 0);
    for (uint8_t i = 0; i < 8; i++) strip.setPixelColor(i, c);
    strip.show();
  }
}

// kedip hijau sekali, nandain "kalibrasi baru aja selesai"
void flashGreen() {
  for (uint8_t r = 0; r < 2; r++) {
    for (uint8_t i = 0; i < 8; i++) strip.setPixelColor(i, strip.Color(0, 255, 0));
    strip.show();
    delay(150);
    strip.clear();
    strip.show();
    delay(150);
  }
}

// mapping.h di-include di SINI (paling bawah) - biar semua yang dia butuhin
// (strip, setMotor, stopMotor, BASE_SPEED, PIVOT_SPEED, BTN_STOP, currentState,
// ST_FINISH, finishStateEnteredAt, lastError, lastErrorSign) udah pasti
// kekenal duluan sama compiler.
#include "mapping.h"
