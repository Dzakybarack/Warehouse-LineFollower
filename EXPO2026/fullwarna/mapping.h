#ifndef MAPPING_H
#define MAPPING_H
// Rute per plan/simpang: edit executeMapStep() di bawah. Primitive & tuning
// juga di file ini (include di paling bawah lineFollowerPD.ino).

enum TurnAction { ACT_LURUS, ACT_KIRI, ACT_KANAN, ACT_PUTAR, ACT_BERHENTI };

// ---- Tuning maneuver (kecepatan & durasi) - WAJIB dikalibrasi manual di
//      lapangan, beda arena/beban robot = beda waktu yang pas ----
const int           TURN_SPEED     = 180; // kecepatan pivot belok/putar (0-255)
const unsigned long TURN_KIRI_MS   = 350; // durasi pivot kiri  (~90 derajat)
const unsigned long TURN_KANAN_MS  = 300; // durasi pivot kanan (~90 derajat)
const unsigned long TURN_PUTAR_MS  = 500; // durasi u-turn      (~180 derajat)
const unsigned long MAJU_MS        = 300; // durasi default maju()
const unsigned long MUNDUR_MS      = 200; // durasi default mundur()
const unsigned long STOP_SESAAT_MS = 50; // durasi default stopSesaat()

// ================== MAPPING JALUR: PRIMITIVE (dipanggil dari mapping.h) ==================
// Semua primitive di bawah "blocking" (robot ga ngikutin PD selama primitive
// ini jalan) - sama gaya kayak doCalibration() yang udah ada. LED animasi
// tetap smooth karena update pakai millis(), bukan delay() polos.

// warna LED per jenis manuver, biar gampang dibedain dari mode lain
uint32_t decisionColor(TurnAction action) {
  switch (action) {
    case ACT_KIRI:     return strip.Color(0, 255, 255);   // cyan
    case ACT_KANAN:    return strip.Color(255, 0, 150);   // pink/magenta
    case ACT_PUTAR:    return strip.Color(150, 0, 255);   // ungu
    case ACT_BERHENTI: return strip.Color(255, 0, 0);     // merah (dipakai stopSesaat)
    default:           return strip.Color(255, 255, 255); // putih (ga kepake, jaga2)
  }
}

// Animasi dot LED nunjukin manuver yang lagi jalan. TIDAK PAKAI delay() -
// tiap dipanggil cuma update posisi kalau STEP_MS udah lewat (cek millis()),
// aman dipanggil berkali-kali di dalam while nunggu durasi tanpa nge-freeze:
//   ACT_KIRI     -> dot lari ke kiri (index 7 -> 0, ulang)
//   ACT_KANAN    -> dot lari ke kanan (index 0 -> 7, ulang)
//   ACT_PUTAR    -> 2 dot nyebar dari tengah ke pinggir (3,4 -> 0,7)
//   ACT_BERHENTI -> 2 dot nyempit dari pinggir ke tengah (0,7 -> 3,4) - dipakai stopSesaat()
void updateDecisionLED(TurnAction action) {
  static unsigned long lastUpdate = 0;
  static uint8_t phase = 0;
  const unsigned long STEP_MS = 40;

  if (millis() - lastUpdate < STEP_MS) return;
  lastUpdate = millis();

  strip.clear();
  uint32_t color = decisionColor(action);

  switch (action) {
    case ACT_KIRI:
      strip.setPixelColor(7 - (phase % 8), color);
      break;
    case ACT_KANAN:
      strip.setPixelColor(phase % 8, color);
      break;
    case ACT_PUTAR: {
      uint8_t step = phase % 4;
      strip.setPixelColor(3 - step, color);
      strip.setPixelColor(4 + step, color);
      break;
    }
    case ACT_BERHENTI: {
      uint8_t step = phase % 4;
      strip.setPixelColor(step, color);
      strip.setPixelColor(7 - step, color);
      break;
    }
    default:
      break;
  }
  strip.show();
  phase++;
}

// blink amber, nandain "robot lagi mundur()"
void indicateMundur() {
  static bool on = false;
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 150) {
    on = !on;
    lastBlink = millis();
    uint32_t c = on ? strip.Color(255, 80, 0) : strip.Color(0, 0, 0);
    for (uint8_t i = 0; i < 8; i++) strip.setPixelColor(i, c);
    strip.show();
  }
}

// pivot belok/putar generik, dipakai turnLeft()/turnRight()/turnAround()
void runTurnManeuver(TurnAction kind, unsigned long durationMs) {
  int leftSpd, rightSpd;
  if (kind == ACT_KIRI)       { leftSpd = -TURN_SPEED; rightSpd =  TURN_SPEED; }
  else if (kind == ACT_KANAN) { leftSpd =  TURN_SPEED; rightSpd = -TURN_SPEED; }
  else                        { leftSpd =  TURN_SPEED; rightSpd = -TURN_SPEED; } // ACT_PUTAR

  setMotor(leftSpd, rightSpd);
  unsigned long t0 = millis();
  while (millis() - t0 < durationMs) {
    updateDecisionLED(kind);
    if (!digitalRead(BTN_STOP)) break; // bisa dibatalin manual kalau darurat
  }
  stopMotor();
}

void turnLeft() {
  DEBUG_PRINTLN(F("[MAPPING] turnLeft()"));
  runTurnManeuver(ACT_KIRI, TURN_KIRI_MS);
}

void turnRight() {
  DEBUG_PRINTLN(F("[MAPPING] turnRight()"));
  runTurnManeuver(ACT_KANAN, TURN_KANAN_MS);
}

void turnAround() {
  DEBUG_PRINTLN(F("[MAPPING] turnAround()"));
  runTurnManeuver(ACT_PUTAR, TURN_PUTAR_MS);
}

void maju(unsigned long ms = MAJU_MS) {
  DEBUG_PRINTLN(F("[MAPPING] maju()"));
  setMotor(BASE_SPEED, BASE_SPEED);
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (!digitalRead(BTN_STOP)) break;
  }
  stopMotor();
}

void mundur(unsigned long ms = MUNDUR_MS) {
  DEBUG_PRINTLN(F("[MAPPING] mundur()"));
  setMotor(-PIVOT_SPEED, -PIVOT_SPEED);
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    indicateMundur();
    if (!digitalRead(BTN_STOP)) break;
  }
  stopMotor();
}

// berhenti SEBENTAR doang - robot TETAP lanjut ke simpang berikutnya
// setelah ini, BEDA sama selesaiRute() yang tamatin seluruh rute
void stopSesaat(unsigned long ms = STOP_SESAAT_MS) {
  DEBUG_PRINTLN(F("[MAPPING] stopSesaat()"));
  stopMotor();
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    updateDecisionLED(ACT_BERHENTI); // dot merah nyempit ke tengah = "lagi pause"
  }
}

void resetLineFollowing() {
  lastError = 0;
  lastErrorSign = 1;
  DEBUG_PRINTLN(F("[MAPPING] resetLineFollowing()"));
}

// ---------------- SERVO (gripper) ----------------
// Gerak servo SELALU halus (1 derajat per SERVO_STEP_DELAY_MS, dari
// servo_config.h), ga pernah write() langsung loncat ke sudut jauh -
// ini yang bikin ga ada lonjakan arus mendadak ke step-down yang
// dipakai bareng Arduino & motor.
void servoSweep(Adafruit_TiCoServo &s, int &posVar, int target) {
  int step = (target >= posVar) ? 1 : -1;
  while (posVar != target) {
    posVar += step;
    s.write(posVar);
    delay(SERVO_STEP_DELAY_MS);
  }
}

void servoNaik() {
  DEBUG_PRINTLN(F("[SERVO] naik"));
  servoSweep(angkatServo, angkatPos, ANGKAT_NAIK_ANGLE);
}

void servoTurun() {
  DEBUG_PRINTLN(F("[SERVO] turun"));
  servoSweep(angkatServo, angkatPos, ANGKAT_TURUN_ANGLE);
}

void servoBukaJepit() {
  DEBUG_PRINTLN(F("[SERVO] buka jepit"));
  servoSweep(jepitServo, jepitPos, JEPIT_BUKA_ANGLE);
}

void servoTutupJepit() {
  DEBUG_PRINTLN(F("[SERVO] tutup jepit"));
  servoSweep(jepitServo, jepitPos, JEPIT_TUTUP_ANGLE);
}

// kombinasi siap pakai - turun, jepit, naik lagi (biar barang ke-angkat)
void ambilKotak() {
  DEBUG_PRINTLN(F("[MAPPING] ambilKotak()"));
  servoTurun();
  servoTutupJepit();
  servoNaik();
}

// kombinasi siap pakai - turun, lepas jepit, naik lagi
void lepasKotak() {
  DEBUG_PRINTLN(F("[MAPPING] lepasKotak()"));
  servoTurun();
  servoBukaJepit();
  servoNaik();
}

// tandain SELURUH rute mapping udah kelar (bukan cuma 1 simpang) - robot
// stop total, LED hijau blink (state FINISH), lalu OTOMATIS balik ke IDLE
// begitu FINISH_DURATION_MS lewat (lihat switch(currentState) di loop()),
// siap buat run berikutnya tinggal pencet START lagi - TIDAK hang selamanya
void selesaiRute() {
  DEBUG_PRINTLN(F("[MAPPING] selesaiRute() - rute selesai, FINISH -> auto balik IDLE"));
  stopMotor();
  finishStateEnteredAt = millis();
  currentState = ST_FINISH;
}

void ambil(){
  servoTurun();
  servoBukaJepit();
  resetLineFollowing();
}

void angkatbarang(){
  servoTutupJepit();
  servoNaik();
  turnAround();
  resetLineFollowing();
}

void taruhbarang(){
  mundur(); 
  lepasKotak(); 
  turnAround(); 
  resetLineFollowing();
}

// ------------------------ EDIT DI SINI ------------------------
// SKENARIO "SODOK": 1 simpang SEBELUM ambil kotak, gripper udah ditaruh
// turun+buka (servoTurun()+servoBukaJepit()), biar udah siap nyodok pas
// robot maju ke simpang pengambilan. Di simpang pengambilannya sendiri,
// TINGGAL tutup+naik (servoTutupJepit()+servoNaik()) - JANGAN panggil
// servoTurun()/servoBukaJepit() lagi di situ, soalnya udah dilakuin di
// simpang sebelumnya.
void executeMapStep(unsigned int simpangKe, bool &actionDone) {
  switch (planNum) {
    case 1: // warna kotak & gudang sejajar
      switch (simpangKe) {
        case 1: turnLeft(); servoTurun(); servoBukaJepit(); resetLineFollowing(); actionDone = true; break;
        case 2: stopSesaat(); servoTutupJepit(); servoNaik(); turnAround(); resetLineFollowing(); actionDone = true; break;
        case 3: actionDone = true; break;
        case 4: stopSesaat(); mundur(); lepasKotak(); turnAround(); actionDone = true; break;
        case 5: turnRight(); actionDone = true; break;
        case 6: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 2: //atas 1
      switch (simpangKe) {
        case 1: turnLeft(); servoTurun(); servoBukaJepit(); resetLineFollowing(); actionDone = true; break;
        case 2: stopSesaat(); servoTutupJepit(); servoNaik(); turnAround(); resetLineFollowing(); actionDone = true; break;
        case 3: turnLeft(); actionDone = true; break;
        case 4: turnRight(); actionDone = true; break;
        case 5: stopSesaat(); lepasKotak(); turnAround(); actionDone = true; break;
        case 6: turnLeft(); actionDone = true; break;
        case 7: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;
    

    case 3: // atas 2
      switch (simpangKe) {
        case 1: turnLeft(); servoTurun(); servoBukaJepit(); resetLineFollowing(); actionDone = true; break;
        case 2: stopSesaat(); servoTutupJepit(); servoNaik(); turnAround(); resetLineFollowing(); actionDone = true; break;
        case 3: turnLeft(); actionDone = true; break;
        case 4: actionDone = true; break;
        case 5: turnRight(); actionDone = true; break;
        case 6: stopSesaat(); lepasKotak(); turnAround(); actionDone = true; break;
        case 7: turnLeft(); actionDone = true; break;
        case 8: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;
   
    case 4: // warna kotak di bawah 1
      switch (simpangKe) {
        case 1: turnLeft(); servoTurun(); servoBukaJepit(); resetLineFollowing(); actionDone = true; break;
        case 2: stopSesaat(); servoTutupJepit(); servoNaik(); turnAround(); resetLineFollowing(); actionDone = true; break;
        case 3: turnRight(); actionDone = true; break;
        case 4: turnLeft(); actionDone = true; break;
        case 5: stopSesaat(); lepasKotak(); turnAround(); actionDone = true; break;
        case 6: turnRight(); actionDone = true; break;
        case 7: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 5: // warna kotak di bawah 2
      switch (simpangKe) {
        case 1: turnLeft(); servoTurun(); servoBukaJepit(); resetLineFollowing(); actionDone = true; break;
        case 2: stopSesaat(); servoTutupJepit(); servoNaik(); turnAround(); resetLineFollowing(); actionDone = true; break;
        case 3: turnRight(); actionDone = true; break;
        case 4: actionDone = true; break;
        case 5: turnLeft(); actionDone = true; break;
        case 6: stopSesaat(); lepasKotak(); turnAround(); actionDone = true; break;
        case 7: turnRight(); actionDone = true; break;
        case 8: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 6: // kejar retry 1 (ga ambil/lepas kotak)
      switch (simpangKe) {
        case 1: actionDone = true; break;
        case 2: maju(); actionDone = true; break;
        case 3: maju(); actionDone = true; break;
        case 4: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 7: // loncat 1 retry (ga ambil/lepas kotak)
      switch (simpangKe) {
        case 1: actionDone = true; break;
        case 2: turnLeft(); actionDone = true; break;
        case 3: actionDone = true; break;
        case 4: actionDone = true; break;
        case 5: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

      case 8: //CP2(BM)
      switch (simpangKe) {
        case 6: turnLeft(); ambil(); actionDone = true; break;
        case 7: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 8: actionDone = true; break;
        case 9: stopSesaat(); taruhbarang(); actionDone = true; break;      //sejajar
        case 10: turnRight(); actionDone = true; break;                     //kanan
        case 11: turnLeft(); ambil(); actionDone = true; break;
        case 12: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 13: actionDone = true; break;
        case 14: stopSesaat(); taruhbarang(); actionDone = true; break;     //sejajar
        case 15: turnRight(); actionDone = true; break;
        case 16: selesaiRute(); 
        default: break;
      }
      break;

      case 9: //CP2(MB)
      switch (simpangKe) {
        case 6: turnLeft(); ambil(); actionDone = true; break;
        case 7: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 8: turnLeft(); actionDone = true; break;
        case 9: turnRight(); actionDone = true; break;
        case 10: stopSesaat(); taruhbarang(); actionDone = true; break;         //atas 1
        case 11: ambil(); actionDone = true; break;
        case 12: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 13: turnRight(); actionDone = true; break;
        case 14: turnLeft(); actionDone = true; break;
        case 15: stopSesaat(); taruhbarang(); actionDone = true; break;         //bawah 1
        case 16: turnRight(); actionDone = true; break;         
        case 17: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

      case 10: //CP2(HM)
      switch (simpangKe) {
        case 6: turnLeft(); ambil(); actionDone = true; break;
        case 7: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 8: turnRight(); actionDone = true; break;
        case 9: turnLeft(); actionDone = true; break;
        case 10: stopSesaat(); taruhbarang(); actionDone = true; break;         //bawah 1
        case 11: turnRight(); actionDone = true; break;
        case 12: actionDone = true; break;
        case 13: turnLeft(); ambil(); actionDone = true; break;
        case 14: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 15: actionDone = true; break;
        case 16: stopSesaat(); taruhbarang(); actionDone = true; break;         //sejajar
        case 17: turnRight(); actionDone = true; break;
        case 18: selesaiRute();
        default: break;
      }
      break;

      case 11: //CP2(MH)
      switch (simpangKe) {
        case 6: turnLeft(); ambil(); actionDone = true; break;
        case 7: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 8: turnLeft(); actionDone = true; break;
        case 9: turnRight(); actionDone = true; break;
        case 10: stopSesaat(); taruhbarang(); actionDone = true; break;       //atas 1
        case 11: ambil(); actionDone = true; break;
        case 12: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 13: turnRight(); actionDone = true; break;
        case 14: actionDone = true; break;
        case 15: turnLeft(); actionDone = true; break;
        case 16: stopSesaat(); taruhbarang(); actionDone = true; break;       //bawah 2
        case 17: turnRight(); actionDone = true; break;
        case 18: actionDone = true; break;
        case 19: actionDone = true; break;
        case 20: selesaiRute();
        default: break;
      }
      break;

      case 12: //CP2(HB)
      switch (simpangKe) {
        case 7: turnLeft(); ambil(); actionDone = true; break;
        case 8: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 9: turnRight(); actionDone = true; break;
        case 10: turnLeft(); actionDone = true; break;
        case 11: stopSesaat(); taruhbarang(); actionDone = true; break;     //bawah 1(Biru)
        case 12: ambil(); actionDone = true; break;
        case 13: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 14: turnRight(); actionDone = true; break;
        case 15: turnLeft(); actionDone = true; break;
        case 16: stopSesaat(); taruhbarang(); actionDone = true; break;  //bawah 1(Hijau)
        case 17: turnRight(); actionDone = true; break;
        case 18: actionDone = true; break;
        case 19: actionDone = true; break;
        case 20: selesaiRute(); 
        default: break;
      }
      break;

      case 13: //CP2(BH)
      switch (simpangKe) {
        case 7: turnLeft(); ambil(); actionDone = true; break;
        case 8: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 9: turnRight(); actionDone = true; break;  
        case 10: actionDone = true; break;                
        case 11: turnLeft(); actionDone = true; break;
        case 12: stopSesaat(); taruhbarang(); actionDone = true; break;  //bawah 2
        case 13: turnRight(); actionDone = true; break;
        case 14: turnLeft(); ambil(); actionDone = true; break;
        case 15: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 16: actionDone = true; break;
        case 17: stopSesaat(); taruhbarang(); actionDone = true; break;  //sejajar
        case 18: turnRight(); actionDone = true; break;
        case 19: actionDone = true; break;
        case 20: selesaiRute();
        default: break;
      }
      break;
      
//    case 8: // loncat 2 retry (lewat 2 simpang, ga ambil/lepas kotak)
//      switch (simpangKe) {
//        case 1: actionDone = true; break;
//        case 2: maju(); actionDone = true; break;
//        case 3: maju(); actionDone = true; break;
//        
//        case 4: turnLeft(); servoTurun(); servoBukaJepit(); resetLineFollowing(); actionDone = true; break;
//        case 5: stopSesaat(); servoTutupJepit(); servoNaik(); turnAround(); resetLineFollowing(); actionDone = true; break;
//        case 6: actionDone = true; break;
//        case 7: stopSesaat(); mundur(); lepasKotak(); turnAround(); actionDone = true; break;
//        case 8: turnRight(); actionDone = true; break;
//       
//        case 9: turnLeft(); servoTurun(); servoBukaJepit(); resetLineFollowing(); actionDone = true; break;
//        case 10: stopSesaat(); servoTutupJepit(); servoNaik(); turnAround(); resetLineFollowing(); actionDone = true; break;
//        case 11: actionDone = true; break;
//        case 12: stopSesaat(); mundur(); lepasKotak(); turnAround(); actionDone = true; break;
//        case 13: turnRight(); actionDone = true; break;
//        
//        case 14: turnLeft(); servoTurun(); servoBukaJepit(); resetLineFollowing(); actionDone = true; break;
//        case 15: stopSesaat(); servoTutupJepit(); servoNaik(); turnAround(); resetLineFollowing(); actionDone = true; break;
//        case 16: actionDone = true; break;
//        case 17: stopSesaat(); mundur(); lepasKotak(); turnAround(); actionDone = true; break;
//        case 18: turnRight(); actionDone = true; break;
//        case 19: maju(); selesaiRute(); actionDone = true; break;
//        default: break;
//      }
//      break;

    case 14: //HBM (sejajar - sejajar - sejajar)
      switch (simpangKe) {
        case 1: turnLeft(); ambil(); actionDone = true; break;
        case 2: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 3: actionDone = true; break;
        case 4: stopSesaat(); taruhbarang(); actionDone = true; break;      //sejajar
        case 5: turnRight(); actionDone = true; break;                      //kanan
        case 6: turnLeft(); ambil(); actionDone = true; break;
        case 7: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 8: actionDone = true; break;
        case 9: stopSesaat(); taruhbarang(); actionDone = true; break;      //sejajar
        case 10: turnRight(); actionDone = true; break;                     //kanan
        case 11: turnLeft(); ambil(); actionDone = true; break;
        case 12: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 13: actionDone = true; break;
        case 14: stopSesaat(); taruhbarang(); actionDone = true; break;     //sejajar
        case 15: turnRight(); actionDone = true; break;
        case 16: selesaiRute(); 
        default: break;
      }
      break;

    case 15: //HMB (sejajar - atas 1 - bawah 1)
      switch (simpangKe) {
        case 1: turnLeft(); ambil(); actionDone = true; break;
        case 2: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 3: actionDone = true; break;
        case 4: stopSesaat(); taruhbarang(); actionDone = true; break;          //sejajar
        case 5: turnRight(); actionDone = true; break;                          //kanan
        case 6: turnLeft(); ambil(); actionDone = true; break;
        case 7: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 8: turnLeft(); actionDone = true; break;
        case 9: turnRight(); actionDone = true; break;
        case 10: stopSesaat(); taruhbarang(); actionDone = true; break;         //atas 1
        case 11: ambil(); actionDone = true; break;
        case 12: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 13: turnRight(); actionDone = true; break;
        case 14: turnLeft(); actionDone = true; break;
        case 15: stopSesaat(); taruhbarang(); actionDone = true; break;         //bawah 1
        case 16: turnRight(); actionDone = true; break;         
        case 17: maju(); selesaiRute(); actionDone = true; break;
        default: break;
      }
      break;

    case 16: //BHM   (atas 1 - bawah 1 - sejajar)
      switch (simpangKe) {
        case 1: turnLeft(); ambil(); actionDone = true; break;
        case 2: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 3: turnLeft(); actionDone = true; break;
        case 4: turnRight(); actionDone = true; break;
        case 5: stopSesaat(); taruhbarang(); actionDone = true; break;          //atas 1
        case 6: ambil(); actionDone = true; break;
        case 7: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 8: turnRight(); actionDone = true; break;
        case 9: turnLeft(); actionDone = true; break;
        case 10: stopSesaat(); taruhbarang(); actionDone = true; break;         //bawah 1
        case 11: turnRight(); actionDone = true; break;
        case 12: actionDone = true; break;
        case 13: turnLeft(); ambil(); actionDone = true; break;
        case 14: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 15: actionDone = true; break;
        case 16: stopSesaat(); taruhbarang(); actionDone = true; break;         //sejajar
        case 17: turnRight(); actionDone = true; break;
        case 18: selesaiRute(); 
        default: break;
      }
      break;    

    case 17: // BMH   (atas 1 - atas 1 - bawah 2)
      switch (simpangKe) {
        case 1: turnLeft(); ambil(); actionDone = true; break;
        case 2: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 3: turnLeft(); actionDone = true; break;
        case 4: turnRight(); actionDone = true; break;
        case 5: stopSesaat(); taruhbarang(); actionDone = true; break;         //atas 1
        case 6: ambil(); actionDone = true; break;
        case 7: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 8: turnLeft(); actionDone = true; break;
        case 9: turnRight(); actionDone = true; break;
        case 10: stopSesaat(); taruhbarang(); actionDone = true; break;       //atas 1
        case 11: ambil(); actionDone = true; break;
        case 12: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 13: turnRight(); actionDone = true; break;
        case 14: actionDone = true; break;
        case 15: turnLeft(); actionDone = true; break;
        case 16: stopSesaat(); taruhbarang(); actionDone = true; break;       //bawah 2
        case 17: turnRight(); actionDone = true; break;
        case 18: actionDone = true; break;
        case 19: actionDone = true; break;
        case 20: selesaiRute();
        default: break;
      }
      break;

    case 18: // MHB   (atas 2 - bawah 1 - bawah 1)
      switch (simpangKe) {
        case 1: turnLeft(); ambil(); actionDone = true; break;
        case 2: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 3: turnLeft(); actionDone = true; break;
        case 4: actionDone = true; break;
        case 5: turnRight(); actionDone = true; break;
        case 6: stopSesaat(); taruhbarang(); actionDone = true; break;      //atas 2
        case 7: ambil(); actionDone = true; break;
        case 8: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 9: turnRight(); actionDone = true; break;
        case 10: turnLeft(); actionDone = true; break;
        case 11: stopSesaat(); taruhbarang(); actionDone = true; break;     //bawah 1
        case 12: ambil(); actionDone = true; break;
        case 13: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 14: turnRight(); actionDone = true; break;
        case 15: turnLeft(); actionDone = true; break;
        case 16: stopSesaat(); taruhbarang(); actionDone = true; break;  //bawah 1
        case 17: turnRight(); actionDone = true; break;
        case 18: actionDone = true; break;
        case 19: actionDone = true; break;
        case 20: selesaiRute(); 
        default: break;
      }
      break;

    case 19: // MBH  (atas 2 - bawah 1 - sejajar)
      switch (simpangKe) {
        case 1: turnLeft(); ambil(); actionDone = true; break;
        case 2: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 3: turnLeft(); actionDone = true; break;
        case 4: actionDone = true; break;
        case 5: turnRight(); actionDone = true; break;
        case 6: stopSesaat(); taruhbarang(); actionDone = true; break;  //atas 2
        case 7: ambil(); actionDone = true; break;
        case 8: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 9: turnRight(); actionDone = true; break;  
        case 10: actionDone = true; break;                
        case 11: turnLeft(); actionDone = true; break;
        case 12: stopSesaat(); taruhbarang(); actionDone = true; break;  //bawah 2
        case 13: turnRight(); actionDone = true; break;
        case 14: turnLeft(); ambil(); actionDone = true; break;
        case 15: stopSesaat(); angkatbarang(); actionDone = true; break;
        case 16: actionDone = true; break;
        case 17: stopSesaat(); taruhbarang(); actionDone = true; break;  //sejajar
        case 18: turnRight(); actionDone = true; break;
        case 19: actionDone = true; break;
        case 20: selesaiRute();
        default: break;
      }
      break;
      
    default:
      break; // planNum di luar 1-8 -> actionDone tetap false, LURUS normal
  }
}
// ----------------------------------------------------------------

//    case 8: // loncat 2 retry (lewat 2 simpang, ga ambil/lepas kotak)
//      switch (simpangKe) {
//        case 1: actionDone = true; break;
//        case 2: actionDone = true; break;
//        case 3: maju(); selesaiRute(); actionDone = true; break;
//        default: break;
//      }
//      break;
#endif
