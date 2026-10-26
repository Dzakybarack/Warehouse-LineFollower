#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

const uint8_t SERVO_JEPIT_PIN  = 9;
const uint8_t SERVO_ANGKAT_PIN = 10;

const int JEPIT_BUKA_ANGLE   = 0;
const int JEPIT_TUTUP_ANGLE  = 65;
const int ANGKAT_NAIK_ANGLE  = 65;
const int ANGKAT_TURUN_ANGLE = 180;

const unsigned long SERVO_STEP_DELAY_MS = 8; // delay per 1 derajat (gerak halus)

#endif
