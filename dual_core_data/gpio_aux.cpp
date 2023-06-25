#include "gpio_aux.h"

extern Adafruit_9DOF dof;
extern Adafruit_LSM303_Accel_Unified accel;
extern Adafruit_LSM303_Mag_Unified mag;

/* Function to initialise sensors */
void initSensors() {
  if (!accel.begin()) {
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1)
      ;
  }
  if (!mag.begin()) {
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1)
      ;
  }
}

/* First-time setup */
void gpioInit() {
  pinMode(SW_PIN_HOME, INPUT_PULLUP);
  pinMode(SW_LASER, OUTPUT);
  //pinMode(SERVO_PIN_AZ,OUTPUT);
  //pinMode(SERVO_PIN_EL,OUTPUT);
}