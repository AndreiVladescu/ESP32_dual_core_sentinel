#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

#include "config.h"

/* Control */
const int fire_ctrl = 0x10;
const int move_step_a_ctrl = 0x20;
const int move_step_e_ctrl = 0x30;
const int send_orientation_ctrl = 0x40;
const int move_servo_a_ctrl = 0x50;
const int move_servo_e_ctrl = 0x60;
const int scram_ctrl = 0x53;
const int restore_ctrl = 0x52;

/* Median filter data type */
typedef struct median_orientation_t {
  float roll = 0,
        pitch = 0,
        heading = 0;
  uint8_t overSampleRate = 31;
  uint8_t currentSampleCounter = 0;
};

/* Function to initialise sensors */
void initSensors();

/* First-time setup */
void gpioInit();