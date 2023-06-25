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
const int move_step_a_ctrl = (int)'A';
const int move_step_e_ctrl = (int)'E';
const int send_orientation_ctrl = (int)'o';
const int move_servo_a_ctrl = (int)'a';
const int move_servo_e_ctrl = (int)'e';
const int scram_ctrl = (int)'s';
const int restore_ctrl = (int)'r';
const int laser_ctrl = (int)'l';

/* Median filter data type */
typedef struct median_orientation_t {
  float roll = 0,
        pitch = 0,
        heading = 0;
  uint8_t overSampleRate = 31;
  uint8_t currentSampleCounter = 0;
};

/* Manage function parameter data type */
typedef struct manage_function_t {
  byte ctrl;
  char rx_str[7];
};

/* Function to initialise sensors */
void initSensors();

/* First-time setup */
void gpioInit();