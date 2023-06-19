#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

#include "config.h"

/* Function to initialise motors */
void initMotors();

/* Function to home-in the steppers */
void homingProcedure();

/* Function to action the trigger */
void fireProcedure();