#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h" // ESP32 specific

#include "config.h"

/* Function to initialise motors */
void initMotors();

/* Function to home-in the steppers */
void homingProcedure();

/* Function to action the trigger */
void fireProcedure();

/* Function to signal back motor arrival */
void motorCallback(AccelStepper* stepper);