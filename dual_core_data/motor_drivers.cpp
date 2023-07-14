#include "freertos/projdefs.h"
#include "motor_drivers.h"

AccelStepper stepper_az(MOTOR_INTERFACE, STEP_PIN_AZ, DIR_PIN_AZ);
AccelStepper stepper_el(MOTOR_INTERFACE, STEP_PIN_EL, DIR_PIN_EL);

Servo servo_az;
Servo servo_el;
Servo servo_trg;

extern Adafruit_9DOF dof;
extern Adafruit_LSM303_Accel_Unified accel;
extern Adafruit_LSM303_Mag_Unified mag;

extern double el_angle;
extern double az_angle;

extern TaskHandle_t TaskFire;
extern TaskHandle_t TaskMotorCallback;

/* Function to initialise motors */
void initMotors() {
  /* System-wide steppers */
  stepper_az.setMaxSpeed(100000);
  stepper_az.setSpeed(100000);
  stepper_az.setAcceleration(10000);
  stepper_az.setEnablePin(ENABLE_PIN_AZ);
  stepper_az.disableOutputs();

  stepper_el.setMaxSpeed(10000);
  stepper_el.setSpeed(5000);
  stepper_el.setAcceleration(1000);
  stepper_el.setEnablePin(ENABLE_PIN_EL);
  stepper_el.disableOutputs();

  /* Visor servos */
  servo_az.attach(SERVO_PIN_AZ);
  servo_el.attach(SERVO_PIN_EL);
  servo_az.write(90 - DEFAULT_SERVO_OFFSET);
  servo_el.write(90);

  /* Trigger servo */
  servo_trg.attach(SERVO_PIN_TRG);
}

/* Function to home-in the steppers */
void homingProcedure() {

  /* Elevation homing based on pitch data from the 9-DOF sensor */

  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t orientation;

  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  int angle;
  while (1) {
    accel.getEvent(&accel_event);
    mag.getEvent(&mag_event);

    if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)) {
      angle = orientation.pitch;
      //if (angle >= SFZN_SYS_LWR_BND_EL && angle <= SFZN_SYS_HGH_BND_EL) {
      stepper_el.moveTo(angle * MICROSTEPS / 1.8);
      while (stepper_el.distanceToGo() != 0) {
        stepper_el.run();
      }
      break;
    }
  }
  stepper_el.setCurrentPosition(0);

  /* Azimuth homing based on end-stop switch */
  long max_distance = -999999;
  stepper_az.moveTo(max_distance * MICROSTEPS * TEETH_GEAR_RATIO);
  while (stepper_az.distanceToGo() != 0) {
    stepper_az.run();
    // If stepper arrived to end switch, it's at almost 180 degrees out of phase
    //
    if (digitalRead(SW_PIN_HOME) == LOW) {
      break;
    }
    vTaskDelay(1);
  }
  while (digitalRead(SW_PIN_HOME) == LOW) {
    stepper_az.setCurrentPosition(-1);
    stepper_az.move(1);
    stepper_az.runSpeed();
  }
  stepper_az.move(90 * MICROSTEPS / 1.8 * TEETH_GEAR_RATIO);
  while (stepper_az.distanceToGo() != 0) {
    stepper_az.run();
  }
  stepper_az.setCurrentPosition(90 * MICROSTEPS / 1.8 * TEETH_GEAR_RATIO);
  Serial.println("Homing complete");
}

/* Function to action the trigger */
void fireProcedure(void* pvParameters) {
  servo_trg.write(TRIGGER_FIRE_POS);
  //TimerHandle_t myTimer;
  //myTimer = xTimerCreate("Fire Procedure Timer", pdMS_TO_TICKS(1000), pdTRUE, 0, NULL);
  //while (1) {
  //if (xTimerIsTimerActive(myTimer) == pdFALSE) {
  //   break;
  //}
  // vTaskDelay(pdMS_TO_TICKS(100));
  //}
  delay(600);
  servo_trg.write(TRIGGER_REST_POS);
  Serial.println("Fired");
  vTaskDelete(NULL);
}

/* Timer motor callback function */
void timerMotorCallback(TimerHandle_t xTimer) {
  vTaskDelay(pdMS_TO_TICKS(10));
}

/* Function to signal back motor arrival */
void motorCallback(void* stepper_void) {
  TimerHandle_t myTimer;
  AccelStepper* stepper = (AccelStepper*)stepper_void;
  if (stepper != NULL) {
    while (stepper->distanceToGo() != 0) {
      vTaskDelay(pdMS_TO_TICKS(4));
    }
  }
  Serial.println("Stp arr");
  vTaskDelete(NULL);
}