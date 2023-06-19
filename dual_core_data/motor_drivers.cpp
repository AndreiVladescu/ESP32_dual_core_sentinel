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

/* Function to initialise motors */
void initMotors() {
  /* System-wide steppers */
  stepper_az.setMaxSpeed(10000);
  stepper_az.setSpeed(1000);
  stepper_az.setAcceleration(1000);
  stepper_az.setEnablePin(ENABLE_PIN_AZ);
  stepper_az.disableOutputs();

  stepper_el.setMaxSpeed(10000);
  stepper_el.setSpeed(1000);
  stepper_el.setAcceleration(1000);
  stepper_el.setEnablePin(ENABLE_PIN_EL);
  stepper_el.disableOutputs();

  /* Visor servos */
  servo_az.attach(SERVO_PIN_AZ);
  servo_el.attach(SERVO_PIN_EL);
  servo_az.write(90);
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

  while (!dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)) {
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  float angle = -orientation.pitch;
  stepper_el.moveTo(angle * MICROSTEPS / 1.8);
  while (stepper_el.distanceToGo() != 0) {
    stepper_el.run();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  el_angle = 90;

  /* Azimuth homing based on end-stop switch */
  long max_distance = -999999;
  stepper_az.moveTo(max_distance * MICROSTEPS);
  while (stepper_az.distanceToGo() != 0) {
    stepper_az.run();
    // If stepper arrived to end switch, it's at almost 180 degrees out of phase
    //
    if (digitalRead(SW_PIN_HOME) == LOW) {
      break;
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  while (digitalRead(SW_PIN_HOME) == LOW) {
    stepper_az.move(50);
    stepper_az.runSpeed();
  }
  stepper_az.moveTo(175 * MICROSTEPS / 1.8);
  while (stepper_az.distanceToGo() != 0) {
    stepper_az.run();
  }
  stepper_az.setCurrentPosition(0);
}

/* Function to action the trigger */
void fireProcedure(){
  servo_trg.write(TRIGGER_FIRE_POS);
  delay(1000);
  servo_trg.write(TRIGGER_REST_POS);
  delay(100);
}