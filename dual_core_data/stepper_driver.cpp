#include <AccelStepper.h>
#include "stepper_driver.h"

AccelStepper stepper_az(MOTOR_INTERFACE, STEP_PIN_AZ, DIR_PIN_AZ);
AccelStepper stepper_el(MOTOR_INTERFACE, STEP_PIN_EL, DIR_PIN_EL);

void test(int test)
{
  test += 5;
}