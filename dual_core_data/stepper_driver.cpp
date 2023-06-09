#include <AccelStepper.h>
#include "stepper_driver.h"

AccelStepper stepper_az(MOTOR_INTERFACE, STEP_PIN, DIR_PIN);
AccelStepper stepper_el(MOTOR_INTERFACE, STEP_PIN, DIR_PIN);

void test(int test)
{
  test += 5;
}