#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

#include "stepper_driver.h"

typedef struct median_orientation_t {
  float roll = 0,
        pitch = 0,
        heading = 0;
  uint8_t overSampleRate = 31;
  uint8_t currentSampleCounter = 0;
};

static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 0;

TaskHandle_t TaskSendData,
  TaskRecvData,
  TaskComputeData,
  TaskMotors,
  TaskCommand;

static SemaphoreHandle_t sensor_mutex;

volatile median_orientation_t medianOrientation;

Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);

/* System-wide steppers */
extern AccelStepper stepper_az;
extern AccelStepper stepper_el;

/* Visor servos */
Servo servo_az;
Servo servo_el;
Servo servo_trg;

// This is your shared flag. Volatile keyword is used here
// to prevent the compiler from optimizing out checks to this variable.
volatile bool dataReadyFlag = false;
volatile bool sendDataFlag = false;

/* Control */
const int fire_ctrl = 0x10;
const int move_step_a_ctrl = 0x20;
const int move_step_e_ctrl = 0x30;
const int send_orientation_ctrl = 0x40;
const int move_servo_a_ctrl = 0x50;
const int move_servo_e_ctrl = 0x60;
const int scram_ctrl = 0x53;
const int restore_ctrl = 0x52;

/* Current positon */
double az_angle = 0;
double el_angle = 0;

void initMotors() {
  /* System-wide steppers */
  stepper_el.setMaxSpeed(10000);
  stepper_el.setSpeed(1000);
  stepper_el.setAcceleration(1000);
  stepper_el.setEnablePin(ENABLE_PIN_EL);
  stepper_el.disableOutputs();

  stepper_az.setMaxSpeed(10000);
  stepper_az.setSpeed(1000);
  stepper_az.setAcceleration(1000);
  stepper_az.setEnablePin(ENABLE_PIN_AZ);
  stepper_az.disableOutputs();

  /* Visor servos */
  servo_az.attach(SERVO_PIN_AZ);
  servo_el.attach(SERVO_PIN_EL);
  servo_az.write(90);
  servo_el.write(90);

  /* Trigger servo */
  servo_trg.attach(SERVO_PIN_TRG);

}

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

// This task will run on core 0 and will send data when it's ready
void sendDataTask(void *pvParameters) {
  while (1) {
    // Wait until data is ready
    while (!dataReadyFlag || !sendDataFlag) {
      vTaskDelay(1);  // This delay helps to prevent the task from hogging CPU time
    }
    xSemaphoreTake(sensor_mutex, portMAX_DELAY);
    Serial.print(int(medianOrientation.roll));
    Serial.print(',');
    Serial.print(int(medianOrientation.pitch));
    Serial.print(',');
    Serial.println(int(medianOrientation.heading));

    medianOrientation.roll = 0;
    medianOrientation.pitch = 0;
    medianOrientation.heading = 0;
    xSemaphoreGive(sensor_mutex);

    // Reset the flags
    dataReadyFlag = false;
    sendDataFlag = false;
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void manageCommands(byte ctrl, const char *rx_string) {
  double angle;

  if (ctrl == fire_ctrl) {
    /*Fire*/
    Serial.println("Fire");
  } else if (ctrl == move_step_a_ctrl) {
    /*Move*/
    Serial.println("Stepper a");
    angle = atof(rx_string);
    if (angle >= SFZN_SYS_LWR_BND_AZ && angle <= SFZN_SYS_HGH_BND_AZ) {
      stepper_az.moveTo(angle * MICROSTEPS / 1.8);
    } else {
      Serial.println("Stepper out of bounds");
    }
  } else if (ctrl == move_step_e_ctrl) {
    /*Move*/
    Serial.println("Stepper e");
    angle = atof(rx_string);
    if (angle >= SFZN_SYS_LWR_BND_EL && angle <= SFZN_SYS_HGH_BND_EL) {
      stepper_el.moveTo(angle * MICROSTEPS / 1.8);
    } else {
      Serial.println("Stepper out of bounds");
    }
  } else if (ctrl == send_orientation_ctrl) {
    /*Send orientation data*/
    sendDataFlag = true;
  } else if (ctrl == move_servo_a_ctrl) {
    /*Move*/
    Serial.println("Servo a");
    angle = atof(rx_string);
    if (angle >= SFZN_VIZ_LWR_BND_AZ && angle <= SFZN_VIZ_HGH_BND_AZ) {
      servo_az.write(angle);
    } else {
      Serial.println("Servo out of bounds");
    }
  } else if (ctrl == move_servo_e_ctrl) {
    /*Move*/
    Serial.println("Servo e");
    angle = atof(rx_string);
    if (angle >= SFZN_VIZ_LWR_BND_EL && angle <= SFZN_VIZ_HGH_BND_EL) {
      servo_az.write(angle);
    } else {
      Serial.println("Servo out of bounds");
    }
  } else if (ctrl == scram_ctrl) {
    /*Scram*/
    Serial.println("Scram");
    // Inverted controls
    stepper_az.enableOutputs();
    stepper_el.enableOutputs();
  } else if (ctrl == restore_ctrl) {
    /*Restore motor function*/
    Serial.println("Restore Control");
    // Inverted controls
    stepper_az.disableOutputs();
    stepper_el.disableOutputs();
  } else {
    /*Carry on*/
    Serial.println("Carry on");
  }
}

// This task will run on core 0 and will receive data
void receiveDataTask(void *pvParameters) {
  static double new_angle = 0;
  static const int max_chars = 7;
  static char rx_string[max_chars];
  static int8_t rx_i = -1;
  static byte buffer_byte;

  while (1) {
    // Wait until data is received
    while (Serial.available() > 0) {
      char ch = Serial.read();

      //Serial.println(ch, HEX);

      if (rx_i == -1) {
        buffer_byte = (byte)ch;
        //Serial.print("Buffer byte is: ");
        //Serial.println(buffer_byte, HEX);
        rx_i++;
        continue;
      }

      if (rx_i < max_chars && (isDigit(ch) || ch == '.')) {
        rx_string[rx_i++] = ch;
      } else {
        if (ch == '\n') {
          rx_string[rx_i] = 0;
          manageCommands((byte)buffer_byte, rx_string);
        }
        rx_i = -1;
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// This task will run on core 1 and will prepare data
void computeDataTask(void *pvParameters) {
  while (1) {
    if (dataReadyFlag) {
      vTaskDelay(1 / portTICK_PERIOD_MS);
      continue;
    }

    xSemaphoreTake(sensor_mutex, portMAX_DELAY);

    sensors_event_t accel_event;
    sensors_event_t mag_event;
    sensors_vec_t orientation;

    accel.getEvent(&accel_event);
    mag.getEvent(&mag_event);

    if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation)) {
      if (medianOrientation.currentSampleCounter == medianOrientation.overSampleRate) {
        //medianOrientation.roll = medianOrientation.roll / medianOrientation.overSampleRate;
        medianOrientation.pitch = medianOrientation.pitch / medianOrientation.overSampleRate;
        medianOrientation.heading = medianOrientation.heading / medianOrientation.overSampleRate;

        medianOrientation.currentSampleCounter = 0;

        // Signal that data is ready
        dataReadyFlag = true;
        //Serial.println("Core 2 finished work");
      } else {
        medianOrientation.currentSampleCounter++;
        //medianOrientation.roll += orientation.roll;
        medianOrientation.pitch += orientation.pitch;
        medianOrientation.heading += orientation.heading;
      }
    }
    xSemaphoreGive(sensor_mutex);

    // This delay helps to prevent the task from hogging CPU time
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void motorsTask(void *pvParameters) {
  while (1) {
    stepper_el.run();
    stepper_az.run();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

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

void gpioInit() {
  pinMode(SW_PIN_HOME, INPUT_PULLUP);
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Adafruit 9 DOF Pitch/Roll/Heading Example"));

  /* Initialise the sensors */
  initSensors();

  /* Initialise the motors */
  initMotors();

  /* GPIO config */
  gpioInit();

  /* Home in */
  homingProcedure();

  /* Initialise Mutex */
  sensor_mutex = xSemaphoreCreateMutex();

  /* Create the tasks */
  xTaskCreatePinnedToCore(sendDataTask,
                          "Send Data Task",
                          8192,
                          NULL,
                          1,
                          &TaskSendData,
                          pro_cpu);

  xTaskCreatePinnedToCore(receiveDataTask,
                          "Receive Data Task",
                          8192,
                          NULL,
                          1,
                          &TaskRecvData,
                          pro_cpu);

  xTaskCreatePinnedToCore(computeDataTask,
                          "Compute Data Task",
                          8192,
                          NULL,
                          1,
                          &TaskComputeData,
                          app_cpu);

  xTaskCreatePinnedToCore(motorsTask,
                          "Motors running Task",
                          8192,
                          NULL,
                          1,
                          &TaskMotors,
                          app_cpu);

  /* Delete setup and loop tasks */
  vTaskDelete(NULL);
}

void loop() {
  /* Should not run */
}
