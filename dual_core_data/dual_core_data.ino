#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_L3GD20_U.h>

typedef struct median_orientation_t {
  float roll = 0,
        pitch = 0,
        heading = 0;
  uint8_t overSampleRate = 31;
  uint8_t currentSampleCounter = 0;
};

static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 0;


TaskHandle_t Task1, Task2, Task3, TaskCommand;

static SemaphoreHandle_t sensor_mutex;

volatile median_orientation_t medianOrientation;

Adafruit_9DOF dof = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(30302);

// This is your shared flag. Volatile keyword is used here
// to prevent the compiler from optimizing out checks to this variable.
volatile bool dataReadyFlag = false;

volatile bool sendDataFlag = false;

/* Control */
const int fire_ctrl = 0x10;
const int move_step_a_ctrl = 0x20;
const int move_step_r_ctrl = 0x30;
const int send_orientation_ctrl = 0x40;
const int move_servo_a_ctrl = 0x50;
const int move_servo_r_ctrl = 0x60;
const int scram_ctrl = 0x70;

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

bool manageCommands(byte ctrl) {
  if (ctrl == fire_ctrl) {
    /*Fire*/
    Serial.println("Fire");
  } else if (ctrl == move_step_a_ctrl) {
    /*Move*/
    Serial.println("Stepper a");
    return true;
  } else if (ctrl == move_step_r_ctrl) {
    /*Move*/
    Serial.println("Stepper r");
    return true;
  } else if (ctrl == send_orientation_ctrl) {
    sendDataFlag = true;
  } else if (ctrl == move_servo_a_ctrl) {
    /*Move*/
    Serial.println("Servo a");
    return true;
  } else if (ctrl == move_servo_r_ctrl) {
    /*Move*/
    Serial.println("Servo r");
    return true;
  } else if (ctrl == scram_ctrl) {
    /*Scram*/
    Serial.println("Scram");
  } else {
    /*Carry on*/
    Serial.println("Carry on");
  }
  return false;
}

// This task will run on core 0 and will receive data
void receiveDataTask(void *pvParameters) {
  static double new_angle = 0;
  static const int max_chars = 7;
  static char rx_string[max_chars];
  static int8_t rx_i = -1;
  static byte buffer_byte;
  static bool temp_rc = false;

  while (1) {
    // Wait until data is received
    while (Serial.available() > 0) {
      char ch = Serial.read();

      Serial.println(ch, HEX);

      if (rx_i == -1){
        buffer_byte = (byte)ch;
        Serial.print("Buffer byte is: ");
        Serial.println(buffer_byte, HEX);
        rx_i++;
        continue;
      }

      if (rx_i < max_chars && (isDigit(ch) || ch == '.')) {
        rx_string[rx_i++] = ch;
      } else {
        if (ch == '\n')
          temp_rc = manageCommands((byte)buffer_byte);

        if (temp_rc) {
          rx_string[rx_i] = 0;
          new_angle = atof(rx_string);
          Serial.println(new_angle, 2);
          temp_rc = false;
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
        medianOrientation.roll = medianOrientation.roll / medianOrientation.overSampleRate;
        medianOrientation.pitch = medianOrientation.pitch / medianOrientation.overSampleRate;
        medianOrientation.heading = medianOrientation.heading / medianOrientation.overSampleRate;

        medianOrientation.currentSampleCounter = 0;

        // Signal that data is ready
        dataReadyFlag = true;
        //Serial.println("Core 2 finished work");
      } else {
        medianOrientation.currentSampleCounter++;
        medianOrientation.roll += orientation.roll;
        medianOrientation.pitch += orientation.pitch;
        medianOrientation.heading += orientation.heading;
      }
    }
    xSemaphoreGive(sensor_mutex);

    // This delay helps to prevent the task from hogging CPU time
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Adafruit 9 DOF Pitch/Roll/Heading Example"));

  /* Initialise the sensors */
  initSensors();

  /* Initialise Mutex */
  sensor_mutex = xSemaphoreCreateMutex();

  /* Create the tasks */
  xTaskCreatePinnedToCore(sendDataTask,
                          "Send Data Task",
                          8192,
                          NULL,
                          1,
                          &Task1,
                          pro_cpu);

  xTaskCreatePinnedToCore(receiveDataTask,
                          "Receive Data Task",
                          8192,
                          NULL,
                          1,
                          &Task2,
                          pro_cpu);

  xTaskCreatePinnedToCore(computeDataTask,
                          "Compute Data Task",
                          8192,
                          NULL,
                          1,
                          &Task3,
                          app_cpu);
                          
  /* Delete setup and loop tasks */
  vTaskDelete(NULL);
}

void loop() {
  /* Should not run */
}
