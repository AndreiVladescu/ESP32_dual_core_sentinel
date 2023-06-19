#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "motor_drivers.h"
#include "gpio_aux.h"

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
extern Servo servo_az;
extern Servo servo_el;
extern Servo servo_trg;

// This is your shared flag. Volatile keyword is used here
// to prevent the compiler from optimizing out checks to this variable.
volatile bool dataReadyFlag = false;
volatile bool sendDataFlag = false;

/* Current positon */
double az_angle = 0;
double el_angle = 0;

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
    fireProcedure();
  } else if (ctrl == move_step_a_ctrl) {
    /*Move*/
    Serial.println("Stepper a");
    angle = atof(rx_string);
    if (angle >= SFZN_SYS_LWR_BND_AZ && angle <= SFZN_SYS_HGH_BND_AZ) {
      stepper_az.moveTo(angle * MICROSTEPS / 1.8);
      motorCallback(&stepper_az);
    } else {
      Serial.println("Stepper out of bounds");
    }
  } else if (ctrl == move_step_e_ctrl) {
    /*Move*/
    Serial.println("Stepper e");
    angle = atof(rx_string);
    if (angle >= SFZN_SYS_LWR_BND_EL && angle <= SFZN_SYS_HGH_BND_EL) {
      stepper_el.moveTo(angle * MICROSTEPS / 1.8);
      motorCallback(&stepper_el);
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
