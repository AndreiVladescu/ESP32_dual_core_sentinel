#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "motor_drivers.h"
#include "gpio_aux.h"

static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

TaskHandle_t TaskSendData,
  TaskRecvData,
  TaskComputeData,
  TaskMotors,
  TaskFire,
  TaskMotorCallback;

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
volatile bool taskExecuted = false;

/* Current positon */
double az_angle = 0;
double el_angle = 0;

/* Laser variable */
bool laser_switch = false;

// This task will run on core 0 and will send data when it's ready
void sendDataTask(void *pvParameters) {
  while (1) {
    // Wait until data is ready
    while (!dataReadyFlag || !sendDataFlag) {
      vTaskDelay(5);  // This delay helps to prevent the task from hogging CPU time
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
    vTaskDelay(5);
  }
}

void manageCommands(byte ctrl, const char *rx_string) {
  double angle;

  if (ctrl == fire_ctrl) {
    /*Fire*/
    xTaskCreate(fireProcedure,
                "Trigger Task",
                4096,
                NULL,
                3,
                &TaskFire);
  } else if (ctrl == move_step_a_ctrl) {
    /*Move*/
    //Serial.println("Stepper a");
    angle = atof(rx_string);
    if (angle >= SFZN_SYS_LWR_BND_AZ && angle <= SFZN_SYS_HGH_BND_AZ) {
      stepper_az.moveTo(angle * MICROSTEPS / 1.8 * TEETH_GEAR_RATIO);
      xTaskCreate(motorCallback,
                  "Motor Callback Azimuth Task",
                  2048,
                  &stepper_az,
                  2,
                  &TaskMotorCallback);
    } else {
      Serial.println("Stepper out of bounds");
    }
  } else if (ctrl == move_step_e_ctrl) {
    /*Move*/
   // Serial.println("Stepper e");
    angle = atof(rx_string);
    if (angle >= SFZN_SYS_LWR_BND_EL && angle <= SFZN_SYS_HGH_BND_EL) {
      stepper_el.moveTo(angle * MICROSTEPS / 1.8);
      xTaskCreate(motorCallback,
                  "Motor Callback Elevation Task",
                  2048,
                  &stepper_el,
                  2,
                  &TaskMotorCallback);
    } else {
      Serial.println("Stepper out of bounds");
    }
  } else if (ctrl == send_orientation_ctrl) {
    /*Send orientation data*/
    sendDataFlag = true;
  } else if (ctrl == move_servo_a_ctrl) {
    /*Move*/
    angle = atof(rx_string);
    if (angle >= SFZN_VIZ_LWR_BND_AZ && angle <= SFZN_VIZ_HGH_BND_AZ) {
      servo_az.write(angle);
      Serial.println("Srv a");
    } else {
      Serial.println("Servo out of bounds");
    }
  } else if (ctrl == move_servo_e_ctrl) {
    /*Move*/
    angle = atof(rx_string);
    if (angle >= SFZN_VIZ_LWR_BND_EL && angle <= SFZN_VIZ_HGH_BND_EL) {
      servo_el.write(angle);
      Serial.println("Srv e");
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
  } else if (ctrl == laser_ctrl) {
    /*Toggle Laser*/
    Serial.println("Laser Toggle");
    laser_switch = !laser_switch;
    digitalWrite(SW_LASER, laser_switch);
  } else {
    /*Carry on*/
    Serial.println("Carry on");
  }
  taskExecuted = true;
  vTaskDelay(5);
}

// This task will run on core 0 and will receive data
void receiveDataTask(void *pvParameters) {
  static double new_angle = 0;
  static const int max_chars = 9;
  static char rx_string[max_chars];
  static int8_t rx_i = -1;
  static byte buffer_byte;

  unsigned long startTime;
  unsigned long endTime;

  while (1) {
    //startTime = millis();
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

      if (rx_i < max_chars && (isDigit(ch) || ch == '.' || ch == '-')) {
        rx_string[rx_i++] = ch;
      } else {
        if (ch == '\n') {
          rx_string[rx_i] = 0;
          manageCommands((byte)buffer_byte, rx_string);
          // endTime = millis();
          //unsigned long executionTime = endTime - startTime;

          // Output execution time
          //Serial.print("Execution Time: ");
          //Serial.print(executionTime);
          //Serial.println(" ms");
        }
        rx_i = -1;
      }
    }
    vTaskDelay(5);
  }
}

// This task will run on core 1 and will prepare data
void computeDataTask(void *pvParameters) {
  while (1) {
    if (dataReadyFlag) {
      vTaskDelay(5);
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
    vTaskDelay(5);
  }
}

void motorsTask(void *pvParameters) {
  while (1) {
    if (stepper_az.distanceToGo() != 0 || stepper_el.distanceToGo() != 0) {
      stepper_el.run();
      stepper_az.run();
    } else {
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
}

void confirmConnection() {
  while (Serial.available() == 0) {
    vTaskDelay(10);
  }
  while (Serial.available() > 0) {
    char ch = Serial.read();
  }
}

void setup() {
  Serial.begin(115200);

  /* Initialise the sensors */
  initSensors();

  /* Initialise the motors */
  initMotors();

  /* GPIO config */
  gpioInit();

  /* Get confirmation of a PC connection */
  // confirmConnection();

  /* Home in */
  homingProcedure();

  /* Initialise Mutex */
  sensor_mutex = xSemaphoreCreateMutex();

  /* Create the tasks */
  xTaskCreatePinnedToCore(sendDataTask,
                          "Send Data Task",
                          8192,
                          NULL,
                          tskIDLE_PRIORITY,
                          &TaskSendData,
                          pro_cpu);

  xTaskCreatePinnedToCore(receiveDataTask,
                          "Receive Data Task",
                          8192,
                          NULL,
                          configMAX_PRIORITIES - 1,
                          &TaskRecvData,
                          pro_cpu);

  xTaskCreatePinnedToCore(computeDataTask,
                          "Compute Data Task",
                          8192,
                          NULL,
                          tskIDLE_PRIORITY,
                          &TaskComputeData,
                          pro_cpu);

  xTaskCreatePinnedToCore(motorsTask,
                          "Motors running Task",
                          8192,
                          NULL,
                          configMAX_PRIORITIES - 1,
                          &TaskMotors,
                          app_cpu);

  /* Delete setup and loop tasks */
  vTaskDelete(NULL);
}

void loop() {
  /* Should not run */
}
