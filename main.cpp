#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#define LEFT_MOTOR_PWM 4
#define LEFT_MOTOR_DIR 2
#define RIGHT_MOTOR_PWM 5
#define RIGHT_MOTOR_DIR 18

const int freq = 1000;
const int resolution = 8;
const int pwmChannel_LEFT = 0;
const int pwmChannel_RIGHT = 1;

QueueHandle_t commandQueue;

typedef enum {
  CMD_FORWARD = 'F',
  CMD_BACKWARD = 'B',
  CMD_LEFT = 'L',
  CMD_RIGHT = 'R',
  CMD_STOP = 'S',
  CMD_NONE = 0
} Command_t;

void moveForward() {
  digitalWrite(LEFT_MOTOR_DIR, HIGH);
  digitalWrite(RIGHT_MOTOR_DIR, HIGH);
  ledcWrite(pwmChannel_LEFT, 200);
  ledcWrite(pwmChannel_RIGHT, 200);
}

void moveBackward() {
  digitalWrite(LEFT_MOTOR_DIR, LOW);
  digitalWrite(RIGHT_MOTOR_DIR, LOW);
  ledcWrite(pwmChannel_LEFT, 200);
  ledcWrite(pwmChannel_RIGHT, 200);
}

void turnLeft() {
  digitalWrite(LEFT_MOTOR_DIR, LOW);
  digitalWrite(RIGHT_MOTOR_DIR, HIGH);
  ledcWrite(pwmChannel_LEFT, 200);
  ledcWrite(pwmChannel_RIGHT, 200);
}

void turnRight() {
  digitalWrite(LEFT_MOTOR_DIR, HIGH);
  digitalWrite(RIGHT_MOTOR_DIR, LOW);
  ledcWrite(pwmChannel_LEFT, 200);
  ledcWrite(pwmChannel_RIGHT, 200);
}

void stopMotors() {
  ledcWrite(pwmChannel_LEFT, 0);
  ledcWrite(pwmChannel_RIGHT, 0);
}

BluetoothSerial SerialBT;

void BluetoothTask() {
  while (1) {
    if (SerialBT.available()) {
      char receivedChar = SerialBT.read();
      Command_t command = (Command_t)receivedChar;

      if (command == CMD_FORWARD || command == CMD_BACKWARD ||
          command == CMD_LEFT || command == CMD_RIGHT || command == CMD_STOP) {

        xQueueSend(commandQueue, &command, 0);
      }
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void MotorControlTask() {
  Command_t currentCommand = CMD_STOP;
  Command_t newCommand = CMD_NONE;

  while (1) {
    if (xQueueReceive(commandQueue, &newCommand, 0) == pdTRUE) {
      currentCommand = newCommand;
    }

    switch (currentCommand) {
    case CMD_FORWARD:
      moveForward();
      break;
    case CMD_BACKWARD:
      moveBackward();
      break;
    case CMD_LEFT:
      turnLeft();
      break;
    case CMD_RIGHT:
      turnRight();
      break;
    case CMD_STOP:
      stopMotors();
      break;
    default:
      break;
    }

    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void setupMotorPins() {
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
}

void setup() {
  Serial.begin(115200);

  setupMotorPins();

  commandQueue = xQueueCreate(5, sizeof(Command_t));

  SerialBT.begin("ESP32");

  xTaskCreatePinnedToCore(BluetoothTask, "BluetoothTask", 4096, NULL, 1, NULL,
                          0);

  xTaskCreatePinnedToCore(MotorControlTask, "MotorControlTask", 4096, NULL, 2,
                          NULL, 1);

  stopMotors();
}

void loop() { vTaskDelete(NULL); }
