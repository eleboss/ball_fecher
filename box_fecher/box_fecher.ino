#include <SoftwareSerial.h>
#include <Servo.h>

const uint8_t SERVO_CMD_LOOSE = 0x68; // Key : 0
const uint8_t SERVO_CMD_TIGHT = 0x30; // Key : 1

const uint32_t SERVO_ANGLE_LOOSE = 0;
const uint32_t SERVO_ANGLE_TIGHT = 100;

#define LED_PIN LED_BUILTIN
#define IR_RX_PIN 3
#define BT_RX_PIN 10
#define BT_TX_PIN 11

#define SERVO_NUM 4
const int SERVO_PINS[SERVO_NUM] = {6, 7, 8, 9};

SoftwareSerial bt(BT_RX_PIN, BT_TX_PIN); // RX, TX
Servo servos[SERVO_NUM];

#define SERVO_ATTACH() do { for (uint8_t i = 0; i < SERVO_NUM; i++) servos[i].attach(SERVO_PINS[i]); } while(0)
#define SERVO_TIGHT()  do { for (uint8_t i = 0; i < SERVO_NUM; i++) servos[i].write(SERVO_ANGLE_LOOSE); } while(0)
#define SERVO_LOOSE()  do { for (uint8_t i = 0; i < SERVO_NUM; i++) servos[i].write(SERVO_ANGLE_TIGHT); } while(0)
#define LED_ON() digitalWrite(LED_PIN, HIGH)
#define LED_OFF() digitalWrite(LED_PIN, LOW)
#define LED_TOG() digitalWrite(LED_PIN, !digitalRead(LED_PIN))


uint8_t btKeyCode = 0;
uint8_t spKeyCode = 0;


void setup()
{
  SERVO_ATTACH();
  SERVO_LOOSE();
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  bt.begin(9600);
}

void switchStateCmd(uint8_t cmd)
{
  switch (cmd) {
    case SERVO_CMD_LOOSE:
      SERVO_LOOSE();
      break;
    case SERVO_CMD_TIGHT:
      SERVO_TIGHT();
      break;
     default:
      break;
  }
}

void blinkLed() {
  static uint32_t count = 0;
  if (++count == 10000) {
    count = 0;
    LED_TOG();
  }
}

void loop() {
  if (bt.available()) {
    btKeyCode = bt.read();
    switchStateCmd(btKeyCode);
  }
  if (Serial.available()) {
    spKeyCode = Serial.read();
    switchStateCmd(spKeyCode);
  }
  blinkLed();
}
