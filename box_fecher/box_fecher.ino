#include <SoftwareSerial.h>
#include <Servo.h>

const uint8_t DOOR_CMD_OPEN = 0x30; // Key : 0
const uint8_t DOOR_CMD_CLOSE = 0x31; // Key : 1
const uint8_t SERVO_CMD_KEEP = 0x32; // Key : 0
const uint8_t SERVO_CMD_DROP = 0x33; // Key : 1

const uint32_t DOOR_OPEN = 100;
const uint32_t DOOR_CLOSE = 0;
const uint32_t SERVO_ANGLE_KEEP = 0;
const uint32_t SERVO_ANGLE_DROP = 60;//暂时定60，看情况改

#define LED_PIN LED_BUILTIN
//#define BT_RX_PIN 10
//#define BT_TX_PIN 11

#define SERVO_NUM 2
const int SERVO_PINS[SERVO_NUM] = {6,9};

//SoftwareSerial bt(BT_RX_PIN, BT_TX_PIN); // RX, TX
Servo servos[SERVO_NUM];

#define SERVO_ATTACH() do { for (uint8_t i = 0; i < SERVO_NUM; i++) servos[i].attach(SERVO_PINS[i]); } while(0)
#define DOOR_OPEN()  do { servos[1].write(DOOR_OPEN); } while(0)
#define DOOR_CLOSE()  do { servos[1].write(DOOR_CLOSE); } while(0)
#define SERVO_KEEP()  do { servos[0].write(SERVO_ANGLE_KEEP); } while(0)
#define SERVO_DROP()  do { servos[0].write(SERVO_ANGLE_DROP); } while(0)
#define LED_ON() digitalWrite(LED_PIN, HIGH)
#define LED_OFF() digitalWrite(LED_PIN, LOW)
#define LED_TOG() digitalWrite(LED_PIN, !digitalRead(LED_PIN))


uint8_t btKeyCode = 0;
uint8_t spKeyCode = 0;


void setup()
{
  SERVO_ATTACH();//初始化SERVO
  SERVO_KEEP();
  DOOR_CLOSE();
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  //bt.begin(9600);
}

void switchStateCmd(uint8_t cmd)
{
  switch (cmd) {
    case DOOR_CMD_OPEN:
      DOOR_OPEN();
      break;
    case DOOR_CMD_CLOSE:
      DOOR_CLOSE();
      break;
    case SERVO_CMD_KEEP:
      SERVO_KEEP();
      break;
    case SERVO_CMD_DROP:
      SERVO_DROP();
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
  if (Serial.available()) {
    spKeyCode = Serial.read();
    switchStateCmd(spKeyCode);
  }
  blinkLed();
}
