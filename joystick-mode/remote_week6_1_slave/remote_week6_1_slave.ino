#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>


// —— Motor Driver Pins ——
// adjust to your wiring
// ----- Motor Driver Pin Definitions -----
#define enA 25  // Right motor speed (PWM)
#define enB 33  // Left motor speed (PWM) 
#define INa 26  // Right motor forward
#define INb 27  // Right motor backward
#define INc 14  // Left motor forward
#define INd 12  // Left motor backward

// ----- Steering Servo Definition -----
Servo steeringServo;  
#define servoPin 13  // GPIO 13 controls the servo

// ----- PWM Setup for ESP32 using LEDC with new API -----
const int pwmFreq = 5000;      // 5 kHz PWM frequency
const int pwmResolution = 8;   // 8-bit resolution (0-255)
int motorSpeed = 255;          // Motor speed (0-255)

// ----- Define servo angles (center is 75°) -----
// Adjust these values as needed.
const int centerAngle = 90;           // Center position
const int leftAngle = centerAngle - 90; // e.g. 75 - 90 = -15° (or adjust as desired)
const int rightAngle = centerAngle + 100; // e.g. 75 + 90 = 165°

// —— Received Data ——
typedef struct { int16_t xAxis, yAxis; } JoyData;
JoyData joy = {0,0};

// —— ESP‑NOW Receive Callback ——
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incoming, int len) {
  memcpy(&joy, incoming, sizeof(joy));
  
}


// —— Motor / Steering Routines ——
void moveForward(int speed) { 
  digitalWrite(INa, LOW);  
  digitalWrite(INb, HIGH);
  digitalWrite(INc, HIGH); 
  digitalWrite(INd, LOW);
  ledcWrite(enA, motorSpeed);
  ledcWrite(enB, motorSpeed);
  Serial.println("Moving Forward");
}

void moveBackward(int speed) {
  digitalWrite(INa, HIGH); 
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);  
  digitalWrite(INd, HIGH);
  ledcWrite(enA, motorSpeed);
  ledcWrite(enB, motorSpeed);
  Serial.println("Moving Backward");
}


void stopMotors() {
  digitalWrite(INa, LOW); 
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW); 
  digitalWrite(INd, LOW);
  ledcWrite(enA, 0);
  ledcWrite(enB, 0);
}

void centerSteering() {
  
  steeringServo.write(centerAngle);
}

// Drive forward but with a slower left wheel
void movingLeft (int turn) {
  // both wheels forward
  steeringServo.write(leftAngle);
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);

  ledcWrite(enA, 200);
  ledcWrite(enB, 200);
  Serial.println("Moving Left");
}


void movingRight(int turn) {
  steeringServo.write(rightAngle);
  digitalWrite(INa, LOW);  
  digitalWrite(INb, HIGH);
  digitalWrite(INc, LOW); 
  digitalWrite(INd, HIGH);

  ledcWrite(enA, 200);
  ledcWrite(enB, 200);
  Serial.println("Moving Right");
}

void setup() {
  Serial.begin(115200);

  // motor pins
  pinMode(INa, OUTPUT); 
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT); 
  pinMode(INd, OUTPUT);

  ledcAttach(enA, pwmFreq, pwmResolution);
  ledcAttach(enB, pwmFreq, pwmResolution);

  // steering
  steeringServo.attach(servoPin);
  steeringServo.write(centerAngle);

  // ESP‑NOW init
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);
}

void loop() {
  int speed = joy.yAxis;       // 0–255
  int turn  = joy.xAxis;       // –255…255

  // —— Drive Logic ——  
  if (joy.yAxis < -100)
  {
    moveForward(speed);
  } 
  else if (joy.yAxis > 100) 
  {
    moveBackward(speed);
  } 
  else {
    stopMotors();
  }

  // —— Steering / Pivot ——  
  if (turn > 50) 
  {
    movingRight(turn);
  } 
  else if (turn < -50) 
  {
    movingLeft(turn);
  } 
  else 
  {
    centerSteering();
  }

  // debug
  Serial.printf("RX:%4d RY:%4d → spd:%3d turn:%4d\n",
                joy.xAxis, joy.yAxis, speed, turn);

  delay(50);
}
