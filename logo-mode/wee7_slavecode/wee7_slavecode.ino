#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>

// ----- Motor Driver Pin Definitions -----
#define enA 33  // Right motor speed (PWM)
#define enB 25  // Left motor speed (PWM)
#define INa 26  // Right motor forward
#define INb 27  // Right motor backward
#define INc 14  // Left motor forward
#define INd 12  // Left motor backward

// ----- Steering Servo Definition -----
Servo steeringServo;  
#define STEERING_SERVO_PIN 13  // GPIO 13 controls the servo

// ----- PWM Setup for ESP32 using LEDC with new API -----
const int pwmFreq = 5000;      // 5 kHz PWM frequency
const int pwmResolution = 8;   // 8-bit resolution (0-255)
int motorSpeed = 230;          // Motor speed (0-255)

// ----- Define servo angles (center is 75°) -----
// Adjust these values as needed.
const int centerAngle = 90;           // Center position
const int lyeftAngle = centerAngle - 90; // e.g. 75 - 90 = -15° (or adjust as desired)
const int rightAngle = centerAngle + 90; // e.g. 75 + 90 = 165°

// ----- ESP-NOW Message Structure -----
// Now holds a full sequence of commands (max 16 characters plus null terminator)
typedef struct struct_message {
  char commands[17];
} struct_message;
struct_message receivedData;

String commandSequence = "";
bool commandReady = false;

// ----- ESP-NOW Receive Callback -----
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  commandSequence = String(receivedData.commands);
  commandReady = true;
  Serial.println("--- COMMAND EXECUTION ---");
}

void setup() {
  Serial.begin(115200);
  
  // Set motor direction pins as OUTPUT
  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  
  // Setup LEDC for motor speed control using new API:
  ledcAttach(enA, pwmFreq, pwmResolution);
  ledcAttach(enB, pwmFreq, pwmResolution);
  
  // Attach the steering servo and center it
  steeringServo.attach(STEERING_SERVO_PIN);
  steeringServo.write(centerAngle);
  
  // Set WiFi to Station mode and initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("Slave ready to receive command sequences.");
}

void loop() {
  // If a command sequence has been received, process each command
  if (commandReady) {
    for (unsigned int i = 0; i < commandSequence.length(); i++) {
      char c = commandSequence.charAt(i);
      Serial.print("Executing command: ");
      Serial.println(c);
      executeCommand(c);
      
      unsigned long commandDelay;
      if (c == '4') {
        commandDelay = 500;
        Serial.println("Received Command: LEFT 90");
        Serial.println("Executing: Turn LEFT 90 degrees");
      } else if (c == '6') {
        commandDelay = 500;
        Serial.println("Received Command: RIGHT 90");
        Serial.println("Executing: Turn Right 90 degrees");
      } else if (c == '1') {
        commandDelay = 1600;
        Serial.println("Received Command: FORWARD 1");
        Serial.println("Executing: Move Forward 80 cm");
      } else if (c == '2') {
        commandDelay = 800;
        Serial.println("Received Command: FORWARD 2");
        Serial.println("Executing: Move Forward 40 cm");
      } else if (c == '0') {
        commandDelay = 600;
        Serial.println("Received Command: STOP");
        Serial.println("Executing: Stopping Motors...");  
      } 
       else {
        commandDelay = 600;
      }
      delay(commandDelay);

    }
    // After processing all commands, clear the sequence and reset the flag
    commandSequence = "";
    commandReady = false;
  }
}

// ----- Function to Execute a Single Command -----
void executeCommand(char c) {
  switch(c) {
    case '1': moveForward(); centerSteering(); break;
    case '2': moveForward(); centerSteering(); break;
    case '4': pivotTurnLeft(); break;
    case '6': pivotTurnRight(); break;
    case '0': stopMotors(); break;
    default:
      Serial.println("Invalid command in sequence.");
      break;
  }
}

//  Motor and Servo Control Functions

void moveForward() {
  // Right motor forward: INa LOW, INb HIGH; Left motor forward: INc HIGH, INd LOW
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, HIGH);
  digitalWrite(INd, LOW);
  ledcWrite(enA, motorSpeed);
  ledcWrite(enB, motorSpeed);
}

void moveBackward() {
  // Right motor backward: INa HIGH, INb LOW; Left motor backward: INc LOW, INd HIGH
  digitalWrite(INa, HIGH);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
  ledcWrite(enA, motorSpeed);
  ledcWrite(enB, motorSpeed);
}

void stopMotors() {
  // Stop motors: set all direction pins LOW and PWM to 0
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

void pivotTurnLeft() {
  // Pivot turn left: right motor moves forward, left motor moves backward.
  // Increase left motor speed if necessary (adjust as needed).
  
  digitalWrite(INa, HIGH);   // Right motor forward
  digitalWrite(INb, LOW);
  digitalWrite(INc, HIGH);   // Left motor backward
  digitalWrite(INd, LOW);
  ledcWrite(enA, motorSpeed);
  ledcWrite(enB, motorSpeed);
  
  // Set servo to left pivot angle
  steeringServo.write(leftAngle);
  Serial.print("Pivot Turning Left (servo set to ");
  Serial.print(leftAngle);
  Serial.println("°)");
}

void pivotTurnRight() {
  // Pivot turn right: l]';/.ft motor moves forward, right motor moves backward.
  digitalWrite(INa, LOW);  // Right motor backward
  digitalWrite(INb, HIGH);
  digitalWrite(INc, LOW);  // Left motor forward
  digitalWrite(INd, HIGH);
  ledcWrite(enA, motorSpeed);
  ledcWrite(enB, motorSpeed);
  
  // Set servo to right pivot angle
  steeringServo.write(rightAngle);
  Serial.print("Pivot Turning Right (servo set to ");
  Serial.print(rightAngle);
  Serial.println("°)");
}