#include <LiquidCrystal.h>
#include <Keypad.h>
#include <WiFi.h>
#include <esp_now.h>

// ----- LCD Setup -----
LiquidCrystal lcd(19, 23, 18, 17, 16, 15);  // RS, E, DB4, DB5, DB6, DB7

// ----- Keypad Setup -----
const byte ROWS = 4;  
const byte COLS = 3;  
char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

byte rowPins[ROWS] = {32, 33, 25, 26};  // Rows (OUTPUT)
byte colPins[COLS] = {27, 14, 12};        // Columns (INPUT)
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ----- User Input -----
String userInput = "";

// ----- ESP-NOW Setup -----
// Replace with your slave (EEEBot) ESP32 MAC address:
uint8_t slaveMAC[] = {0x08,0x3A,0x8D,0x0D,0x91,0x98 };

typedef struct struct_message {
  char commands[17]; // Up to 16 command digits + null terminator
} struct_message;

struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.print("Enter Number:");
  
  // Enable internal pull-ups for keypad column pins
  pinMode(27, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);

  // Set WiFi mode and initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    while (true);
  }
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer (slave)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, slaveMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (true);
  }
}

void loop() {
  char key = keypad.getKey();
  if (key) {
    Serial.print("Key Pressed: ");
    Serial.println(key);
    
    // If a digit (0-9) is pressed, accumulate it into userInput
    if (key >= '0' && key <= '9') {
      // Only allow a maximum of 16 digits
      if (userInput.length() < 16) {
        userInput += key;
        lcd.setCursor(0, 1);
        lcd.print(userInput);
      }
    } 
    else if (key == '#') {  // '#' acts as "Enter" and sends the command sequence
      lcd.setCursor(0, 1);
      lcd.print(userInput);
      if (userInput.length() > 0) {
         // Validate: Ensure every character is one of the allowed commands
         // Allowed: '2' (forward), '4' (forward+steer left), '5' (backward), '6' (forward+steer right), '0' (stop)
         bool valid = true;
         for (unsigned int i = 0; i < userInput.length(); i++) {
           char c = userInput.charAt(i);
           if (c != '1' && c != '2' && c != '4' && c != '5' && c != '6' && c != '0') {
             valid = false;
             break;
           }
         }
         if (valid) {
             // Copy the command sequence into our message struct and send it.
             userInput.toCharArray(myData.commands, sizeof(myData.commands));
             esp_now_send(slaveMAC, (uint8_t *) &myData, sizeof(myData));
             Serial.print("Sent command sequence: ");
             Serial.println(myData.commands);
        } else {
             Serial.println("Invalid command(s) entered.");
              }
         // Clear input after sending
         userInput = "";
         lcd.clear();
         lcd.setCursor(0, 0);
         lcd.print("Enter Number:");
      }
    } 
    else if (key == '*') {  // '' clears the command sequence
      userInput = "";
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Enter Number:");
    }
  }
}