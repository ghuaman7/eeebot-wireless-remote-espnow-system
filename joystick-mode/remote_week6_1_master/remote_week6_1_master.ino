#include <WiFi.h>
#include <esp_now.h>

// —— ESP‑NOW Broadcast Address (or replace with your car’s MAC) ——
uint8_t broadcastAddress[] = { 0x08,0x3A,0x8D,0x0D,0x91,0x98 };

// —— Data packet structure (must match the slave) ——
typedef struct {
  int16_t xAxis;
  int16_t yAxis;
} struct_message;
struct_message valuesToSend;

// —— Callbacks ——
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// —— Smoothing & Mapping Parameters ——
const int numberOfSamples = 10;
int recentX[numberOfSamples], recentY[numberOfSamples];
int idx = 0;
long totalX = 0, totalY = 0;
int smoothedX, smoothedY;
int16_t mappedX, mappedY;  // final –255…+255

// —— Joystick Pins ——
const int XPin = 34;  // VRX
const int YPin = 39;  // VRY

void setup() {
  Serial.begin(115200);
  Serial.println("Master starting...");

  // Zero‐fill smoothing buffers
  for (int i = 0; i < numberOfSamples; i++) {
    recentX[i] = recentY[i] = 0;
  }

  // Init Wi‑Fi & ESP‑NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP‑NOW init failed");
    while (true) { delay(1000); }
  }
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (true) { delay(1000); }
  }

}

void loop() {
  // 1) Remove oldest sample
  totalX -= recentX[idx];
  totalY -= recentY[idx];

  // 2) Read & store new sample
  int rawX = analogRead(XPin);
  int rawY = analogRead(YPin);
  recentX[idx] = rawX;
  recentY[idx] = rawY;

  // 3) Add to totals
  totalX += rawX;
  totalY += rawY;

  // 4) Advance buffer index
  if (++idx >= numberOfSamples) idx = 0;

  // 5) Compute smoothed values
  smoothedX = totalX / numberOfSamples;
  smoothedY = totalY / numberOfSamples;

  // 6) Map around measured centres (1930/1880)
  mappedX = constrain(((smoothedX - 1930L) * 255) / 1930, -255, 255);
  mappedY = constrain(((smoothedY - 1880L) * 255) / 1880, -255, 255);

  // 7) Send mapped values
  valuesToSend.xAxis = mappedX;
  valuesToSend.yAxis = mappedY;
  esp_err_t result = esp_now_send(broadcastAddress,
                                  (uint8_t *)&valuesToSend,
                                  sizeof(valuesToSend));
  if (result != ESP_OK) {
    Serial.println("Error sending");
  }

  // 8) Print only the mapped values
  Serial.printf("Mapped→ X:%4d | Y:%4d\n", mappedX, mappedY);
  delay(100);
}
  int xValue = analogRead(XPin);
  int yValue = analogRead(YPin); 

