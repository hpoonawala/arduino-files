// Want to use ESP Now instead of wifi

#include <esp_now.h>
#include <WiFi.h>

#define LED_BUILTIN 2
#define ADCPIN_X 32
#define ADCPIN_Y 33
#define SW_PIN 15

int adcValue_X;
float voltValue_X;
int adcValue_Y;
float voltValue_Y;
bool SWpress = LOW;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {// From Rui Santos:https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/
  float X;
  float Y;
  bool SW;
} struct_message;
// Create a struct_message called myData
struct_message myData;

uint8_t receiverMacAddress[] = {0xEC, 0x94, 0xCB, 0x6D, 0x48, 0x90};
// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {// From Rui Santos:
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
// Callback function that will be called when data is received
void onDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
  Serial.println("Data received!");
  Serial.print("From: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", mac[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
 
  Serial.print("Data: ");
  for (int i = 0; i < len; i++) {
    Serial.printf("%02X ", data[i]);
  }
  Serial.println();
}
  // Add the receiver's MAC address to the peer list
  esp_now_peer_info_t peerInfo;
void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);
Serial.println(WiFi.macAddress());
 WiFi.mode(WIFI_STA);
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed");
    return;
  }

  // Register callback function to handle received data
  esp_now_register_recv_cb(onDataReceived);
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SW_PIN, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN, SWpress);

}

void loop() {
  adcValue_X = analogRead(ADCPIN_X);
  myData.X = ((adcValue_Y * 3.3) / 4095);
  adcValue_Y = analogRead(ADCPIN_Y);
  myData.Y = 3.3 - ((adcValue_X * 3.3) / 4095);
  myData.SW = digitalRead(SW_PIN);
  digitalWrite(LED_BUILTIN, myData.SW);
  Serial.println(myData.X);
  Serial.println(myData.Y);
  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
delay(50);
}
