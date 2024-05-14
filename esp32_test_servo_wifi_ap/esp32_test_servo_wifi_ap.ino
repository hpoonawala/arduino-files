// Control of a differential drive robot over wifi
// Continuous rotation servos
// Maps joystick axes to forward and angular speeds 
// press button when joystick not moved to set trim
// NOvember 12, 2023
// Hasan Poonawala
// Code snippets are based on existing webpages and examples in the package libraries
// Adding ESP Now

#include <esp_now.h>
#include <WiFi.h>

#define LED_BUILTIN 2

volatile int leftTick; // Stores the angle values
volatile int rightTick;
const byte interruptPinA = 4; // left wheel encoder signal out connects to this pin
const byte interruptPinB = 15; // right wheel

int leftSpeed; // In microseconds corresponding to PWM on cycle
int rightSpeed;
float forwardValZero =1.56;// Joystick reading when untouched
float turnValZero = 1.73; // Joystick reading when untouched
int forwardVal; // Joystick reading
int turnVal; // Joystick reading
const int delta = 120; // based on 16 PWMResolution
const int zeroSpeed = 5000;// based on 16 PWMResolution
const int PWMFreq = 50;
const int leftPWMChannel = 1;
const int rightPWMChannel = 2;
const int PWMResolution = 16;

int leftServoPin = 19;
int rightServoPin = 18;
int pos = 0;

void IRAM_ATTR pulseA () // left wheel encoder signal change interript service routine
{
  if (leftSpeed > 5001) //  true => positive angular velocity 
    leftTick+=1;
  if (leftSpeed < 4999) // 
    leftTick-=1;
}  // end of pulseA

void IRAM_ATTR pulseB () // right wheel encoder signal change interript service routine
{
  if (rightSpeed > 5001)//  true => positive angular velocity 
    rightTick+=1;
  if (rightSpeed < 4999)
    rightTick-=1;
}  // end of pulseB


// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {// From Rui Santos:https://randomnerdtutorials.com/esp-now-esp32-arduino-ide/
  float X;
  float Y;
  bool SW;
} struct_message;
// Create a struct_message called myData
struct_message myData;


uint8_t senderMacAddress[] = {0xD8, 0xBC, 0x38, 0xE5, 0xBF, 0x98};

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  digitalWrite(LED_BUILTIN, myData.SW);
  //Serial.println(myData.X);
  //Serial.println(myData.Y);
  rightSpeed = zeroSpeed + (int) (delta*2.0/3.3*(myData.X-forwardValZero))- (int) (delta*2.0/3.3*(myData.Y-turnValZero));
  leftSpeed = zeroSpeed + (int) (delta*2.0/3.3*(myData.X-forwardValZero))+ (int) (delta*2.0/3.3*(myData.Y-turnValZero));
  //if (rightSpeed > zeroSpeed+delta) {rightSpeed = zeroSpeed+delta;}
  //if (rightSpeed < zeroSpeed-delta) {rightSpeed = zeroSpeed-delta;}
  //if (leftSpeed > zeroSpeed+delta) {leftSpeed = zeroSpeed+delta;}
  //if (leftSpeed < zeroSpeed-delta) {leftSpeed = zeroSpeed-delta;}
  //Serial.println(rightSpeed);
  //Serial.println(leftSpeed);
  ledcWrite(rightPWMChannel, rightSpeed); 
  ledcWrite(leftPWMChannel, leftSpeed);
  if(myData.SW == false){
    forwardValZero = myData.X;
    turnValZero = myData.Y;
  }
}

esp_now_peer_info_t peerInfo;
void setup() {
  // put your setup code here, to run once:
  // Set LED pin, encoder pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // To ensure that incorrect readings not stored for zero vals
  pinMode(interruptPinA, INPUT_PULLUP);
  attachInterrupt(interruptPinA, pulseA, CHANGE);
  pinMode(interruptPinB, INPUT_PULLUP);
  attachInterrupt(interruptPinB, pulseB, CHANGE);
  Serial.begin(115200);
Serial.println(WiFi.macAddress());

  
  // Allow allocation of all timers for Servo conrol
  ledcSetup(rightPWMChannel, PWMFreq, PWMResolution);
  ledcSetup(leftPWMChannel, PWMFreq, PWMResolution);
  ledcAttachPin(leftServoPin, leftPWMChannel);
  ledcWrite(leftPWMChannel, zeroSpeed);
  ledcAttachPin(rightServoPin, rightPWMChannel);
  ledcWrite(rightPWMChannel, zeroSpeed);
  leftSpeed = zeroSpeed; 
  rightSpeed = zeroSpeed;
  forwardVal = forwardValZero;
  turnVal = turnValZero;
    // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  memcpy(peerInfo.peer_addr, senderMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}


void loop() {
}
