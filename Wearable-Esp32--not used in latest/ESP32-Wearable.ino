#include <IRremote.h>
#include <WiFi.h>

const int kIrLedPin1 = 15;         // GPIO for first IR LED
const int kIrLedPin2 = 27;         // GPIO for second IR LED
const uint16_t kIrAddress = 0x10AB; // Fixed unique address

// WiFi credentials - replace with your cart's network info
const char* ssid = "CartNetwork";
const char* password = "cartpassword";

void setup() {
  Serial.begin(115200);
  
  // Initialize both IR senders
  IrSender.begin(kIrLedPin1);  // Initialize first IR LED
  // Note: The IRremote library typically only supports one transmitter at a time
  // So we'll need to switch between them or use a different approach
  
  Serial.println("IR Transmitters Ready");
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

unsigned long lastSendTime = 0;
unsigned int signalID = 0;

void loop() {
  unsigned long currentTime = millis();
  
  // Send IR signal every 500ms
  if (currentTime - lastSendTime >= 200) {
    signalID++;
    uint16_t command = signalID & 0xFFFF;  // 16-bit incrementing value
    
    // Send from first IR LED
    IrSender.begin(kIrLedPin1);  // Switch to first LED
    IrSender.sendNEC(kIrAddress, command, 0);
    
    // Send from second IR LED
    IrSender.begin(kIrLedPin2);  // Switch to second LED
    IrSender.sendNEC(kIrAddress, command, 0);
    
    Serial.print("Sending IR NEC code with address: 0x");
    Serial.print(kIrAddress, HEX);
    Serial.print(", command: 0x");
    Serial.println(command, HEX);
    
    lastSendTime = currentTime;
  }
  
  // Check WiFi connection periodically
  if (currentTime % 5000 == 0 && WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, attempting to reconnect...");
    WiFi.reconnect();
  }
}
