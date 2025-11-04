#include <WiFi.h>
#include <WebServer.h>
#include <IRremote.h>
#include <BluetoothSerial.h>

// Bluetooth Setup
BluetoothSerial SerialBT;

// Sensor Pins
#define TRIG_LEFT 5
#define ECHO_LEFT 18
#define TRIG_CENTER 19
#define ECHO_CENTER 21
#define TRIG_RIGHT 22
#define ECHO_RIGHT 23
#define IR_COLLISION_PIN 27  // Pick-up detection
#define IR_RECEIVE_PIN 26
#define BUZZER_PIN 25

// IR Configuration
const unsigned int IR_EXPECTED_ADDRESS = 0x10AB;
unsigned long lastIRSignalTime = 0;
const unsigned long IR_TIMEOUT = 5000;

// Operation Modes
#define MODE_HYBRID 0  // Primary mode now
#define MODE_MANUAL 1
#define MODE_OPENCV 2

// System States
bool irSignalDetected = false;
bool powerOn = true;
int operationMode = MODE_HYBRID;  // Default to hybrid mode
bool cartPickedUp = false;
bool buzzerDisabled = false;
unsigned long pickupDetectionTime = 0;
const unsigned long PICKUP_BUZZER_TIMEOUT = 3000;

// Motor Pins & Settings
#define MOTOR_LEFT_IN1 13
#define MOTOR_LEFT_IN2 14
#define MOTOR_LEFT_PWM 12
#define MOTOR_RIGHT_IN3 16
#define MOTOR_RIGHT_IN4 17
#define MOTOR_RIGHT_PWM 15
#define DEFAULT_FWD_SPEED 160
#define DEFAULT_TURN_SPEED 190
#define HYBRID_TURN_SPEED 80
int forwardSpeed = DEFAULT_FWD_SPEED;
int turnSpeed = DEFAULT_TURN_SPEED;
int currentSpeed = DEFAULT_FWD_SPEED;

// Web Server Configuration
const char* ssid = "CartNetwork";
const char* password = "cartpassword";
const char* bluetoothName = "ESP32_Cart";
WebServer server(80);
IPAddress local_IP(192, 168, 10, 1);
IPAddress gateway(192, 168, 10, 1);
IPAddress subnet(255, 255, 255, 0);

// System Variables
long prevLeft = 0, prevCenter = 0, prevRight = 0;
String lastAction = "Idle";
unsigned long lastBuzzToggle = 0;
bool buzzerState = false;
bool buzzerActive = false;
unsigned long lastSensorSend = 0;

// Movement Functions
void moveForward(int speedLeft, int speedRight) {
  analogWrite(MOTOR_LEFT_PWM, speedLeft);
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  analogWrite(MOTOR_RIGHT_PWM, speedRight);
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
}

void moveBackward(int speedLeft, int speedRight) {
  analogWrite(MOTOR_LEFT_PWM, speedLeft);
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  analogWrite(MOTOR_RIGHT_PWM, speedRight);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, HIGH);
}

void turnLeft() {
  analogWrite(MOTOR_LEFT_PWM, turnSpeed);
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  analogWrite(MOTOR_RIGHT_PWM, forwardSpeed);
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
}

void turnRight() {
  analogWrite(MOTOR_LEFT_PWM, forwardSpeed);
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  analogWrite(MOTOR_RIGHT_PWM, turnSpeed);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, HIGH);
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
}

void emergencyStop() {
  stopMotors();
  currentSpeed = 0;
  lastAction = "Emergency Stop";
}

// Enhanced Distance Measurement
long getSequentialDistance(int trigPin, int echoPin) {
  digitalWrite(TRIG_LEFT, LOW);
  digitalWrite(TRIG_CENTER, LOW);
  digitalWrite(TRIG_RIGHT, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 30000);
  return duration <= 0 ? -1 : duration * 0.034 / 2;
}

// Send sensor data via Bluetooth
void sendSensorData() {
  long distanceLeft = getSequentialDistance(TRIG_LEFT, ECHO_LEFT);
  delay(10);
  long distanceCenter = getSequentialDistance(TRIG_CENTER, ECHO_CENTER);
  delay(10);
  long distanceRight = getSequentialDistance(TRIG_RIGHT, ECHO_RIGHT);

  // Handle invalid readings
  if(distanceLeft <= 0) distanceLeft = prevLeft;
  if(distanceCenter <= 0) distanceCenter = prevCenter;
  if(distanceRight <= 0) distanceRight = prevRight;
  
  prevLeft = distanceLeft;
  prevCenter = distanceCenter;
  prevRight = distanceRight;

  String sensorData = "US:" + String(distanceLeft) + "," + 
                     String(distanceCenter) + "," + 
                     String(distanceRight);
  SerialBT.println(sensorData);
}

// Hybrid Mode Command Execution
void executeHybridCommand(char command) {
  switch(command) {
    case 'F': moveForward(currentSpeed, currentSpeed); break;
    case 'B': moveBackward(currentSpeed, currentSpeed); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    case 'S': stopMotors(); break;
    case 'D': emergencyStop(); break;
    default: break;
  }
  lastAction = String("Hybrid: ") + command;
}

// Web Interface Handler (simplified)
void handleRoot() {
  String page = "<!DOCTYPE html><html><head><title>FollowCart</title>";
  page += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  page += "<style>body{font-family:Arial,sans-serif;margin:20px}</style>";
  page += "</head><body><h1>FollowCart Control</h1>";
  
  page += "<h2>Current Mode: ";
  switch(operationMode) {
    case MODE_HYBRID: page += "Hybrid"; break;
    case MODE_MANUAL: page += "Manual"; break;
    case MODE_OPENCV: page += "OpenCV"; break;
  }
  page += "</h2>";
  
  page += "<h3>Mode Selection:</h3>";
  page += "<a href='/mode/hybrid'><button>Hybrid</button></a>";
  page += "<a href='/mode/manual'><button>Manual</button></a>";
  page += "<a href='/mode/opencv'><button>OpenCV</button></a>";
  
  page += "</body></html>";
  server.send(200, "text/html", page);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize Bluetooth
  SerialBT.begin(bluetoothName);
  Serial.println("Bluetooth Started! Name: " + String(bluetoothName));

  // Configure Access Point
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);
  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Server endpoints
  server.on("/", handleRoot);
  server.on("/mode/hybrid", []() {
    operationMode = MODE_HYBRID;
    server.send(200, "text/plain", "Hybrid Mode Activated");
  });
  server.on("/mode/manual", []() {
    operationMode = MODE_MANUAL;
    server.send(200, "text/plain", "Manual Mode Activated");
  });
  server.on("/mode/opencv", []() {
    operationMode = MODE_OPENCV;
    server.send(200, "text/plain", "OpenCV Mode Activated");
  });

  server.begin();
  
  // Pin setups
  pinMode(IR_COLLISION_PIN, INPUT);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  pinMode(BUZZER_PIN, OUTPUT);

  // Ultrasonic sensors
  pinMode(TRIG_LEFT, OUTPUT); pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_CENTER, OUTPUT); pinMode(ECHO_CENTER, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  // Motor control pins
  pinMode(MOTOR_LEFT_IN1, OUTPUT); pinMode(MOTOR_LEFT_IN2, OUTPUT); 
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT); pinMode(MOTOR_RIGHT_IN4, OUTPUT); 
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  
  stopMotors();
}

void loop() {
  server.handleClient();

  // In hybrid mode, continuously send sensor data
  if (operationMode == MODE_HYBRID && millis() - lastSensorSend > 200) {
    sendSensorData();
    lastSensorSend = millis();
  }

  // Handle Bluetooth commands
  if (SerialBT.available()) {
    char command = SerialBT.read();
    Serial.print("Received BT command: ");
    Serial.println(command);
    
    if (operationMode == MODE_HYBRID) {
      executeHybridCommand(command);
    } else {
      // Handle manual/OpenCV commands
      switch(command) {
        case 'F': moveForward(currentSpeed, currentSpeed); break;
        case 'B': moveBackward(currentSpeed, currentSpeed); break;
        case 'L': turnLeft(); break;
        case 'R': turnRight(); break;
        case 'S': stopMotors(); break;
        case 'D': emergencyStop(); break;
        case '0'...'9': 
          currentSpeed = (command - '0') * 25.5;
          break;
        case 'q': currentSpeed = 255; break;
      }
    }
  }

  // Pick-up detection
  if (powerOn && digitalRead(IR_COLLISION_PIN) {
    cartPickedUp = true;
    pickupDetectionTime = millis();
    buzzerActive = !buzzerDisabled;
    lastAction = "Cart Picked Up!";
    stopMotors();
  }

  // Buzzer control
  if (buzzerActive && !buzzerDisabled && powerOn) {
    if (millis() - lastBuzzToggle > 500) {
      buzzerState = !buzzerState;
      digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
      lastBuzzToggle = millis();
    }
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  delay(10); // Main loop delay
}