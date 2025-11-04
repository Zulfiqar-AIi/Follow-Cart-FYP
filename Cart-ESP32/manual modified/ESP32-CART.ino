#include <WiFi.h>
#include <WebServer.h>
#include <IRremote.h>
#include <BluetoothSerial.h>
#include <HX711.h> // For weight sensor

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
#define IR_RECEIVE_PIN 26    // Changed from 33 (to free SCK for HX711)
#define BUZZER_PIN 25

// HX711 Load Cell Pins
#define LOADCELL_DOUT_PIN 32
#define LOADCELL_SCK_PIN 33
HX711 scale;
float currentWeight = 0;     // Stores weight in kg
bool overloaded = false;     // Weight exceeds 1kg flag
const float WEIGHT_THRESHOLD = 1.0; // 1kg threshold

// IR Configuration
 //const unsigned int IR_EXPECTED_ADDRESS = 0x10AB;
unsigned long lastIRSignalTime = 0;
const unsigned long IR_TIMEOUT = 500;

// Operation Modes
#define MODE_AUTONOMOUS 0
#define MODE_MANUAL 1
#define MODE_OPENCV 2
#define MODE_HYBRID 3 // New hybrid mode

// System States
bool irSignalDetected = false;
bool powerOn = true;
int operationMode = MODE_AUTONOMOUS;
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
#define DEFAULT_FWD_SPEED 140
#define DEFAULT_TURN_SPEED 180
#define HYBRID_TURN_SPEED 80 // Lower speed for hybrid mode turns
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

// Function Prototypes
void handleBluetoothCommands();
void moveBackward(int speedLeft, int speedRight);
void emergencyStop();
void hybridNavigation(char command);

// Modified Distance Measurement with Sequential Triggering
long getSequentialDistance(int trigPin, int echoPin) {
  // Stop all other sensors first to prevent interference
  digitalWrite(TRIG_LEFT, LOW);
  digitalWrite(TRIG_CENTER, LOW);
  digitalWrite(TRIG_RIGHT, LOW);
  delayMicroseconds(2);
  
  // Trigger the specific sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echo with timeout (30ms = ~5m max range)
  long duration = pulseIn(echoPin, HIGH, 30000);
  
  // Validate reading
  if(duration <= 0) {
    return -1; // Invalid reading
  }
  return duration * 0.034 / 2;
}

// Movement Functions
void moveForward(int speedLeft, int speedRight) {
  if (overloaded) return; // Prevent movement if overloaded
  analogWrite(MOTOR_LEFT_PWM, speedLeft);
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  analogWrite(MOTOR_RIGHT_PWM, speedRight);
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
}

void moveBackward(int speedLeft, int speedRight) {
  if (overloaded) return; // Prevent movement if overloaded
  analogWrite(MOTOR_LEFT_PWM, speedLeft);
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  analogWrite(MOTOR_RIGHT_PWM, speedRight);
  digitalWrite(MOTOR_RIGHT_IN3, LOW);
  digitalWrite(MOTOR_RIGHT_IN4, HIGH);
}

void turnLeft() {
  if (overloaded) return; // Prevent movement if overloaded
  analogWrite(MOTOR_LEFT_PWM, turnSpeed);
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  analogWrite(MOTOR_RIGHT_PWM, forwardSpeed);
  digitalWrite(MOTOR_RIGHT_IN3, HIGH);
  digitalWrite(MOTOR_RIGHT_IN4, LOW);
}

void turnRight() {
  if (overloaded) return; // Prevent movement if overloaded
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

// Hybrid Navigation - Combines OpenCV commands with ultrasonic obstacle avoidance
void hybridNavigation(char command) {
  // First get all sensor readings
  long distanceLeft = getSequentialDistance(TRIG_LEFT, ECHO_LEFT);
  delay(10); // Reduced delay for faster response
  long distanceCenter = getSequentialDistance(TRIG_CENTER, ECHO_CENTER);
  delay(10);
  long distanceRight = getSequentialDistance(TRIG_RIGHT, ECHO_RIGHT);
  
  // Handle invalid readings by using previous values
  if(distanceLeft <= 0) distanceLeft = prevLeft;
  if(distanceCenter <= 0) distanceCenter = prevCenter;
  if(distanceRight <= 0) distanceRight = prevRight;
  
  // Store current readings for next iteration
  prevLeft = distanceLeft;
  prevCenter = distanceCenter;
  prevRight = distanceRight;

  // Check for obstacles in all directions
  bool obstacleLeft = distanceLeft <= 20;
  bool obstacleCenter = distanceCenter <= 20;
  bool obstacleRight = distanceRight <= 20;

  // If obstacle in all directions, stop regardless of command
  if (obstacleLeft && obstacleCenter && obstacleRight) {
    stopMotors();
    lastAction = "Obstacle All Directions";
    buzzerActive = !buzzerDisabled;
    return;
  }

  // Execute command with obstacle avoidance
  switch(command) {
    case 'F': // Forward command
      if (obstacleCenter) {
        // If center blocked but sides clear, turn slightly in safest direction
        if (!obstacleLeft && obstacleRight) {
          // Left is clear, turn slightly left
          analogWrite(MOTOR_LEFT_PWM, HYBRID_TURN_SPEED);
          digitalWrite(MOTOR_LEFT_IN1, LOW);
          digitalWrite(MOTOR_LEFT_IN2, HIGH);
          analogWrite(MOTOR_RIGHT_PWM, currentSpeed);
          digitalWrite(MOTOR_RIGHT_IN3, HIGH);
          digitalWrite(MOTOR_RIGHT_IN4, LOW);
          lastAction = "Forward (Adjusted Left)";
        } 
        else if (obstacleLeft && !obstacleRight) {
          // Right is clear, turn slightly right
          analogWrite(MOTOR_LEFT_PWM, currentSpeed);
          digitalWrite(MOTOR_LEFT_IN1, HIGH);
          digitalWrite(MOTOR_LEFT_IN2, LOW);
          analogWrite(MOTOR_RIGHT_PWM, HYBRID_TURN_SPEED);
          digitalWrite(MOTOR_RIGHT_IN3, LOW);
          digitalWrite(MOTOR_RIGHT_IN4, HIGH);
          lastAction = "Forward (Adjusted Right)";
        } 
        else if (!obstacleLeft && !obstacleRight) {
          // Both sides clear, choose to turn right (can change to random)
          analogWrite(MOTOR_LEFT_PWM, currentSpeed);
          digitalWrite(MOTOR_LEFT_IN1, HIGH);
          digitalWrite(MOTOR_LEFT_IN2, LOW);
          analogWrite(MOTOR_RIGHT_PWM, HYBRID_TURN_SPEED);
          digitalWrite(MOTOR_RIGHT_IN3, LOW);
          digitalWrite(MOTOR_RIGHT_IN4, HIGH);
          lastAction = "Forward (Adjusted Right)";
        } 
        else {
          // All directions blocked
          stopMotors();
          lastAction = "Cannot Move Forward";
        }
      } 
      else {
        // No center obstacle, move forward normally
        moveForward(currentSpeed, currentSpeed);
        lastAction = "Forward";
      }
      break;
      
    case 'B': // Backward command
      moveBackward(currentSpeed, currentSpeed);
      lastAction = "Backward";
      break;
      
    case 'L': // Left command
      if (obstacleLeft) {
        // If left blocked but right clear, turn right instead
        if (!obstacleRight) {
          turnRight();
          lastAction = "Right (Left Blocked)";
        } 
        else {
          stopMotors();
          lastAction = "Cannot Turn Left";
        }
      } 
      else {
        turnLeft();
        lastAction = "Left";
      }
      break;
      
    case 'R': // Right command
      if (obstacleRight) {
        // If right blocked but left clear, turn left instead
        if (!obstacleLeft) {
          turnLeft();
          lastAction = "Left (Right Blocked)";
        } 
        else {
          stopMotors();
          lastAction = "Cannot Turn Right";
        }
      } 
      else {
        turnRight();
        lastAction = "Right";
      }
      break;
      
    case 'S': // Stop command
      stopMotors();
      lastAction = "Stopped";
      break;
      
    case 'D': // Emergency Stop
      emergencyStop();
      lastAction = "Emergency Stop";
      break;
      
    default:
      // Unknown command - ignore
      break;
  }
}

// Web Interface Handler
void handleRoot() {
  // Get distances sequentially to prevent interference
  long currLeft = getSequentialDistance(TRIG_LEFT, ECHO_LEFT);
  delay(50);
  long currCenter = getSequentialDistance(TRIG_CENTER, ECHO_CENTER);
  delay(50);
  long currRight = getSequentialDistance(TRIG_RIGHT, ECHO_RIGHT);

  // Handle invalid readings by using previous values
  if(currLeft <= 0) currLeft = prevLeft;
  if(currCenter <= 0) currCenter = prevCenter;
  if(currRight <= 0) currRight = prevRight;
  
  // Store current readings for next iteration
  prevLeft = currLeft;
  prevCenter = currCenter;
  prevRight = currRight;

  String page = "<!DOCTYPE html><html><head><title>FollowCart: Smart Moving Companion</title>";
  page += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  page += "<meta http-equiv='refresh' content='1'>";
  page += "<style>";
  page += "body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 20px; background-color: #f5f5f5; }";
  page += ".container { max-width: 900px; margin: 0 auto; background: white; border-radius: 10px; padding: 20px; box-shadow: 0 0 15px rgba(0,0,0,0.1); }";
  page += "header { text-align: center; margin-bottom: 20px; padding-bottom: 20px; border-bottom: 1px solid #eee; }";
  page += "h1 { color: #2c3e50; margin-bottom: 5px; }";
  page += ".subtitle { color: #7f8c8d; font-size: 1.1em; margin-bottom: 15px; }";
  page += ".university { background-color: #3498db; color: white; padding: 5px 10px; border-radius: 5px; display: inline-block; margin-bottom: 15px; font-weight: bold; }";
  page += ".status-box { background: #f9f9f9; border-radius: 8px; padding: 15px; margin-bottom: 20px; }";
  page += ".grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 15px; margin-bottom: 20px; }";
  page += "button { width: 100%; padding: 12px; border: none; border-radius: 6px; font-size: 16px; cursor: pointer; transition: all 0.3s; margin: 5px 0; }";
  page += "input[type='range'] { width: 100%; margin: 10px 0; }";
  page += "form button { margin-top: 10px; }";
  page += ".active { background-color: #2ecc71; color: white; }";
  page += ".inactive { background-color: #ecf0f1; color: #7f8c8d; }";
  page += ".warning { background-color: #e67e22; color: white; }";
  page += ".danger { background-color: #e74c3c; color: white; }";
  page += "table { width: 100%; border-collapse: collapse; margin: 15px 0; }";
  page += "th, td { padding: 12px; text-align: left; border-bottom: 1px solid #ddd; }";
  page += "th { background-color: #f2f2f2; }";
  page += ".alert { background-color: #ffebee; padding: 10px; border-radius: 4px; margin: 10px 0; }";
  page += ".team-section { margin-top: 30px; background-color: #f8f9fa; padding: 15px; border-radius: 8px; }";
  page += ".team-title { color: #2c3e50; border-bottom: 1px solid #ddd; padding-bottom: 10px; }";
  page += ".team-member { margin: 5px 0; padding: 8px; background-color: white; border-radius: 5px; }";
  page += "@media (max-width: 600px) { .grid { grid-template-columns: 1fr; } }";
  page += "</style></head><body>";
  page += "<div class='container'>";
  
  // Header with title and university
  page += "<header>";
  page += "<h1>FollowCart: Smart Moving Companion</h1>";
  page += "<div class='university'>Dawood University of Engineering & Technology</div>";
  page += "</header>";
  
  // Status section
  page += "<div class='status-box'>";
  page += "<h2>Current Status</h2>";
  page += "<p><strong>Action:</strong> " + lastAction + "</p>";
  page += "<p><strong>Mode:</strong> ";
  switch(operationMode) {
    case MODE_AUTONOMOUS: page += "Autonomous"; break;
    case MODE_MANUAL: page += "Manual (Bluetooth)"; break;
    case MODE_OPENCV: page += "OpenCV based"; break;
    case MODE_HYBRID: page += "OpenCV + Ultrasonic Hybrid"; break;
  }
  page += "</p>";
  
  if(cartPickedUp) {
    page += "<div class='alert'><strong>⚠ Cart State:</strong> PICKED UP!</div>";
  } else {
    page += "<p><strong>Cart State:</strong> On Ground</p>";
  }
  
  page += "<p><strong>Buzzer:</strong> " + String(buzzerDisabled ? "🔕 DISABLED" : "🔔 ENABLED") + "</p>";
  page += "<p><strong>Power:</strong> " + String(powerOn ? "⚡ ON" : "⛔ OFF") + "</p>";
  page += "</div>";

  // Sensor readings
  page += "<div class='status-box'>";
  page += "<h2>Sensor Readings</h2>";
  page += "<table>";
  page += "<tr><th>Sensor</th><th>Distance (cm)</th></tr>";
  page += "<tr><td>Left</td><td>" + String(currLeft) + "</td></tr>";
  page += "<tr><td>Center</td><td>" + String(currCenter) + "</td></tr>";
  page += "<tr><td>Right</td><td>" + String(currRight) + "</td></tr>";
  page += "<tr><td>Weight</td><td>" + String(currentWeight, 1) + " kg</td></tr>";
  page += "</table>";

  // Overload alert
  if (overloaded) {
    page += "<div class='alert'><strong>⚠ OVERLOAD:</strong> Weight exceeds 1kg!</div>";
  }
  page += "</div>";

  // Control sections
  page += "<div class='grid'>";
  
  // Power control
  page += "<div class='status-box'>";
  page += "<h2>Power Control</h2>";
  page += "<a href='/power/on'><button class='" + String(powerOn ? "active" : "inactive") + "'>Turn ON</button></a>";
  page += "<a href='/power/off'><button class='" + String(!powerOn ? "danger" : "inactive") + "'>Turn OFF</button></a>";
  page += "</div>";

  // Mode selection
  page += "<div class='status-box'>";
  page += "<h2>Operation Mode</h2>";
  page += "<a href='/mode/auto'><button class='" + String(operationMode == MODE_AUTONOMOUS ? "active" : "inactive") + "'>Autonomous</button></a>";
  page += "<a href='/mode/manual'><button class='" + String(operationMode == MODE_MANUAL ? "active" : "inactive") + "'>Manual</button></a>";
  page += "<a href='/mode/opencv'><button class='" + String(operationMode == MODE_OPENCV ? "active" : "inactive") + "'>OpenCV</button></a>";
  page += "<a href='/mode/hybrid'><button class='" + String(operationMode == MODE_HYBRID ? "active" : "inactive") + "'>Hybrid Mode</button></a>";
  page += "</div>";

  // Speed control
  page += "<div class='status-box'>";
  page += "<h2>Speed Control</h2>";
  page += "<form action='/setSpeed' method='POST'>";
  page += "<p>Forward Speed: <span id='fwdValue'>" + String(forwardSpeed) + "</span></p>";
  page += "<input type='range' name='fwdSpeed' min='50' max='255' value='" + String(forwardSpeed) + "' oninput='document.getElementById(\"fwdValue\").innerHTML=this.value'>";
  page += "<p>Turn Speed: <span id='turnValue'>" + String(turnSpeed) + "</span></p>";
  page += "<input type='range' name='turnSpeed' min='30' max='200' value='" + String(turnSpeed) + "' oninput='document.getElementById(\"turnValue\").innerHTML=this.value'>";
  page += "<button type='submit'>Update Speeds</button>";
  page += "</form>";
  page += "</div>";

  // Buzzer control
  page += "<div class='status-box'>";
  page += "<h2>Buzzer Control</h2>";
  page += "<a href='/buzzer/on'><button class='" + String(!buzzerDisabled ? "active" : "inactive") + "'>Enable Buzzer</button></a>";
  page += "<a href='/buzzer/off'><button class='" + String(buzzerDisabled ? "warning" : "inactive") + "'>Disable Buzzer</button></a>";
  page += "</div>";
  page += "</div>"; // Close grid

  // Bluetooth info (if in manual/OpenCV/hybrid mode)
  if(operationMode == MODE_MANUAL || operationMode == MODE_OPENCV || operationMode == MODE_HYBRID) {
    page += "<div class='status-box'>";
    page += "<h2>Bluetooth Control</h2>";
    page += "<p>Device Name: <strong>" + String(bluetoothName) + "</strong></p>";
    page += "<p>Available Commands:</p>";
    page += "<ul>";
    page += "<li>F - Forward</li>";
    page += "<li>B - Backward</li>";
    page += "<li>L - Left</li>";
    page += "<li>R - Right</li>";
    page += "<li>S - Stop</li>";
    page += "<li>D - Emergency Stop</li>";
    page += "<li>0-9 - Speed (0-90%)</li>";
    page += "<li>q - Full Speed (100%)</li>";
    page += "</ul>";
    if (operationMode == MODE_HYBRID) {
      page += "<p><strong>Hybrid Mode:</strong> Commands will be adjusted based on obstacle detection</p>";
    }
    page += "</div>";
  }

  // Team members section
  page += "<div class='team-section'>";
  page += "<h3 class='team-title'>Development Team</h3>";
  page += "<div class='team-member'>M. Asadullah Sohail (M-21/F-BSCS-04)</div>";
  page += "<div class='team-member'>Muhammad Daniyal (M-21/F-BSCS-26)</div>";
  page += "<div class='team-member'>Zulfiqar Ali (M-21/F-BSCS-40)</div>";
  page += "<div class='team-member'>Hafiz Sameer Khan (M-21/F-BSCS-99)</div>";
  page += "</div>";

  page += "</div>"; // Close container
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

  // Initialize HX711
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(-500); // Calibration factor (adjust during testing)
  scale.tare(); // Reset to zero

  // Server endpoints
  server.on("/", handleRoot);
  server.on("/power/on", []() {
    powerOn = true;
    server.send(200, "text/plain", "Cart Powered ON");
  });
  server.on("/power/off", []() {
    powerOn = false;
    stopMotors();
    server.send(200, "text/plain", "Cart Powered OFF");
  });
  server.on("/mode/auto", []() {
    operationMode = MODE_AUTONOMOUS;
    stopMotors();
    server.send(200, "text/plain", "Autonomous Mode Activated");
  });
  server.on("/mode/manual", []() {
    operationMode = MODE_MANUAL;
    stopMotors();
    server.send(200, "text/plain", "Manual Mode Activated");
  });
  server.on("/mode/opencv", []() {
    operationMode = MODE_OPENCV;
    stopMotors();
    server.send(200, "text/plain", "OpenCV Mode Activated");
  });
  server.on("/mode/hybrid", []() {
    operationMode = MODE_HYBRID;
    stopMotors();
    server.send(200, "text/plain", "Hybrid Mode Activated");
  });
  server.on("/buzzer/on", []() {
    buzzerDisabled = false;
    server.send(200, "text/plain", "Buzzer Enabled");
  });
  server.on("/buzzer/off", []() {
    buzzerDisabled = true;
    digitalWrite(BUZZER_PIN, LOW);
    server.send(200, "text/plain", "Buzzer Disabled");
  });
  server.on("/setSpeed", HTTP_POST, []() {
    if (server.hasArg("fwdSpeed")) {
      forwardSpeed = server.arg("fwdSpeed").toInt();
      currentSpeed = forwardSpeed; // Update current speed
    }
    if (server.hasArg("turnSpeed")) {
      turnSpeed = server.arg("turnSpeed").toInt();
    }
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.begin();
  
  // Pin setups
  pinMode(IR_COLLISION_PIN, INPUT);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Ultrasonic sensors
  pinMode(TRIG_LEFT, OUTPUT); pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_CENTER, OUTPUT); pinMode(ECHO_CENTER, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  // Motor control pins
  pinMode(MOTOR_LEFT_IN1, OUTPUT); pinMode(MOTOR_LEFT_IN2, OUTPUT); pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT); pinMode(MOTOR_RIGHT_IN4, OUTPUT); pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  
  stopMotors();
}

void handleBluetoothCommands() {
  if (SerialBT.available()) {
    char command = SerialBT.read();
    Serial.print("Received BT command: ");
    Serial.println(command);
    
    switch(command) {
      case 'F': 
        if (operationMode == MODE_HYBRID) {
          hybridNavigation(command);
        } else {
          moveForward(currentSpeed, currentSpeed); 
          lastAction = "Forward";
        }
        break;
      case 'B': 
        if (operationMode == MODE_HYBRID) {
          hybridNavigation(command);
        } else {
          moveBackward(currentSpeed, currentSpeed); 
          lastAction = "Backward";
        }
        break;
      case 'L': 
        if (operationMode == MODE_HYBRID) {
          hybridNavigation(command);
        } else {
          turnLeft(); 
          lastAction = "Turning Left";
        }
        break;
      case 'R': 
        if (operationMode == MODE_HYBRID) {
          hybridNavigation(command);
        } else {
          turnRight(); 
          lastAction = "Turning Right";
        }
        break;
      case 'S': 
        stopMotors(); 
        lastAction = "Stopped"; 
        break;
      case 'D': 
        emergencyStop(); 
        lastAction = "Emergency Stop"; 
        break;
      case '0': case '1': case '2': case '3': case '4':
      case '5': case '6': case '7': case '8': case '9':
        currentSpeed = (command - '0') * 25.5;
        lastAction = "Speed " + String((command - '0') * 10) + "%";
        break;
      case 'q':
        currentSpeed = 255;
        lastAction = "Speed 100%";
        break;
      default:
        Serial.println("Unknown command");
    }
  }
}

void loop() {
  server.handleClient();

  // Read weight every 500ms - SKIP IN MANUAL/OPENCV/HYBRID MODES TO AVOID BLUETOOTH DELAYS
  static unsigned long lastWeightRead = 0;
  if (millis() - lastWeightRead > 500 && operationMode == MODE_AUTONOMOUS) {
    if (scale.wait_ready_timeout(50)) { // Reduced timeout from 200ms to 50ms
      currentWeight = scale.get_units(3); // Reduced from 5 to 3 readings for speed
      if (currentWeight < 0) currentWeight = 0; // Ignore negative values
      overloaded = (currentWeight > WEIGHT_THRESHOLD);
    }
    lastWeightRead = millis();
  }

  // Stop cart & activate buzzer if overloaded
  if (overloaded && powerOn) {
    emergencyStop();
    buzzerActive = !buzzerDisabled;
    lastAction = "Overloaded!";
  }

  // Pick-up detection (Autonomous mode only)
  if (powerOn && operationMode == MODE_AUTONOMOUS) {
    cartPickedUp = digitalRead(IR_COLLISION_PIN) == HIGH;
    if (cartPickedUp) {
      pickupDetectionTime = millis();
      buzzerActive = !buzzerDisabled;
      lastAction = "Cart Picked Up!";
    }
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

  // Handle Bluetooth in manual/OpenCV/hybrid modes
  if ((operationMode == MODE_MANUAL || operationMode == MODE_OPENCV || operationMode == MODE_HYBRID) && powerOn) {
    handleBluetoothCommands();
    delay(10); // Minimal delay for Bluetooth processing
    return; // Skip all sensor delays and autonomous logic
  }

  // Autonomous mode logic with sequential sensor reading - ONLY IN AUTONOMOUS MODE
  if (powerOn && operationMode == MODE_AUTONOMOUS && !overloaded) {
    // IR signal check
// With this (simplified version):
if (IrReceiver.decode()) {
  lastIRSignalTime = millis();
  irSignalDetected = true;
  IrReceiver.resume(); // Enable receiving the next IR signal
}

    // IR timeout check
    bool irTimedOut = (millis() - lastIRSignalTime) > IR_TIMEOUT;
    if (irTimedOut && irSignalDetected) {
      irSignalDetected = false;
      buzzerActive = !buzzerDisabled;
    }

    // Get distances sequentially with delays between readings - ONLY IN AUTONOMOUS MODE
    long distanceLeft = getSequentialDistance(TRIG_LEFT, ECHO_LEFT);
    delay(50); // Important delay between sensor readings
    long distanceCenter = getSequentialDistance(TRIG_CENTER, ECHO_CENTER);
    delay(50);
    long distanceRight = getSequentialDistance(TRIG_RIGHT, ECHO_RIGHT);

    // Handle invalid readings by using previous values
    if(distanceLeft <= 0) distanceLeft = prevLeft;
    if(distanceCenter <= 0) distanceCenter = prevCenter;
    if(distanceRight <= 0) distanceRight = prevRight;
    
    // Store current readings for next iteration
    prevLeft = distanceLeft;
    prevCenter = distanceCenter;
    prevRight = distanceRight;

    // Navigation logic
    if (irTimedOut || cartPickedUp) {
      stopMotors();
      lastAction = irTimedOut ? "No IR Signal" : "Cart Picked Up";
    } 
    else if (distanceLeft <= 20 || distanceCenter <= 20 || distanceRight <= 20) {
      stopMotors();
      lastAction = "Obstacle Detected";
      buzzerActive = !buzzerDisabled;
    } 
    else {
      buzzerActive = false;
      digitalWrite(BUZZER_PIN, LOW);
      
      if (distanceCenter < distanceLeft && distanceCenter < distanceRight) {
        moveForward(forwardSpeed, forwardSpeed);
        lastAction = "Moving Forward";
      } 
      else if (distanceLeft < distanceRight) {
        turnLeft();
        lastAction = "Turning Left";
      } 
      else {
        turnRight();
        lastAction = "Turning Right";
      }
    }
    
    delay(50); // Main loop delay only for autonomous mode
  }
}