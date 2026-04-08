#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <iostream>
#include <sstream>

// Add forward declarations for setup() and loop()
void setup();
void loop();

// Initialize PWM servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo configuration
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz
#define SERVOMIN 125   // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 575   // This is the 'maximum' pulse length count (out of 4096)
#define MOVE_DELAY 20  // Delay between servo movements during playback

struct ServoPins {
  uint8_t channel;  // PWM channel on PCA9685
  String servoName;
  int initialPosition;
  int currentPosition;  // Track current position
};

std::vector<ServoPins> servoPins = {
  { 0, "Base", 90, 90 },       // Channel 0: Base - 90 degrees
  { 1, "Shoulder", 50, 50 },   // Channel 1: Shoulder - 20 degrees
  { 2, "Elbow", 40, 40 },      // Channel 2: Elbow - 180 degrees
  { 3, "Wrist", 10, 100 },     // Channel 3: Wrist - 10 degrees
  { 4, "WristRoll", 90, 90 },  // Channel 4: WristRoll - 90 degrees
  { 5, "Gripper", 90, 90 }     // Channel 5: Gripper - 90 degrees
};

struct RecordedStep {
  std::vector<int> positions;  // Store all servo positions
  unsigned long timestamp;     // When this step was recorded
};
std::vector<RecordedStep> recordedSteps;

bool recordSteps = false;
bool playRecordedSteps = false;
unsigned long recordStartTime = 0;
unsigned long lastPlaybackTime = 0;

const char *ssid = "Robotics Arm";
const char *password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket wsRobotArmInput("/RobotArmInput");

const char *htmlHomePage PROGMEM = R"HTMLHOMEPAGE(
<!DOCTYPE html>
<html>
  <head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <style>
      body {
        margin: 0;
        padding: 15px;
        background-color: #f0f0f0;
        font-family: Arial, sans-serif;
      }
      
      .container {
        max-width: 800px;
        margin: 0 auto;
      }
      
      h1 {
        color: #2c3e50;
        text-align: center;
        font-size: 24px;
        margin: 10px 0 20px;
      }
      
      .control-table {
        width: 100%;
        border-collapse: collapse;
      }
      
      .control-table td {
        padding: 8px 4px;
        vertical-align: middle;
      }
      
      .control-label {
        text-align: left;
        font-size: 16px;
        font-weight: bold;
        color: #34495e;
        width: 100px;
      }
      
      .control-slider {
        width: 100%;
      }
      
      input[type=button] {
        background-color: #e74c3c;
        color: white;
        border: none;
        border-radius: 20px;
        padding: 8px 15px;
        width: 100%;
        height: 40px;
        font-size: 16px;
        cursor: pointer;
        transition: background-color 0.3s;
      }
      
      input[type=button]:active {
        background-color: #c0392b;
      }
      
      .slider {
        -webkit-appearance: none;
        width: 100%;
        height: 15px;
        border-radius: 10px;
        background: #bdc3c7;
        outline: none;
        opacity: 0.7;
        transition: opacity 0.2s;
      }
      
      .slider:hover {
        opacity: 1;
      }
      
      .slider::-webkit-slider-thumb {
        -webkit-appearance: none;
        appearance: none;
        width: 25px;
        height: 25px;
        border-radius: 50%;
        background: #e74c3c;
        cursor: pointer;
      }
      
      .slider::-moz-range-thumb {
        width: 25px;
        height: 25px;
        border-radius: 50%;
        background: #e74c3c;
        cursor: pointer;
      }
      
      .button-row {
        margin-top: 15px;
      }
      
      .noselect {
        -webkit-touch-callout: none;
        -webkit-user-select: none;
        -khtml-user-select: none;
        -moz-user-select: none;
        -ms-user-select: none;
        user-select: none;
      }
      
      /* Responsive adjustments */
      @media screen and (max-width: 480px) {
        body {
          padding: 10px;
        }
        
        h1 {
          font-size: 20px;
        }
        
        .control-label {
          font-size: 14px;
          width: 80px;
        }
        
        input[type=button] {
          height: 35px;
          font-size: 14px;
        }
        
        .slider::-webkit-slider-thumb {
          width: 20px;
          height: 20px;
        }
        
        .slider::-moz-range-thumb {
          width: 20px;
          height: 20px;
        }
      }
    </style>
  </head>
<body class="noselect">
    <div class="container">
      <h1>6 DOF Robot Arm Control</h1>
      
      <table class="control-table">
        <tr>
          <td class="control-label">Gripper:</td>
          <td class="control-slider">
            <input type="range" min="0" max="180" value="90" class="slider" id="Gripper" oninput='sendButtonInput("Gripper",value)'>
          </td>
        </tr>
        <tr>
          <td class="control-label">Wrist Roll:</td>
          <td class="control-slider">
            <input type="range" min="0" max="180" value="90" class="slider" id="WristRoll" oninput='sendButtonInput("WristRoll",value)'>
          </td>
        </tr>
        <tr>
          <td class="control-label">Wrist:</td>
          <td class="control-slider">
            <input type="range" min="0" max="180" value="10" class="slider" id="Wrist" oninput='sendButtonInput("Wrist",value)'>
          </td>
        </tr>
        <tr>
          <td class="control-label">Elbow:</td>
          <td class="control-slider">
            <input type="range" min="0" max="180" value="180" class="slider" id="Elbow" oninput='sendButtonInput("Elbow",value)'>
          </td>
        </tr>
        <tr>
          <td class="control-label">Shoulder:</td>
          <td class="control-slider">
            <input type="range" min="0" max="180" value="20" class="slider" id="Shoulder" oninput='sendButtonInput("Shoulder",value)'>
          </td>
        </tr>
        <tr>
          <td class="control-label">Base:</td>
          <td class="control-slider">
            <input type="range" min="0" max="180" value="90" class="slider" id="Base" oninput='sendButtonInput("Base",value)'>
          </td>
        </tr>
      </table>

      
      <div class="button-row">
        <table class="control-table">
          <tr>
            <td class="control-label">Record:</td>
            <td><input type="button" id="Record" value="OFF" ontouchend='onclickButton(this)'></td>
          </tr>
          <tr>
            <td class="control-label">Play:</td>
            <td><input type="button" id="Play" value="OFF" ontouchend='onclickButton(this)'></td>
          </tr>
        </table>
      </div>
    </div>
  
    <script>
      var webSocketRobotArmInputUrl = "ws:\/\/" + window.location.hostname + "/RobotArmInput";      
      var websocketRobotArmInput;
      
      function initRobotArmInputWebSocket() 
      {
        websocketRobotArmInput = new WebSocket(webSocketRobotArmInputUrl);
        websocketRobotArmInput.onopen    = function(event){};
        websocketRobotArmInput.onclose   = function(event){setTimeout(initRobotArmInputWebSocket, 2000);};
        websocketRobotArmInput.onmessage = function(event)
        {
          var keyValue = event.data.split(",");
          var button = document.getElementById(keyValue[0]);
          button.value = keyValue[1];
          if (button.id == "Record" || button.id == "Play")
          {
            button.style.backgroundColor = (button.value == "ON" ? "#27ae60" : "#e74c3c");  
            enableDisableButtonsSliders(button);
          }
        };
      }
      
      function sendButtonInput(key, value) 
      {
        var data = key + "," + value;
        websocketRobotArmInput.send(data);
      }
      
      function onclickButton(button) 
      {
        button.value = (button.value == "ON") ? "OFF" : "ON" ;        
        button.style.backgroundColor = (button.value == "ON" ? "#27ae60" : "#e74c3c");          
        var value = (button.value == "ON") ? 1 : 0 ;
        sendButtonInput(button.id, value);
        enableDisableButtonsSliders(button);
      }
      
      function enableDisableButtonsSliders(button)
      {
        if(button.id == "Play")
        {
          var disabled = "auto";
          if (button.value == "ON")
          {
            disabled = "none";            
          }
          document.getElementById("Gripper").style.pointerEvents = disabled;
          document.getElementById("WristRoll").style.pointerEvents = disabled;
          document.getElementById("Wrist").style.pointerEvents = disabled;
          document.getElementById("Elbow").style.pointerEvents = disabled;          
          document.getElementById("Shoulder").style.pointerEvents = disabled;          
          document.getElementById("Base").style.pointerEvents = disabled; 
          document.getElementById("Record").style.pointerEvents = disabled;
        }
        if(button.id == "Record")
        {
          var disabled = "auto";
          if (button.value == "ON")
          {
            disabled = "none";            
          }
          document.getElementById("Play").style.pointerEvents = disabled;
        }        
      }
           
      window.onload = initRobotArmInputWebSocket;
      document.getElementById("control-table").addEventListener("touchend", function(event){
        event.preventDefault()
      });      
    </script>
  </body>    
</html>
)HTMLHOMEPAGE";


// Add a function to move to home position
void moveToHome() {
  // Move each servo to home position in sequence
  writeServoValues(0, 90);  // Base
  delay(500);               // Small delay between movements
  writeServoValues(1, 20);  // Shoulder
  delay(500);
  writeServoValues(2, 180);  // Elbow
  delay(500);
  writeServoValues(3, 10);  // Wrist
  delay(500);
  writeServoValues(4, 90);  // WristRoll
  delay(500);
  writeServoValues(5, 90);  // Gripper
}

int angleToPulse(int angle) {
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}

void writeServoValues(int servoIndex, int angle) {
  if (servoIndex >= 0 && servoIndex < servoPins.size()) {
    // Update current position and write to PWM driver
    servoPins[servoIndex].currentPosition = angle;
    pwm.setPWM(servoPins[servoIndex].channel, 0, angleToPulse(angle));

    // Record the step if recording is active
    if (recordSteps) {
      RecordedStep step;
      step.timestamp = millis() - recordStartTime;

      // Store current positions of all servos
      for (const auto &servo : servoPins) {
        step.positions.push_back(servo.currentPosition);
      }
      recordedSteps.push_back(step);
    }
  }
}

void playRecordedRobotArmSteps() {
  static size_t currentStep = 0;

  if (recordedSteps.empty() || !playRecordedSteps) {
    playRecordedSteps = false;
    currentStep = 0;
    return;
  }

  unsigned long currentTime = millis() - lastPlaybackTime;

  // Check if it's time to play the next step
  if (currentStep < recordedSteps.size() && currentTime >= recordedSteps[currentStep].timestamp) {
    // Move all servos to their recorded positions
    for (size_t i = 0; i < servoPins.size(); i++) {
      int targetPosition = recordedSteps[currentStep].positions[i];
      if (servoPins[i].currentPosition != targetPosition) {
        pwm.setPWM(servoPins[i].channel, 0, angleToPulse(targetPosition));
        servoPins[i].currentPosition = targetPosition;
        wsRobotArmInput.textAll(servoPins[i].servoName + "," + String(targetPosition));
      }
    }

    currentStep++;

    // If we've reached the end, reset playback
    if (currentStep >= recordedSteps.size()) {
      playRecordedSteps = false;
      currentStep = 0;
      wsRobotArmInput.textAll("Play,OFF");
    }
  }
}

void setUpPinModes() {
  // Initialize I2C
  Wire.begin(26, 25);  // SDA = GPIO 26, SCL = GPIO 25 for ESP32

  // Initialize PWM driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  // Initialize all servos to their initial positions
  for (int i = 0; i < servoPins.size(); i++) {
    pwm.setPWM(servoPins[i].channel, 0, angleToPulse(servoPins[i].initialPosition));
    delay(50);  // Short delay between servo initializations
  }
}

void sendCurrentRobotArmState() {
  for (int i = 0; i < servoPins.size(); i++) {
    wsRobotArmInput.textAll(servoPins[i].servoName + "," + servoPins[i].currentPosition);
  }
  wsRobotArmInput.textAll(String("Record,") + (recordSteps ? "ON" : "OFF"));
  wsRobotArmInput.textAll(String("Play,") + (playRecordedSteps ? "ON" : "OFF"));
}

void onRobotArmInputWebSocketEvent(AsyncWebSocket *server,
                                   AsyncWebSocketClient *client,
                                   AwsEventType type,
                                   void *arg,
                                   uint8_t *data,
                                   size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      // Send current state to newly connected client
      sendCurrentRobotArmState();
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      // Optional: You could stop any ongoing operations if needed when client disconnects
      break;

    case WS_EVT_DATA:
      {  // Create a scope for local variables
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
          // Create a null-terminated string from the received data
          std::string myData = "";
          myData.assign((char *)data, len);
          std::istringstream ss(myData);
          std::string key, value;
          std::getline(ss, key, ',');
          std::getline(ss, value, ',');

          // Log received command
          Serial.printf("Key [%s] Value[%s]\n", key.c_str(), value.c_str());
          int valueInt = atoi(value.c_str());

          // Process commands
          if (key == "Record") {
            recordSteps = valueInt;
            if (recordSteps) {
              recordedSteps.clear();
              recordStartTime = millis();
              Serial.println("Started recording");
            } else {
              Serial.println("Stopped recording");
            }
            // Broadcast recording state to all clients
            wsRobotArmInput.textAll(String("Record,") + (recordSteps ? "ON" : "OFF"));
          } else if (key == "Play") {
            playRecordedSteps = valueInt;
            if (playRecordedSteps) {
              if (recordedSteps.empty()) {
                playRecordedSteps = false;
                wsRobotArmInput.textAll("Play,OFF");
                Serial.println("No steps recorded, can't play");
              } else {
                lastPlaybackTime = millis();
                Serial.println("Started playback");
              }
            } else {
              Serial.println("Stopped playback");
            }
            // Broadcast playback state to all clients
            wsRobotArmInput.textAll(String("Play,") + (playRecordedSteps ? "ON" : "OFF"));
          }
          // Servo control commands
          else if (key == "Base") {
            writeServoValues(0, valueInt);
            wsRobotArmInput.textAll("Base," + String(valueInt));
          } else if (key == "Shoulder") {
            writeServoValues(1, valueInt);
            wsRobotArmInput.textAll("Shoulder," + String(valueInt));
          } else if (key == "Elbow") {
            writeServoValues(2, valueInt);
            wsRobotArmInput.textAll("Elbow," + String(valueInt));
          } else if (key == "Wrist") {
            writeServoValues(3, valueInt);
            wsRobotArmInput.textAll("Wrist," + String(valueInt));
          } else if (key == "WristRoll") {
            writeServoValues(4, valueInt);
            wsRobotArmInput.textAll("WristRoll," + String(valueInt));
          } else if (key == "Gripper") {
            writeServoValues(5, valueInt);
            wsRobotArmInput.textAll("Gripper," + String(valueInt));
          } else {
            Serial.printf("Unknown command: %s\n", key.c_str());
          }
        } else if (info->opcode == WS_BINARY) {
          Serial.println("Binary data received - not supported");
        }
      }
      break;

    case WS_EVT_PONG:
      Serial.println("Pong received");
      break;

    case WS_EVT_ERROR:
      Serial.printf("WebSocket error #%u from client %u\n", *((uint16_t *)arg), client->id());
      break;

    default:
      Serial.printf("Unhandled WebSocket event type: %d\n", type);
      break;
  }
}

void handleRoot(AsyncWebServerRequest *request) {
  request->send_P(200, "text/html", htmlHomePage);
}

void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "File Not Found");
}

// Required setup() function - this is where your original setup code goes
void setup() {
  Serial.begin(115200);
  setUpPinModes();

  // Move to home position at startup
  moveToHome();

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.on("/", HTTP_GET, handleRoot);
  server.onNotFound(handleNotFound);

  wsRobotArmInput.onEvent(onRobotArmInputWebSocketEvent);
  server.addHandler(&wsRobotArmInput);

  server.begin();
  Serial.println("HTTP server started");
}

// Required loop() function - this is where your original loop code goes
void loop() {
  wsRobotArmInput.cleanupClients();
  if (playRecordedSteps) {
    playRecordedRobotArmSteps();
  }
  delay(MOVE_DELAY);  // Add small delay to prevent overwhelming the servos
}