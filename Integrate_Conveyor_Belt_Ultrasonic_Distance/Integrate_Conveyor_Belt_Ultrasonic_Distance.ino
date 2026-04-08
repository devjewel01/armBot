// Conveyor Belt and Ultrasonic Sensor Combined Test
// Ultrasonic Sensor Pins
const int TRIG_PIN = 9;
const int ECHO_PIN = 8;

// TB6600 Stepper Driver Pins
const int STEP_PIN = 5;
const int DIR_PIN = 6;

// Configuration
const int STEPS_PER_REV = 6400;      // 32 microsteps * 200 steps
const int STEP_DELAY = 800;          // Microseconds between steps
const float MIN_DISTANCE = 7.0;      // Minimum distance for object detection (cm)
const float MAX_DISTANCE = 45.0;     // Maximum distance for object detection (cm)

// System States
enum ConveyorState {
  WAITING_FOR_OBJECT,
  MOVING_OBJECT,
  OBJECT_IN_POSITION,
  STOPPED
};

ConveyorState currentState = WAITING_FOR_OBJECT;
unsigned long lastDistanceCheck = 0;
const int DISTANCE_CHECK_INTERVAL = 100; // Check distance every 100ms

void setup() {
  Serial.begin(9600);
  
  // Configure ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Configure stepper motor pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW); // Set initial direction
  
  printMenu();
}

void printMenu() {
  Serial.println("\n=== Conveyor Belt and Distance Sensor Test ===");
  Serial.println("Commands:");
  Serial.println("1: Start automatic mode (move when object detected)");
  Serial.println("2: Stop conveyor");
  Serial.println("3: Move conveyor forward for 1 second");
  Serial.println("4: Print current distance");
  Serial.println("5: Print this menu");
  Serial.println("==========================================\n");
}

float measureDistance() {
  // Clear trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send trigger pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate distance
  float distance = duration * 0.034 / 2;
  
  return distance;
}

void moveConveyor(int numSteps) {
  for(int i = 0; i < numSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY);
  }
}

void printSystemStatus(float distance) {
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm | State: ");
  
  switch(currentState) {
    case WAITING_FOR_OBJECT:
      Serial.println("Waiting for object");
      break;
    case MOVING_OBJECT:
      Serial.println("Moving object");
      break;
    case OBJECT_IN_POSITION:
      Serial.println("Object in position");
      break;
    case STOPPED:
      Serial.println("Stopped");
      break;
  }
}

void handleAutomaticMode() {
  float distance = measureDistance();
  
  // Only print status if distance has changed significantly (>0.5cm)
  static float lastPrintedDistance = -1;
  if (abs(distance - lastPrintedDistance) > 0.5) {
    printSystemStatus(distance);
    lastPrintedDistance = distance;
  }
  
  switch(currentState) {
    case WAITING_FOR_OBJECT:
      if (distance <= MAX_DISTANCE && distance > MIN_DISTANCE) {
        currentState = MOVING_OBJECT;
        Serial.println("Object detected - Starting conveyor");
      }
      break;
      
    case MOVING_OBJECT:
      if (distance <= MIN_DISTANCE) {
        currentState = OBJECT_IN_POSITION;
        Serial.println("Object reached target position - Stopping conveyor");
      }
      else if (distance > MAX_DISTANCE) {
        currentState = WAITING_FOR_OBJECT;
        Serial.println("Object lost - Stopping conveyor");
      }
      else {
        moveConveyor(100); // Move a small amount
      }
      break;
      
    case OBJECT_IN_POSITION:
      if (distance > MIN_DISTANCE) {
        currentState = WAITING_FOR_OBJECT;
        Serial.println("Object removed - Ready for next object");
      }
      break;
  }
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch(command) {
      case '1':
        Serial.println("Starting automatic mode");
        currentState = WAITING_FOR_OBJECT;
        break;
        
      case '2':
        Serial.println("Stopping conveyor");
        currentState = STOPPED;
        break;
        
      case '3':
        Serial.println("Moving conveyor for 1 second");
        for(int i = 0; i < 1000000/STEP_DELAY/2; i++) {
          moveConveyor(1);
        }
        Serial.println("Movement complete");
        break;
        
      case '4': {
        float distance = measureDistance();
        printSystemStatus(distance);
        break;
      }
      
      case '5':
        printMenu();
        break;
        
      default:
        Serial.println("Invalid command!");
        printMenu();
        break;
    }
    
    // Clear any remaining characters in serial buffer
    while(Serial.available()) {
      Serial.read();
    }
  }

  // Handle automatic mode if active
  if (currentState != STOPPED && millis() - lastDistanceCheck >= DISTANCE_CHECK_INTERVAL) {
    handleAutomaticMode();
    lastDistanceCheck = millis();
  }
}