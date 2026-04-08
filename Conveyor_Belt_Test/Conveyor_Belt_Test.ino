// TB6600 Stepper Driver Test Code
const int STEP_PIN = 5;  // Connect STEP pin to Arduino pin 5
const int DIR_PIN = 6;   // Connect DIR pin to Arduino pin 6

// Motor configuration
const int STEPS_PER_REV = 6400;  // Steps per revolution (32 microsteps * 200 steps)
const int STEP_DELAY = 800;      // Microseconds between steps (controls speed)

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Configure motor pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  Serial.println("TB6600 Stepper Motor Test");
  Serial.println("------------------------");
  Serial.println("Commands:");
  Serial.println("1: Rotate clockwise 1 revolution");
  Serial.println("2: Rotate counter-clockwise 1 revolution");
  Serial.println("3: Continuous run clockwise");
  Serial.println("4: Continuous run counter-clockwise");
  Serial.println("5: Stop motor");
  Serial.println("6: Test conveyor movement (short forward bursts)");
}

void rotateMotor(bool clockwise, int steps) {
  // Set direction
  digitalWrite(DIR_PIN, clockwise ? LOW : HIGH);
  
  // Rotate the specified number of steps
  for(int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY);
  }
}

boolean isRunning = false;
boolean currentDirection = true;  // true for clockwise

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch(command) {
      case '1':
        Serial.println("Rotating clockwise 1 revolution...");
        isRunning = false;
        rotateMotor(true, STEPS_PER_REV);
        Serial.println("Revolution complete");
        break;
        
      case '2':
        Serial.println("Rotating counter-clockwise 1 revolution...");
        isRunning = false;
        rotateMotor(false, STEPS_PER_REV);
        Serial.println("Revolution complete");
        break;
        
      case '3':
        Serial.println("Starting continuous clockwise rotation...");
        isRunning = true;
        currentDirection = true;
        break;
        
      case '4':
        Serial.println("Starting continuous counter-clockwise rotation...");
        isRunning = true;
        currentDirection = false;
        break;
        
      case '5':
        Serial.println("Stopping motor...");
        isRunning = false;
        break;
        
      case '6':
        Serial.println("Testing conveyor movement (short bursts)...");
        testConveyorMovement();
        break;
        
      default:
        Serial.println("Invalid command!");
        break;
    }
  }
  
  // Handle continuous rotation if enabled
  if (isRunning) {
    digitalWrite(DIR_PIN, currentDirection ? LOW : HIGH);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY);
  }
}

void testConveyorMovement() {
  Serial.println("Moving conveyor in bursts...");
  
  for(int burst = 0; burst < 5; burst++) {
    Serial.print("Burst ");
    Serial.println(burst + 1);
    
    // Move for a short duration
    for(int steps = 0; steps < STEPS_PER_REV/4; steps++) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(STEP_DELAY);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(STEP_DELAY);
    }
    
    // Pause between bursts
    delay(1000);
  }
  
  Serial.println("Conveyor test complete");
}