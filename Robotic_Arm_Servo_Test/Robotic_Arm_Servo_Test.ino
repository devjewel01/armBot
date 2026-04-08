#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PWM driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo configuration
const int SERVO_FREQ = 60;  // Analog servos run at ~60 Hz
const int MIN_PWM = 150;    // Minimum PWM value (0 degrees)
const int MAX_PWM = 650;    // Maximum PWM value (180 degrees)
const int SERVO_DELAY = 40; // Delay between servo movements

// Servo pin assignments on PWM driver
const int BASE_SERVO = 8;      // Base rotation
const int SHOULDER_SERVO = 2;  // Shoulder joint
const int ELBOW_SERVO = 3;     // Elbow joint
const int WRIST_PITCH = 5;     // Wrist up/down
const int WRIST_ROLL = 6;      // Wrist rotation
const int GRIPPER_SERVO = 7;   // Gripper

// Default positions (adjust as needed)
int currentPositions[] = {90, 90, 90, 90, 90, 120};
const int HOME_POSITIONS[] = {90, 20, 180, 10, 90, 120};

void setup() {
  Serial.begin(9600);
  Serial.println("Robotic Arm Servo Test");
  
  // Initialize PWM driver
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  
  delay(10);
  
  printMenu();
}

void printMenu() {
  Serial.println("\n=== Robotic Arm Servo Test ===");
  Serial.println("Commands:");
  Serial.println("1: Test individual servo");
  Serial.println("2: Move to home position");
  Serial.println("3: Test gripper open/close");
  Serial.println("4: Test picking motion");
  Serial.println("5: Test basic sequence");
  Serial.println("6: Print current positions");
  Serial.println("7: Show this menu");
  Serial.println("============================\n");
}

// Convert angle to PWM value
int angleToPWM(int angle) {
  return map(angle, 0, 180, MIN_PWM, MAX_PWM);
}

// Move servo smoothly
void moveServo(int servoPin, int targetAngle) {
  int currentAngle = currentPositions[getServoIndex(servoPin)];
  
  Serial.print("Moving servo ");
  Serial.print(getServoName(servoPin));
  Serial.print(" from ");
  Serial.print(currentAngle);
  Serial.print(" to ");
  Serial.println(targetAngle);
  
  // Determine direction and step
  int step = (targetAngle > currentAngle) ? 1 : -1;
  
  // Move servo gradually
  while(currentAngle != targetAngle) {
    currentAngle += step;
    pwm.setPWM(servoPin, 0, angleToPWM(currentAngle));
    delay(SERVO_DELAY);
  }
  
  // Update current position
  currentPositions[getServoIndex(servoPin)] = targetAngle;
}

int getServoIndex(int servoPin) {
  switch(servoPin) {
    case BASE_SERVO: return 0;
    case SHOULDER_SERVO: return 1;
    case ELBOW_SERVO: return 2;
    case WRIST_PITCH: return 3;
    case WRIST_ROLL: return 4;
    case GRIPPER_SERVO: return 5;
    default: return 0;
  }
}

String getServoName(int servoPin) {
  switch(servoPin) {
    case BASE_SERVO: return "Base";
    case SHOULDER_SERVO: return "Shoulder";
    case ELBOW_SERVO: return "Elbow";
    case WRIST_PITCH: return "Wrist Pitch";
    case WRIST_ROLL: return "Wrist Roll";
    case GRIPPER_SERVO: return "Gripper";
    default: return "Unknown";
  }
}

void testIndividualServo() {
  Serial.println("\nSelect servo to test:");
  Serial.println("1: Base");
  Serial.println("2: Shoulder");
  Serial.println("3: Elbow");
  Serial.println("4: Wrist Pitch");
  Serial.println("5: Wrist Roll");
  Serial.println("6: Gripper");
  
  while(!Serial.available()) {}
  int servo = Serial.read() - '0';
  
  // Clear remaining characters
  while(Serial.available()) Serial.read();
  
  if(servo < 1 || servo > 6) {
    Serial.println("Invalid servo selection!");
    return;
  }
  
  int servoPin;
  switch(servo) {
    case 1: servoPin = BASE_SERVO; break;
    case 2: servoPin = SHOULDER_SERVO; break;
    case 3: servoPin = ELBOW_SERVO; break;
    case 4: servoPin = WRIST_PITCH; break;
    case 5: servoPin = WRIST_ROLL; break;
    case 6: servoPin = GRIPPER_SERVO; break;
    default: return;
  }
  
  Serial.println("Enter target angle (0-180):");
  while(!Serial.available()) {}
  
  String input = Serial.readStringUntil('\n');
  int angle = input.toInt();
  
  if(angle >= 0 && angle <= 180) {
    moveServo(servoPin, angle);
  } else {
    Serial.println("Invalid angle!");
  }
}

void moveToHome() {
  Serial.println("Moving to home position...");
  
  // Move servos in sequence
  moveServo(GRIPPER_SERVO, HOME_POSITIONS[5]);   // Open gripper first
  moveServo(WRIST_ROLL, HOME_POSITIONS[4]);      // Level wrist
  moveServo(WRIST_PITCH, HOME_POSITIONS[3]);     // Level wrist pitch
  moveServo(ELBOW_SERVO, HOME_POSITIONS[2]);     // Raise elbow
  moveServo(SHOULDER_SERVO, HOME_POSITIONS[1]);  // Lower shoulder
  moveServo(BASE_SERVO, HOME_POSITIONS[0]);      // Center base
  
  Serial.println("Home position reached");
}

void testGripper() {
  Serial.println("Testing gripper...");
  
  // Open gripper
  Serial.println("Opening gripper");
  moveServo(GRIPPER_SERVO, 120);
  delay(1000);
  
  // Close gripper
  Serial.println("Closing gripper");
  moveServo(GRIPPER_SERVO, 155);
  delay(1000);
  
  // Open gripper again
  Serial.println("Opening gripper");
  moveServo(GRIPPER_SERVO, 120);
}

void testPickingMotion() {
  Serial.println("Testing picking motion...");
  
  // Start from home
  moveToHome();
  delay(1000);
  
  // Move to object position
  moveServo(BASE_SERVO, 170);      // Rotate to target
  moveServo(SHOULDER_SERVO, 85);   // Lower arm
  moveServo(ELBOW_SERVO, 90);      // Extend arm
  moveServo(WRIST_PITCH, 10);      // Align gripper
  moveServo(WRIST_ROLL, 90);       // Level gripper
  
  // Close gripper
  moveServo(GRIPPER_SERVO, 155);
  delay(1000);
  
  // Lift object
  moveServo(SHOULDER_SERVO, 45);
  moveServo(ELBOW_SERVO, 130);
  
  // Return to home
  moveToHome();
}

void testBasicSequence() {
  Serial.println("Running basic test sequence...");
  
  // Move to home position
  moveToHome();
  delay(1000);
  
  // Swing base left and right
  Serial.println("Testing base rotation...");
  moveServo(BASE_SERVO, 45);
  delay(500);
  moveServo(BASE_SERVO, 135);
  delay(500);
  moveServo(BASE_SERVO, 90);
  delay(500);
  
  // Test shoulder movement
  Serial.println("Testing shoulder...");
  moveServo(SHOULDER_SERVO, 45);
  delay(500);
  moveServo(SHOULDER_SERVO, 20);
  delay(500);
  
  // Test elbow movement
  Serial.println("Testing elbow...");
  moveServo(ELBOW_SERVO, 135);
  delay(500);
  moveServo(ELBOW_SERVO, 180);
  delay(500);
  
  // Test wrist movements
  Serial.println("Testing wrist...");
  moveServo(WRIST_PITCH, 45);
  moveServo(WRIST_ROLL, 45);
  delay(500);
  moveServo(WRIST_PITCH, 10);
  moveServo(WRIST_ROLL, 90);
  delay(500);
  
  // Test gripper
  testGripper();
  
  // Return to home
  moveToHome();
}

void printCurrentPositions() {
  Serial.println("\nCurrent servo positions:");
  Serial.print("Base: "); Serial.println(currentPositions[0]);
  Serial.print("Shoulder: "); Serial.println(currentPositions[1]);
  Serial.print("Elbow: "); Serial.println(currentPositions[2]);
  Serial.print("Wrist Pitch: "); Serial.println(currentPositions[3]);
  Serial.print("Wrist Roll: "); Serial.println(currentPositions[4]);
  Serial.print("Gripper: "); Serial.println(currentPositions[5]);
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch(command) {
      case '1':
        testIndividualServo();
        break;
        
      case '2':
        moveToHome();
        break;
        
      case '3':
        testGripper();
        break;
        
      case '4':
        testPickingMotion();
        break;
        
      case '5':
        testBasicSequence();
        break;
        
      case '6':
        printCurrentPositions();
        break;
        
      case '7':
        printMenu();
        break;
        
      default:
        Serial.println("Invalid command!");
        printMenu();
        break;
    }
    
    // Clear any remaining characters
    while(Serial.available()) Serial.read();
  }
}