#include <Wire.h>
#include <Servo.h>
#include <Stepper.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

// Configuration namespace with improved organization
namespace Config {
    // System Constants
    constexpr unsigned long SERIAL_BAUD_RATE = 9600;
    constexpr unsigned long WATCHDOG_TIMEOUT_MS = 30000;  // Added watchdog
    
    // PWM and Motion Constants
    constexpr int MIN_PWM = 150;
    constexpr int MAX_PWM = 650;
    constexpr int SERVO_DELAY_MS = 40;
    constexpr int MOTION_SMOOTHNESS = 5;
    constexpr int MOVEMENT_DELAY_MS = 1000;
    constexpr int SERVO_COUNT = 6;
    constexpr int PWM_FREQUENCY = 60;
    
    // EEPROM Configuration
    constexpr int EEPROM_START_ADDRESS = 0;
    constexpr int EEPROM_ADDRESS_SPACING = 2;
    constexpr unsigned long EEPROM_WRITE_INTERVAL_MS = 5000;
    
    // Conveyor Belt Configuration
    constexpr int DETECTION_DISTANCE_MIN = 7;  // cm
    constexpr int DETECTION_DISTANCE_MAX = 45; // cm
    constexpr unsigned long CONVEYOR_TIMEOUT_MS = 30000;
    constexpr int CONVEYOR_DEFAULT_SPEED = 70;
    
    // Ultrasonic Sensor Configuration
    constexpr int TRIG_PIN = 9;
    constexpr int ECHO_PIN = 8;
    constexpr unsigned long SONAR_TIMEOUT_MICROS = 23529;
    constexpr float SOUND_SPEED_CM_MICROSEC = 0.034;
    constexpr int MAX_VALID_DISTANCE_CM = 100;
    constexpr unsigned long DISTANCE_CHECK_INTERVAL_MS = 100;
    
    // Stepper Motor Configuration
    constexpr int STEPPER_DIR_PIN = 6;
    constexpr int STEPPER_STEP_PIN = 5;
    constexpr int STEPS_PER_REV = 6400;
    constexpr unsigned long BASE_STEP_INTERVAL_MICROS = 800;
    constexpr unsigned long STEP_PULSE_WIDTH_MICROS = 10;
    constexpr unsigned long STEPS_PER_BATCH = 50;
    constexpr unsigned long BATCH_DELAY_MICROS = 500;
    
    // Error Codes
    enum class ErrorCode {
        NONE = 0,
        PWM_INIT_FAILED,
        INVALID_MOVEMENT,
        SENSOR_ERROR,
        CONVEYOR_TIMEOUT,
        INVALID_COMMAND
    };
    
    // System Status
    enum class SystemStatus {
        INITIALIZING,
        READY,
        ERROR,
        BUSY,
        WAITING_FOR_OBJECT,
        OBJECT_DETECTED
    };
    
    // Object Configurations
    enum class ObjectSize { SMALL, MEDIUM, LARGE };
    
    struct GripperConfig {
        static constexpr int OPEN_POSITION = 120;
        static constexpr int CLOSE_SMALL = 155;
        static constexpr int CLOSE_MEDIUM = 150;
        static constexpr int CLOSE_LARGE = 140;
        
        static int getGripPosition(ObjectSize size) {
            switch (size) {
                case ObjectSize::SMALL: return CLOSE_SMALL;
                case ObjectSize::MEDIUM: return CLOSE_MEDIUM;
                case ObjectSize::LARGE: return CLOSE_LARGE;
                default: return OPEN_POSITION;
            }
        }
    };
    
    struct PlacementConfig {
        static constexpr int SMALL_POSITION = 90;
        static constexpr int MEDIUM_POSITION = 50;
        static constexpr int LARGE_POSITION = 10;
        
        static int getPosition(ObjectSize size) {
            switch (size) {
                case ObjectSize::SMALL: return SMALL_POSITION;
                case ObjectSize::MEDIUM: return MEDIUM_POSITION;
                case ObjectSize::LARGE: return LARGE_POSITION;
                default: return MEDIUM_POSITION;
            }
        }
    };
}

// Improved distance sensor class with error handling
class DistanceSensor {
private:
    float lastValidReading = -1;
    unsigned long lastReadTime = 0;
    
    float readRawDistance() {
        digitalWrite(Config::TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(Config::TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(Config::TRIG_PIN, LOW);
        
        long duration = pulseIn(Config::ECHO_PIN, HIGH, Config::SONAR_TIMEOUT_MICROS);
        if (duration == 0) return -1;
        
        float distance = (duration * Config::SOUND_SPEED_CM_MICROSEC) / 2;
        return (distance > 0 && distance < Config::MAX_VALID_DISTANCE_CM) ? distance : -1;
    }
    
    float getFilteredReading() {
        constexpr int SAMPLES = 3;
        float total = 0;
        int validReadings = 0;
        
        for (int i = 0; i < SAMPLES; i++) {
            float reading = readRawDistance();
            if (reading > 0) {
                total += reading;
                validReadings++;
            }
            delayMicroseconds(1000);
        }
        
        return (validReadings > 0) ? (total / validReadings) : -1;
    }

public:
    void begin() {
        pinMode(Config::TRIG_PIN, OUTPUT);
        pinMode(Config::ECHO_PIN, INPUT);
    }
    
    float getDistance() {
        unsigned long currentTime = millis();
        if (currentTime - lastReadTime >= Config::DISTANCE_CHECK_INTERVAL_MS) {
            float reading = getFilteredReading();
            if (reading > 0) {
                lastValidReading = reading;
            }
            lastReadTime = currentTime;
        }
        return lastValidReading;
    }
};

// Improved conveyor system with better error handling
class ConveyorSystem {
private:
    DistanceSensor sensor;
    bool objectDetected = false;
    bool isMoving = false;
    unsigned long moveStartTime = 0;
    int currentSpeed = Config::CONVEYOR_DEFAULT_SPEED;
    unsigned long lastStepTime = 0;
    unsigned long stepCount = 0;
    
    void configureStepperPins() {
        pinMode(Config::STEPPER_DIR_PIN, OUTPUT);
        pinMode(Config::STEPPER_STEP_PIN, OUTPUT);
        digitalWrite(Config::STEPPER_DIR_PIN, LOW);
    }
    
    void stepMotor() {
        unsigned long currentMicros = micros();
        unsigned long stepInterval = Config::BASE_STEP_INTERVAL_MICROS * 
                                   (100 - currentSpeed) / 50;
        
        if (currentMicros - lastStepTime >= stepInterval) {
            digitalWrite(Config::STEPPER_STEP_PIN, HIGH);
            delayMicroseconds(Config::STEP_PULSE_WIDTH_MICROS);
            digitalWrite(Config::STEPPER_STEP_PIN, LOW);
            lastStepTime = currentMicros;
            
            stepCount++;
            if (stepCount >= Config::STEPS_PER_BATCH) {
                delayMicroseconds(Config::BATCH_DELAY_MICROS);
                stepCount = 0;
            }
        }
    }

public:
    void begin() {
        sensor.begin();
        configureStepperPins();
        reset();
    }
    
    void setSpeed(int speed) {
        currentSpeed = constrain(speed, 1, 100);
    }
    
    bool isObjectDetected() const {
        return objectDetected;
    }
    
    void reset() {
        objectDetected = false;
        isMoving = false;
        stepCount = 0;
        moveStartTime = 0;
    }
    
    Config::ErrorCode update() {
        float distance = sensor.getDistance();
        unsigned long currentTime = millis();
        
        // Check for timeout
        if (isMoving && (currentTime - moveStartTime > Config::CONVEYOR_TIMEOUT_MS)) {
            reset();
            return Config::ErrorCode::CONVEYOR_TIMEOUT;
        }
        
        // Invalid reading
        if (distance < 0) {
            return Config::ErrorCode::SENSOR_ERROR;
        }
        
        // Object detection logic
        if (distance <= Config::DETECTION_DISTANCE_MIN) {
            if (!objectDetected) {
                objectDetected = true;
                isMoving = false;
                Serial.println(F("Object detected in position"));
            }
        }
        else if (distance <= Config::DETECTION_DISTANCE_MAX) {
            if (!objectDetected && !isMoving) {
                isMoving = true;
                moveStartTime = currentTime;
                Serial.println(F("Conveyor started moving"));
            }
            if (isMoving) {
                stepMotor();
            }
        }
        else {
            if (objectDetected) {
                Serial.println(F("Object removed"));
                objectDetected = false;
            }
            isMoving = false;
        }
        
        return Config::ErrorCode::NONE;
    }
};

// Improved movement coordination
struct MovementStep {
    uint8_t servoIndex;
    uint8_t position;
    
    MovementStep(uint8_t idx, uint8_t pos) : servoIndex(idx), position(pos) {}
};

class PickPlaceSequence {
public:
    static MovementStep* createSequence(Config::ObjectSize size, size_t& length) {
        int gripperPosition = Config::GripperConfig::getGripPosition(size);
        int placementPosition = Config::PlacementConfig::getPosition(size);
        
        static MovementStep sequence[] = {
            {2, 150}, // Raise arm
            {0, 170}, // Move to conveyor
            {5, 120}, // Open gripper
            {1, 85},  // Lower for pick
            {3, 10},  // Adjust wrist
            {4, 90},  // Align claw
            {5, 0},   // Close gripper (updated dynamically)
            {1, 50},  // Lift object
            {0, 0},   // Rotate to placement (updated dynamically)
            {3, 30},  // Orient for placement
            {2, 140}, // Extend arm
            {1, 100}, // Lower arm
            {4, 90},  // Final alignment
            {5, 120}, // Release object
            {1, 50}   // Return to safe height
        };
        
        sequence[6].position = gripperPosition;
        sequence[8].position = placementPosition;
        
        length = sizeof(sequence) / sizeof(MovementStep);
        return sequence;
    }
};

// Main robotic arm class with improved error handling and state management
class RoboticArm {
private:
    Adafruit_PWMServoDriver pwm;
    ConveyorSystem conveyor;
    Config::SystemStatus status = Config::SystemStatus::INITIALIZING;
    Config::ErrorCode lastError = Config::ErrorCode::NONE;
    
    const int pinPositions[Config::SERVO_COUNT] = {8, 2, 3, 5, 6, 7};
    const int movementOrder[Config::SERVO_COUNT] = {1, 2, 0, 3, 4, 5};
    int currentPosition[Config::SERVO_COUNT];
    const int homePosition[Config::SERVO_COUNT] = {90, 20, 180, 10, 90, 120};
    
    unsigned long lastEEPROMWrite = 0;
    unsigned long lastWatchdogReset = 0;
    
    void updateWatchdog() {
        lastWatchdogReset = millis();
    }
    
    bool checkWatchdog() {
        return (millis() - lastWatchdogReset) < Config::WATCHDOG_TIMEOUT_MS;
    }
    
    int angleToPWM(int angle) const {
        return map(angle, 0, 180, Config::MIN_PWM, Config::MAX_PWM);
    }
    
    bool isValidPosition(int position, bool isGripper = false) const {
        if (isGripper) {
            return position == Config::GripperConfig::OPEN_POSITION ||
                   position == Config::GripperConfig::CLOSE_SMALL ||
                   position == Config::GripperConfig::CLOSE_MEDIUM ||
                   position == Config::GripperConfig::CLOSE_LARGE;
        }
        return position >= 0 && position <= 180;
    }
    
    void processCommand(char cmd) {
        switch (cmd) {
            case '0':
                Serial.println(F("Moving to home position"));
                moveToPosition(homePosition);
                break;
                
            case '1':
            case '2':
            case '3': {
                if (!conveyor.isObjectDetected()) {
                    Serial.println(F("No object detected"));
                    break;
                }
                
                Config::ObjectSize size;
                switch (cmd) {
                    case '1': size = Config::ObjectSize::SMALL; break;
                    case '2': size = Config::ObjectSize::MEDIUM; break;
                    case '3': size = Config::ObjectSize::LARGE; break;
                }
                
                executePickPlaceSequence(size);
                break;
            }
            
            default:
                Serial.println(F("Invalid command. Available commands:"));
                Serial.println(F("0: Home position"));
                Serial.println(F("1: Pick/place small object"));
                Serial.println(F("2: Pick/place medium object"));
                Serial.println(F("3: Pick/place large object"));
                break;
        }
    }

public:
    RoboticArm() : pwm() {
        memcpy(currentPosition, homePosition, sizeof(homePosition));
    }
    
    bool begin() {
        Serial.begin(Config::SERIAL_BAUD_RATE);
        Serial.println(F("Initializing Robotic Arm..."));
        
        if (!pwm.begin()) {
            status = Config::SystemStatus::ERROR;
            lastError = Config::ErrorCode::PWM_INIT_FAILED;
            return false;
        }
        
        pwm.setPWMFreq(Config::PWM_FREQUENCY);
        conveyor.begin();
        loadPositionsFromEEPROM();
        moveToPosition(homePosition);
        
        status = Config::SystemStatus::READY;
        updateWatchdog();
        return true;
    }
    
    void process() {
        if (status == Config::SystemStatus::ERROR) {
            handleError();
            return;
        }
        
        if (!checkWatchdog()) {
            status = Config::SystemStatus::ERROR;
            return;
        }
        
        // Update conveyor system
        Config::ErrorCode conveyorError = conveyor.update();
        if (conveyorError != Config::ErrorCode::NONE) {
            handleError(conveyorError);
            return;
        }
        
        // Process any pending serial commands
        if (Serial.available() > 0) {
            char cmd = Serial.read();
            processCommand(cmd);
            // Clear any remaining bytes in serial buffer
            while (Serial.available()) Serial.read();
        }
        
        updateWatchdog();
    }

private:
    void handleError(Config::ErrorCode error = Config::ErrorCode::NONE) {
        lastError = error;
        status = Config::SystemStatus::ERROR;
        
        Serial.print(F("Error detected: "));
        switch (error) {
            case Config::ErrorCode::PWM_INIT_FAILED:
                Serial.println(F("PWM initialization failed"));
                break;
            case Config::ErrorCode::INVALID_MOVEMENT:
                Serial.println(F("Invalid movement detected"));
                break;
            case Config::ErrorCode::SENSOR_ERROR:
                Serial.println(F("Distance sensor error"));
                break;
            case Config::ErrorCode::CONVEYOR_TIMEOUT:
                Serial.println(F("Conveyor movement timeout"));
                break;
            default:
                Serial.println(F("Unknown error"));
                break;
        }
        
        // Try to recover by moving to home position
        moveToPosition(homePosition);
        status = Config::SystemStatus::READY;
    }
    
    void loadPositionsFromEEPROM() {
        for (int i = 0; i < Config::SERVO_COUNT; i++) {
            int position;
            int address = Config::EEPROM_START_ADDRESS + (i * Config::EEPROM_ADDRESS_SPACING);
            EEPROM.get(address, position);
            
            if (i == 5) { // Gripper servo
                currentPosition[i] = isValidPosition(position, true) ? 
                    position : Config::GripperConfig::OPEN_POSITION;
            } else {
                currentPosition[i] = isValidPosition(position) ? 
                    position : homePosition[i];
            }
        }
        Serial.println(F("Positions loaded from EEPROM"));
    }
    
    void savePositionToEEPROM(int servoIndex) {
        unsigned long currentTime = millis();
        if (currentTime - lastEEPROMWrite < Config::EEPROM_WRITE_INTERVAL_MS) {
            return;
        }
        
        int address = Config::EEPROM_START_ADDRESS + (servoIndex * Config::EEPROM_ADDRESS_SPACING);
        EEPROM.put(address, currentPosition[servoIndex]);
        lastEEPROMWrite = currentTime;
    }
    
    bool moveServoSmooth(int servoIndex, int targetAngle) {
        if (!isValidPosition(targetAngle, servoIndex == 5)) {
            Serial.println(F("Invalid position requested"));
            return false;
        }

        int currentPWM = angleToPWM(currentPosition[servoIndex]);
        int targetPWM = angleToPWM(targetAngle);
        int step = (targetPWM > currentPWM) ? Config::MOTION_SMOOTHNESS : -Config::MOTION_SMOOTHNESS;

        while ((step > 0 && currentPWM <= targetPWM) || 
               (step < 0 && currentPWM >= targetPWM)) {
            pwm.setPWM(pinPositions[servoIndex], 0, currentPWM);
            delay(Config::SERVO_DELAY_MS);
            currentPWM += step;
        }

        // Ensure final position is exact
        pwm.setPWM(pinPositions[servoIndex], 0, targetPWM);
        currentPosition[servoIndex] = targetAngle;
        savePositionToEEPROM(servoIndex);
        return true;
    }
    
    void moveToPosition(const int targetPosition[]) {
        for (int i = 0; i < Config::SERVO_COUNT; i++) {
            int servoIndex = movementOrder[i];
            if (!moveServoSmooth(servoIndex, targetPosition[servoIndex])) {
                handleError(Config::ErrorCode::INVALID_MOVEMENT);
                return;
            }
            delay(Config::MOVEMENT_DELAY_MS);
        }
    }
    
    bool executePickPlace(const MovementStep sequence[], size_t sequenceLength) {
        Serial.println(F("Executing pick and place sequence"));
        status = Config::SystemStatus::BUSY;
        
        for (size_t i = 0; i < sequenceLength; i++) {
            const MovementStep& step = sequence[i];
            
            if (step.servoIndex >= Config::SERVO_COUNT) {
                handleError(Config::ErrorCode::INVALID_MOVEMENT);
                return false;
            }
            
            if (!moveServoSmooth(step.servoIndex, step.position)) {
                handleError(Config::ErrorCode::INVALID_MOVEMENT);
                return false;
            }
            
            delay(Config::MOVEMENT_DELAY_MS);
            
            Serial.print(F("Step "));
            Serial.print(i + 1);
            Serial.print(F("/"));
            Serial.println(sequenceLength);
        }
        
        delay(500); // Short delay before returning home
        moveToPosition(homePosition);
        status = Config::SystemStatus::READY; 
        return true;
    }
    
    void executePickPlaceSequence(Config::ObjectSize size) {
    size_t sequenceLength;
    MovementStep* sequence = PickPlaceSequence::createSequence(size, sequenceLength);
    
    if (executePickPlace(sequence, sequenceLength)) {
        conveyor.reset();
        Serial.println(F("Pick and place sequence completed")); // Add this line
    }
}

public:
    Config::SystemStatus getStatus() const {
        return status;
    }
    
    Config::ErrorCode getLastError() const {
        return lastError;
    }
    
    void resetError() {
        lastError = Config::ErrorCode::NONE;
        status = Config::SystemStatus::READY;
    }
};


// Global instance of RoboticArm
RoboticArm arm;

void setup() {
    // Initialize the robotic arm
    if (!arm.begin()) {
        // If initialization fails, enter error loop
        while (1) {
            delay(1000);  // Wait in error state
        }
    }
}

void loop() {
    // Process robotic arm operations
    arm.process();
}
    