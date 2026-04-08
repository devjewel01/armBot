// Ultrasonic sensor test code
const int TRIG_PIN = 9;  // Connect trigger pin to Arduino pin 9
const int ECHO_PIN = 8;  // Connect echo pin to Arduino pin 8

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  pinMode(TRIG_PIN, OUTPUT);  // Set trigger pin as output
  pinMode(ECHO_PIN, INPUT);   // Set echo pin as input
  
  Serial.println("Ultrasonic Sensor Test");
  Serial.println("--------------------");
  Serial.println("Distance measurements will be printed every 500ms");
}

void loop() {
  // Clear trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send 10 microsecond pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the echo pin, pulseIn() returns the duration in microseconds
  long duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate distance in centimeters
  // Speed of sound = 343m/s = 34300cm/s
  // Duration is round trip, so divide by 2
  // Distance = (Time × Speed) ÷ 2
  float distance = duration * 0.034 / 2;
  
  // Print the distance
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // Add status indication for typical detection range
  if (distance < 7) {
    Serial.println("Status: Object very close (pickup position)");
  } else if (distance <= 45) {
    Serial.println("Status: Object in detection range");
  } else {
    Serial.println("Status: No object in range");
  }
  
  Serial.println("--------------------");
  delay(500);  // Wait half second before next reading
}