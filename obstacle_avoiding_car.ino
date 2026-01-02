/*
 * ====================================================================
 * OBSTACLE AVOIDING CAR - WITHOUT SERVO MOTOR
 * ====================================================================
 * Hardware: Arduino Uno R3, HC-SR04, L298N Motor Driver, DC Motors
 * 
 * CIRCUIT CONNECTIONS:
 * ====================================================================
 * 
 * HC-SR04 Ultrasonic Sensor:
 * ---------------------------
 * VCC  -> Arduino 5V
 * GND  -> Arduino GND
 * TRIG -> Arduino Pin 9
 * ECHO -> Arduino Pin 10
 * 
 * L298N Motor Driver:
 * ---------------------------
 * Motor Driver Power:
 * - 12V Power Supply -> 12V terminal on L298N
 * - GND -> GND terminal on L298N
 * - 5V Output from L298N -> Arduino 5V (if jumper is on)
 *   OR use separate power for Arduino
 * 
 * Motor Driver Control Pins:
 * - ENA (Enable A) -> Arduino Pin 5 (PWM for speed control)
 * - IN1 -> Arduino Pin 6
 * - IN2 -> Arduino Pin 7
 * - IN3 -> Arduino Pin 8
 * - IN4 -> Arduino Pin 11
 * - ENB (Enable B) -> Arduino Pin 3 (PWM for speed control)
 * 
 * Motor Driver Outputs:
 * - OUT1 & OUT2 -> Left Motor
 * - OUT3 & OUT4 -> Right Motor
 * 
 * ====================================================================
 */

// ===================== PIN DEFINITIONS =====================

// Ultrasonic Sensor Pins
const int TRIG_PIN = 9;
const int ECHO_PIN = 10;

// Motor Driver Pins - Left Motor (Motor A)
const int ENA = 5;   // PWM pin for left motor speed
const int IN1 = 6;   // Left motor direction control
const int IN2 = 7;   // Left motor direction control

// Motor Driver Pins - Right Motor (Motor B)
const int IN3 = 8;   // Right motor direction control
const int IN4 = 11;  // Right motor direction control
const int ENB = 3;   // PWM pin for right motor speed

// ===================== CONFIGURATION =====================

const int OBSTACLE_DISTANCE = 20;  // Distance in cm to detect obstacle
const int MOTOR_SPEED = 180;       // Motor speed (0-255), 180 = ~70% speed
const int TURN_TIME = 400;         // Time to turn in milliseconds
const int BACKUP_TIME = 300;       // Time to backup in milliseconds

// ===================== SETUP =====================

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Setup ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Setup motor driver pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Start with motors stopped
  stopMotors();
  
  Serial.println("Obstacle Avoiding Car Initialized!");
  Serial.println("Starting in 2 seconds...");
  delay(2000);
}

// ===================== MAIN LOOP =====================

void loop() {
  int distance = measureDistance();
  
  // Print distance for debugging
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // Check if obstacle is detected
  if (distance < OBSTACLE_DISTANCE && distance > 0) {
    Serial.println("Obstacle detected! Avoiding...");
    avoidObstacle();
  } else {
    // No obstacle, move forward
    moveForward();
  }
  
  delay(100);  // Small delay for stability
}

// ===================== DISTANCE MEASUREMENT =====================

int measureDistance() {
  // Clear the trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Send 10 microsecond pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read the echo pin
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);  // Timeout after 30ms
  
  // Calculate distance in cm
  // Speed of sound is 343 m/s or 0.0343 cm/microsecond
  // Distance = (duration * speed) / 2 (divide by 2 for round trip)
  int distance = duration * 0.0343 / 2;
  
  // Return 0 if no echo received (out of range)
  if (duration == 0) {
    return 999;  // Return large value if no obstacle
  }
  
  return distance;
}

// ===================== OBSTACLE AVOIDANCE =====================

void avoidObstacle() {
  // Stop immediately
  stopMotors();
  delay(200);
  
  // Backup
  Serial.println("Backing up...");
  moveBackward();
  delay(BACKUP_TIME);
  
  // Stop
  stopMotors();
  delay(200);
  
  // Measure distances by turning slightly
  // Turn right and check
  Serial.println("Checking right...");
  turnRight();
  delay(200);
  stopMotors();
  delay(200);
  int rightDistance = measureDistance();
  Serial.print("Right distance: ");
  Serial.println(rightDistance);
  
  // Turn left (return to center and go left)
  Serial.println("Checking left...");
  turnLeft();
  delay(400);
  stopMotors();
  delay(200);
  int leftDistance = measureDistance();
  Serial.print("Left distance: ");
  Serial.println(leftDistance);
  
  // Decide which direction to turn based on measurements
  if (rightDistance > leftDistance) {
    // More space on right, turn right
    Serial.println("Turning right...");
    turnRight();
    delay(TURN_TIME);
  } else {
    // More space on left, keep turning left
    Serial.println("Turning left...");
    turnLeft();
    delay(TURN_TIME);
  }
  
  // Stop after turning
  stopMotors();
  delay(200);
}

// ===================== MOTOR CONTROL FUNCTIONS =====================

void moveForward() {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, MOTOR_SPEED);
  
  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, MOTOR_SPEED);
}

void moveBackward() {
  // Left motor backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, MOTOR_SPEED);
  
  // Right motor backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, MOTOR_SPEED);
}

void turnLeft() {
  // Left motor backward (or stop)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, MOTOR_SPEED);
  
  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, MOTOR_SPEED);
}

void turnRight() {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, MOTOR_SPEED);
  
  // Right motor backward (or stop)
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, MOTOR_SPEED);
}

void stopMotors() {
  // Stop both motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

// ===================== END OF CODE =====================

/*
 * TROUBLESHOOTING TIPS:
 * ====================================================================
 * 
 * 1. Motors not running:
 *    - Check power supply (motors need 6-12V)
 *    - Ensure L298N jumpers are properly set
 *    - Check motor connections to OUT1-OUT4
 * 
 * 2. Motors running in wrong direction:
 *    - Swap the two wires of that motor on L298N output
 *    - Or swap IN1/IN2 or IN3/IN4 pin assignments in code
 * 
 * 3. Car not avoiding obstacles:
 *    - Check ultrasonic sensor connections
 *    - Verify sensor is getting 5V power
 *    - Open Serial Monitor (9600 baud) to see distance readings
 *    - Adjust OBSTACLE_DISTANCE value if needed
 * 
 * 4. Car turning too much or too little:
 *    - Adjust TURN_TIME value (increase for more turn)
 *    - Adjust MOTOR_SPEED for faster/slower movement
 * 
 * 5. Erratic behavior:
 *    - Ensure common ground between Arduino and motor driver
 *    - Check all connections are secure
 *    - Battery might be low - check voltage
 * 
 * CUSTOMIZATION:
 * ====================================================================
 * 
 * You can adjust these values at the top of the code:
 * - OBSTACLE_DISTANCE: Detection range (default 20cm)
 * - MOTOR_SPEED: Speed of motors 0-255 (default 180)
 * - TURN_TIME: How long to turn (default 400ms)
 * - BACKUP_TIME: How long to backup (default 300ms)
 * 
 */
