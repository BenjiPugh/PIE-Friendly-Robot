#include <Adafruit_MotorShield.h>

// Set up motor shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Find motors
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);

// Sensor pins
int leftSens = A0;
int rightSens = A1;

// Current motor values
int leftMotorVal = 0;
int rightMotorVal = 0;

// Start time
unsigned long tStart = 0;

// Serial input command buffer
uint16_t cmd_buffer_pos = 0;
const uint8_t CMD_BUFFER_LEN = 20;
char cmd_buffer[CMD_BUFFER_LEN];

// PD params
double k_p = 0.15;
double k_d = 0.02;
int sensDiffPrev = 0;
int tPrevious = 0;
int baseSpeed = 30;

// Toggle printing CSV output to serial
bool print_csv = false;

void setup() {
  Serial.begin(9600);          // Set up Serial library at 9600 bps

  if (!AFMS.begin()) {         // Start motor shield with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
  tStart = millis();
  tPrevious = tStart;
}

void loop() {

  detectSerial();
  
  //bangBang(); // Bang-bang control loop (disabled)
  pdControl(); // PID main control loop
  
  // Write motor outputs
  motorWrite();

  // CSV serial printout for plotting purposes
  if (print_csv){
    Serial.print(analogRead(leftSens));
    Serial.print(",");
    Serial.print(analogRead(rightSens));
    Serial.print(",");
    Serial.print(leftMotorVal);
    Serial.print(",");
    Serial.println(rightMotorVal);
    delay(20);
  }
}

// Read from serial input buffer to command buffer
// Clear command buffer if newline detected or buffer full
// Read from serial input buffer otherwise
void detectSerial() {
  if (Serial.available()) {
		char ch = Serial.read(); // Read character
		
		if (ch == '\r') {
		  // New line detected - end of entered command
		  Serial.println("New line detected");
		  cmd_buffer[cmd_buffer_pos] = '\0'; // Null terminate command string
		  
			parseCommandBuffer(); // Parse and execute command

		  cmd_buffer_pos = 0; // Reset index back to 0
		  memset(cmd_buffer, 0, sizeof(cmd_buffer)); // Set buffer to be all zero

		} else if (cmd_buffer_pos == CMD_BUFFER_LEN - 1) {
			// Command buffer is full and needs to be reset to read the new character
		  cmd_buffer_pos = 0; // Reset index back to 0
		  memset(cmd_buffer, 0, sizeof(cmd_buffer)); // Set command to 0

		  cmd_buffer[cmd_buffer_pos] = ch; // Save the new character
		  cmd_buffer_pos++; // Increment counter position

		} else {
		  cmd_buffer[cmd_buffer_pos] = ch; // Save the new character
		  cmd_buffer_pos++; // Increment counter position
		}
  }
}

// Parse and execute commands sent over serial:
void parseCommandBuffer() {
  Serial.print("Command read: ");
  Serial.println(cmd_buffer);
  if (strncmp(cmd_buffer, "LM", 2) == 0) {
		// Directly set left motor speed
    int val = atoi(cmd_buffer + 2);
    leftMotorVal = val;
    motorWrite();
    
    Serial.print("Setting left motor value: ");
    Serial.println(val);

  } else if (strncmp(cmd_buffer, "RM", 2) == 0) {
		// Directly set right motor speed
    int val = atoi(cmd_buffer + 2);
    rightMotorVal = val;
    motorWrite();
    
    Serial.print("Setting right motor value: ");
    Serial.println(val);

  }
  else if (strncmp(cmd_buffer, "KP", 2) == 0) {
		// Directly set proportional gain Kp
    double val = atof(cmd_buffer + 2);
    k_p = val;
    
    Serial.print("Setting KP value: ");
    Serial.println(val);

  }
    else if (strncmp(cmd_buffer, "KD", 2) == 0) {
		// Directly set derivative gain Kd
    double val = atof(cmd_buffer + 2);
    k_d = val;
    
    Serial.print("Setting KD value: ");
    Serial.println(val);

  }
  else if (strncmp(cmd_buffer, "BS", 2) == 0) {
		// Set base speed of vehicles
    int val = atoi(cmd_buffer + 2);
    baseSpeed = val;
    
    Serial.print("Setting base_speed value: ");
    Serial.println(val);

  }
  else if (strncmp(cmd_buffer, "DA", 2) == 0) {
		// Start data output
    print_csv = true;
    
    Serial.println("Starting CSV");
  }
  else if (strncmp(cmd_buffer, "AG", 2) == 0) {
	// Directly set right motor speed
    int val = atoi(cmd_buffer + 2);
    
    
    Serial.print("Recieved angle: ");
    Serial.println(val);

  }
}
/*
int create_setpoint(measured_angle) {
    

}
*/


// Write currently desired motor values to the motors
// Accounts for H bridge direction changes
void motorWrite() {
	// Check sign of value and flip motor directions accordingly
  if (leftMotorVal > 0) {
    leftMotor->run(FORWARD);
  } else if (leftMotorVal < 0) {
    leftMotor->run(BACKWARD);
  } else {
    leftMotor->run(RELEASE);
  }

  if (rightMotorVal >= 0) {
    rightMotor->run(FORWARD);
  } else {
    rightMotor->run(BACKWARD);
  }
  
	// Write motor speeds
  leftMotor->setSpeed(abs(leftMotorVal));
  rightMotor->setSpeed(abs(rightMotorVal));
}

// Bangbang control
void bangBang() {
  int leftSensVal = analogRead(leftSens);
  int rightSensVal = analogRead(rightSens);
  int difference = leftSensVal - rightSensVal;

  // Set right motor to max if left sensor is darker, or vice versa
  if (difference > 0) {
    leftMotorVal = 0;
    rightMotorVal = 128;
  } else {
    leftMotorVal = -128;
    rightMotorVal = 0;
  }
}

// PD control
void pdControl() {
  int leftSensVal = analogRead(leftSens);
  int rightSensVal = analogRead(rightSens);
  int sensDiff = rightSensVal - leftSensVal;
  int tElapsed = millis() - tPrevious;

  // Motor speed difference value: the controller output
  // Sum of kP * sensor difference + k_d * rate of change of the sensor difference
  int motorDiff = k_p * sensDiff + k_d * (sensDiff-sensDiffPrev)/tElapsed;

  // Apply speed gradient to motors
  leftMotorVal = -max(min(baseSpeed + motorDiff, 128),-128);
  rightMotorVal = max(min(baseSpeed - motorDiff, 128), -128);
  
  // Update last timestamp
  sensDiffPrev = millis();
}