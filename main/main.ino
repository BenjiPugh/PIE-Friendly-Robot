#include <Adafruit_MotorShield.h>

#include <Adafruit_LSM303DLH_Mag.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Set up motor shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Set up magnetometer
Adafruit_LSM303DLH_Mag_Unified mag = Adafruit_LSM303DLH_Mag_Unified(12345);

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
double k_p = 0.1;
double k_d = 0.0;
int error_prev = 0;
int tPrevious = 0;
int baseSpeed = 30;

int setpoint = 0;

// Toggle printing CSV output to serial
bool print_csv = false;

void setup() {
  Serial.begin(115200);          // Set up Serial library at 9600 bps

  if (!AFMS.begin()) {         // Start motor shield with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
  tStart = millis();
  tPrevious = tStart;

  if (!mag.begin()) {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
}

void loop() {

  detectSerial();
  
  //bangBang(); // Bang-bang control loop (disabled)
  pdControl(); // PID main control loop
  
  // Write motor outputs
  motorWrite();

  // CSV serial printout for plotting purposes
  if (print_csv){
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
    // get person angle from person detection script
    int val = atoi(cmd_buffer + 2);
    setpoint = calculate_setpoint(val, compass_heading());
    Serial.print("Person angle: ");
    Serial.println(val);

  }
}

int compass_heading() {
    
    /* Get a new sensor event */
    sensors_event_t event;
    mag.getEvent(&event);

    float Pi = 3.14159;

    // Calculate the angle of the vector y,x
    float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;

    // Normalize to 0-360
    if (heading < -180) {
      heading = 180 + heading;
    } else if (heading > 180) {
      heading = heading - 180;
    }
    //Serial.print("Heading: ");
    //Serial.println(heading);

    return int(heading);

}

int calculate_setpoint(int person_angle, int compass_heading) {
    return int((compass_heading + person_angle) % 360);
}


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
  float error = (setpoint - compass_heading()) % 360;
  if (error > 180) {
    error = error - 360;
  } else if (error < -180) {
    error = error + 360;
  }
  float tElapsed = (millis() - tPrevious)/1000.0;

  // Motor speed difference value: the controller output
  // Sum of kP * sensor difference + k_d * rate of change of the sensor difference
  int motorDiff = k_p * error + k_d * (error-error_prev)/tElapsed;

  float error_prev = error;
  
  // Apply speed gradient to motors
  leftMotorVal = -max(min(baseSpeed + motorDiff, 128),-128);
  rightMotorVal = max(min(baseSpeed - motorDiff, 128), -128);
  
  // Update last timestamp
  tPrevious = millis();
}
