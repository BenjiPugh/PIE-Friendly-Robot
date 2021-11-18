#include <Adafruit_MotorShield.h>


#include <Adafruit_Sensor.h>
#include <Wire.h>


// Connect to the two encoder outputs!
#define LEFT_ENCODER_1   2
#define LEFT_ENCODER_2   10
#define RIGHT_ENCODER_1  3
#define RIGHT_ENCODER_1  11 

// These let us convert ticks-to-RPM
#define GEARING     20
#define ENCODERMULT 12

// Set up motor shield
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Find motors
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);


// Current motor values
int leftMotorVal = 0;
int rightMotorVal = 0;
volatile bool motordir = FORWARD;
volatile int left_count = 0;
volatile int right_count = 0;

// Bot Parameters
float wheel_distance = 0.25; //meters apart the wheels on the bot are
float wheel_radius = 0.03; //meters wheel radius
float turn_ratio = wheel_radius / wheel_distance * 360 / GEARING / ENCODERMULT; // angle based on the encoder count delta of the two wheels

// Start time
unsigned long tStart = 0;

// Serial input command buffer
uint16_t cmd_buffer_pos = 0;
const uint8_t CMD_BUFFER_LEN = 20;
char cmd_buffer[CMD_BUFFER_LEN];

// PD params
double k_p = 0.15;
double k_d = 0.0;
int error_prev = 0;
int tPrevious = 0;
int baseSpeed = 0;

int setpoint = 0;

// Toggle printing CSV output to serial
bool print_csv = false;

void setup() {
  Serial.begin(115200);          // Set up Serial library at 115200 bps

  pinMode(LEFT_ENCODER_1, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_2, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_1, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_1), left_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_1), right_interrupt, RISING);

  delay(100);

  //while (!Serial) delay(1);

  Serial.println("MMMMotor party!");
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }


  if (!AFMS.begin()) {         // Start motor shield with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
  tStart = millis();
  tPrevious = tStart;

  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);

}

void loop() {

  detectSerial();
  
  //bangBang(); // Bang-bang control loop (disabled)
  pdControl(); // PID main control loop
  
  // Write motor outputs
  motorWrite();

  delay(20);

  // CSV serial printout for plotting purposes
  if (print_csv){
    Serial.print(leftMotorVal);
    Serial.print(",");
    Serial.println(rightMotorVal);
    delay(20);
  }
}

void left_interrupt() {

  motordir = digitalRead(LEFT_ENCODER_2);
  if (motordir) {
    left_count += -1;
    } else {
      left_count += 1;
    }
}

void right_interrupt() {

  motordir = digitalRead(RIGHT_ENCODER_2);
  if (motordir) {
    right_count += -1;
    } else {
      right_count += 1;
    }
}


int calculate_angle() {
  
   return int((right_count - left_count) * turn_ratio);
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
    // TODO: get person angle calculated in tag_detection.py
    int val = atoi(cmd_buffer + 2);
    // TODO: We'll get person angle from tag_detection.py, but how get setpoint?
    setpoint = calculate_setpoint(val);
    Serial.print("Tag angle: ");
    Serial.println(val);

  }
}


// Calculate the setpoint based off the current heading and the angle of person given over serial
int calculate_setpoint(int person_angle) {
  heading = calculate_angle();
  return(heading + person_angle);
  
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


// PD control
void pdControl() {
  Serial.print("Setpoint:");
  Serial.println(setpoint);
  float heading = calculate_angle();
  float error = (setpoint - heading);
  Serial.print("Heading (0-360):");
  Serial.println(heading);

  Serial.print("Error:");
  Serial.println(error);
  float tElapsed = (millis() - tPrevious)/1000.0;

  // Motor speed difference value: the controller output
  // Sum of kP * sensor difference + k_d * rate of change of the sensor difference
  int motorDiff = k_p * error + k_d * (error-error_prev)/tElapsed;

  error_prev = error;
  
  // Apply speed gradient to motors
  leftMotorVal = max(min(baseSpeed + motorDiff, 128),-128);
  rightMotorVal = -max(min(baseSpeed - motorDiff, 128), -128);
  
  // Update last timestamp
  tPrevious = millis();
}
