#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <Wire.h>

#include "main.h"

// Connect to the two encoder outputs!
#define LEFT_ENCODER_1   3
#define LEFT_ENCODER_2   11
#define RIGHT_ENCODER_1  2
#define RIGHT_ENCODER_2  10

// These let us convert ticks-to-RPM
#define GEARING     20
#define ENCODERMULT 12

Servo leftArm;
Servo rightArm;

// Pin Numbers
const int buttonPin = 0;     // the number of the pushbutton pin
const int leftArmPin =  8;      // the number of the left arm servo pin
const int rightArmPin =  9;      // the number of the right arm servo pin
const int ledPin =  12;      // the number of the button LED

// Debug led pin
int raspi_led = 5;

// Distance sensor pin
int distance_sens = A0;
int distance_reading = 0;

// Total guess for the distance sensor reading.
// TODO: actually test and determine this.
int hug_threshold = 500; 
unsigned long prev_timestamp = 0;
unsigned long consent_wait_time = 3000;

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
float wheel_distance = 0.165; //meters apart the wheels on the bot are
float wheel_radius = 0.03175; //meters wheel radius
float turn_ratio = wheel_radius / wheel_distance * 360 / GEARING / ENCODERMULT; // angle based on the encoder count delta of the two wheels

// Start time
unsigned long tStart = 0;

// Serial input command buffer
uint16_t cmd_buffer_pos = 0;
const uint8_t CMD_BUFFER_LEN = 20;
char cmd_buffer[CMD_BUFFER_LEN];

// PD params
double k_p = 0.6;
double k_d = 0.0;
int error_prev = 0;
int tPrevious = 0;
int baseSpeed = 70;
int topSpeed = 120;

int setpoint = 0;

// Behavioral FSM
/*enum class botState {
  Searching,
  Follow,
  ConsentWait,
  Hug
};*/
botState state;

// Stores the last communication time, used in the FSM
unsigned long lastComm = 0;
// variable for reading the pushbutton status
int buttonState = 0;

// Toggle printing CSV output to serial
bool print_csv = false;

void setup() {
  Serial.begin(115200);          // Set up Serial library at 115200 bps

  // Initialize pushbutton pin as an input
  pinMode(buttonPin, INPUT_PULLUP);

  //Initialize the button LED as an output
  pinMode(ledPin, OUTPUT);

  //attach servos to respective pins
  leftArm.attach(leftArmPin);
  rightArm.attach(rightArmPin);

  pinMode(LEFT_ENCODER_1, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_2, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_1, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_1), left_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_1), right_interrupt, RISING);

  lastComm = millis();

  delay(1000);

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

  pinMode(raspi_led, OUTPUT);
  digitalWrite(raspi_led, false);

  state = Searching;
}

void loop() {
  detectSerial();



  
  //bangBang(); // Bang-bang control loop (disabled)
  //pdControl(); // PID main control loop - moved into FSM

  FSM();

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

void FSM() {
    // Behavioral FSM
  switch(state) {
    case Searching:
      Serial.println("Searching");
      // Spin in place slowly until tag is detected
      //leftMotorVal = baseSpeed / 2; // or whatever speed works

      leftMotorVal = 0;
      rightMotorVal = 0;
      
      
      // If the raspberry pi has recently communicated, start following
      if (millis() - lastComm < 300) {
        state = Follow;
        break;
      }
      break;
    case Follow:
      // PID controlled movement
      Serial.println("Following");
      pdControl();
      // Update motor speeds
      
      // If the raspberry pi has stopped communicating, stop
      if (millis() - lastComm > 400) {
        state = Searching;
        break;
      }
      distance_reading = analogRead(distance_sens);
      if (distance_reading > hug_threshold) {
        prev_timestamp = millis();
        state = ConsentWait;
        break;
      }
      break;
    case ConsentWait:
      Serial.println("Waiting");
      leftMotorVal = 0;
      rightMotorVal = 0;
      
      buttonState = digitalRead(buttonPin);
      // If button is NOT pressed this iteration
      if (buttonState == HIGH) {
        // turn LED on:
        digitalWrite(ledPin, HIGH);
        leftArm.write(90);
        rightArm.write(90);
      // If button is pressed at any point,
      // transition to Hug state
      } else {
         digitalWrite(ledPin, LOW);
         state = Hug;
         break;
      }
      
      // If the time since transitioning into ConsentWait state
      // exceeds consent_wait_time, go back to Searching
      if (millis() - prev_timestamp > consent_wait_time) {
         digitalWrite(ledPin, LOW);
         state = Searching;
         break;
      }
      break;
    case Hug:
      leftMotorVal = 0;
      rightMotorVal = 0;
      Serial.println("Waiting");
      // turn LED off:
      
      leftArm.write(60);
      rightArm.write(60);
      delay(3700);
      leftArm.write(90);
      rightArm.write(90);
      delay(3500);
      leftArm.write(120);
      rightArm.write(120);
      delay(3700);

      state = Searching;
      break;
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
  //Serial.print("right count:");
  //Serial.println(right_count);
  //Serial.print("left count:");
  //Serial.println(left_count);
   return int((right_count - left_count) * turn_ratio);
  }



// Read from serial input buffer to command buffer
// Clear command buffer if newline detected or buffer full
// Read from serial input buffer otherwise
void detectSerial() {
  while (Serial.available()) {
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
    int val = atoi(cmd_buffer + 2);
    setpoint = calculate_setpoint(val);
    lastComm = millis();
    Serial.print("Tag angle: ");
    Serial.println(val);
    digitalWrite(raspi_led, true);
  }
}



// Calculate the setpoint based off the current heading and the angle of person given over serial
int calculate_setpoint(int person_angle) {
  int heading = calculate_angle();
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
  //Serial.print("Setpoint:");
  //Serial.println(setpoint);
  float heading = calculate_angle();
  float error = (setpoint - heading);
  //Serial.print("Heading:");
  //Serial.println(heading);

  // Process the error to make sure it is between -180 and 180
  while (error > 180) {
    error = error - 360;
    }

  while (error < -180) {
    error = error + 360;
    }
  //Serial.print("Error:");
  //Serial.println(error);
  float tElapsed = (millis() - tPrevious)/1000.0;

  // Motor speed difference value: the controller output
  // Sum of kP * sensor difference + k_d * rate of change of the sensor difference
  int motorDiff = k_p * error + k_d * (error-error_prev)/tElapsed;

  error_prev = error;
  
  // Apply speed gradient to motors
  leftMotorVal = max(min(baseSpeed - motorDiff, topSpeed),-topSpeed);
  rightMotorVal = max(min(baseSpeed + motorDiff, topSpeed), -topSpeed);
  //Serial.print("Left_M:");
  //Serial.println(leftMotorVal);
  //Serial.print("Right_M:");
  //Serial.println(rightMotorVal);
  
  // Update last timestamp
  tPrevious = millis();
}
