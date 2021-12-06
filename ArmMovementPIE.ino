#include <Servo.h>
Servo leftArm;
Servo rightArm;

// Pin Numbers
const int buttonPin = 2;     // the number of the pushbutton pin
const int leftArmPin =  8;      // the number of the left arm servo pin
const int rightArmPin =  9;      // the number of the right arm servo pin
const int ledPin =  12;      // the number of the button LED

int buttonState = 0;         // variable for reading the pushbutton status

void setup() {
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT); 
  digitalWrite(buttonPin, HIGH);

  //initialize the button LED as an output
  pinMode(ledPin, OUTPUT);

  //attach servos to respective pins
  leftArm.attach(leftArmPin);
  rightArm.attach(rightArmPin);
  
  
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is LOW:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
    leftArm.write(90);
    rightArm.write(90);
    
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
    leftArm.write(60);
    rightArm.write(60);
    delay(3700);
    leftArm.write(90);
    rightArm.write(90);
    delay(3500);
    leftArm.write(120);
    rightArm.write(120);
    delay(3700);
  }
}
