// Motor control signals
#define LEFT_ENABLE   5
#define RIGHT_ENABLE   6
#define IN1   7
#define IN2   8
#define IN3   9
#define IN4   11

#define ERROR_LED 13
#define ERROR_BLINK_RATE_MS 500

#define BLUETOOTH_DELAY_SEC 5

//  Speed of the robot in feet/second
#define SPEED_RATE_FORWARD 2.166              // Feet/Second
#define SPEED_RATE_REVERSE 2.10               // Feet/Second

//  Speed adjustments for each side so it goes more straight

//  Minimum and maximum speeds the car is allowed to go
#define MINIMUM_SPEED 110
#define MAXIMUM_SPEED 245

//  Starting speed and adjustment settings
#define SPEED_LEFT 110
#define SPEED_LEFT_ADJUST 0
#define TURN_RATE_LEFT 289.0                  // Degrees/Second
#define TURN_RATE_LEFT_ADJUST 0

#define SPEED_RIGHT 110
#define SPEED_RIGHT_ADJUST 0
#define TURN_RATE_RIGHT 289.0                 // Degrees/Second
#define TURN_RATE_RIGHT_ADJUST 0

#define TURN_INCREMENT_DEG 15

//  Trackers for the current speed of each side
unsigned int leftSpeedCurrent = 110;
unsigned int rightSpeedCurrent = 110;
unsigned int currentSpeed = 110;
int speedIncrement = 30;

/*
  Utility functions
*/

void blinkLED(uint8_t ledPin, uint16_t rate_ms) {
  digitalWrite(ledPin, HIGH);
  delay(rate_ms);
  digitalWrite(ledPin, LOW);
  delay(rate_ms);
}

/*
  Robot functions
*/

void setSpeed(unsigned int leftSpeed, unsigned int rightSpeed) {
  enableMotors();

  if (leftSpeed < MINIMUM_SPEED or leftSpeed + SPEED_LEFT_ADJUST > MAXIMUM_SPEED or
       rightSpeed < MINIMUM_SPEED or rightSpeed + SPEED_RIGHT_ADJUST > MAXIMUM_SPEED) {
    //  Either the left speed or right speed, or both, are out of range
    while(true) {
      blinkLED(ERROR_LED, ERROR_BLINK_RATE_MS);
    }
  } else {
    //  Set the speeds
    analogWrite(LEFT_ENABLE, leftSpeed + SPEED_LEFT_ADJUST);
    leftSpeedCurrent = leftSpeed;
    analogWrite(RIGHT_ENABLE, rightSpeed + SPEED_RIGHT_ADJUST);
    rightSpeedCurrent = rightSpeed;
  }

  //  Set the speeds
  analogWrite(LEFT_ENABLE, leftSpeed + SPEED_LEFT_ADJUST);
  leftSpeedCurrent = leftSpeed;
  analogWrite(RIGHT_ENABLE, rightSpeed + SPEED_RIGHT_ADJUST);
  rightSpeedCurrent = rightSpeed;
}

void disableMotors(void) {    
  digitalWrite(LEFT_ENABLE, LOW);
  digitalWrite(RIGHT_ENABLE, LOW);
}

void enableMotors(void) {
  digitalWrite(LEFT_ENABLE, HIGH);
  digitalWrite(RIGHT_ENABLE, HIGH);
}

void brakeFull(void) {
  disableMotors();

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forward(float distance) {
  float runTimeMS;
  
  enableMotors();
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  runTimeMS = distance / SPEED_RATE_FORWARD * 1000;
 
  delay(runTimeMS); 

  brakeFull();
}

void reverse(float distance) {
  float runTimeMS;
  
  enableMotors();
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  runTimeMS = distance / SPEED_RATE_REVERSE * 1000;

  delay(runTimeMS);

  brakeFull();
}

void turnLeftRelative(uint16_t turnDegrees) {
  float runTimeMS;
  
  enableMotors();

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  runTimeMS = turnDegrees / (TURN_RATE_LEFT + TURN_RATE_LEFT_ADJUST) * 1000.0;

  delay(runTimeMS);

  brakeFull();
}

void turnRightRelative(uint16_t turnDegrees) {
  float runTimeMS;
  
  enableMotors();
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  runTimeMS = turnDegrees / (TURN_RATE_RIGHT + TURN_RATE_RIGHT_ADJUST) * 1000.0;

  delay(runTimeMS);

  brakeFull();
}

/*
  Calibration functions
*/

void calibrateForward() {
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  delay(5000);
  brakeFull();
}

void calibrateReverse() {
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  delay(5000);
  brakeFull();
}
void calibrateRight() {
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  delay(5000);
  brakeFull();
}

void calibrateLeft() {
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  delay(5000);
  brakeFull();
}

void setup(void) {
  Serial.begin(15200);
  
  // Setup all the pins
  pinMode(LEFT_ENABLE, OUTPUT);
  pinMode(RIGHT_ENABLE, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //  Allow Bluetooth time to connect
  Serial.println("Waiting for Bluetooth to connect");
  for (uint8_t i = 0; i < BLUETOOTH_DELAY_SEC * 2; i++) {
    blinkLED(ERROR_LED, 500);
  }

  //  Enable the motors
  Serial.println("Enabling motors.");
  enableMotors();

  //  Make sure the motors are stopped
  Serial.println("Stopping all motors");
  brakeFull();
  
  //  Set initial motor speeds
  Serial.println("Setting initial motor speeds");
  setSpeed(SPEED_LEFT + SPEED_LEFT_ADJUST, SPEED_RIGHT + SPEED_RIGHT_ADJUST);

/*  For Robot Calibration ONLY  */
  //setSpeed(255, 255);
  //calibrateForward();
  //calibrateReverse();
  //calibrateRight();
  //calibrateLeft();
}

void loop(void) {    
  char command;

  blinkLED(ERROR_LED, 1000);
  
  if (Serial.available()) {
    //  Handle commands from remote
    command = Serial.read();

    //  Convert lower to upper case for commands
    if (command >= 97 and command <= 122) {
      command -= 32;
    }

    switch(command) {
      case 'F':
        forward(1.0);
        break;

      case 'B':
        reverse(1.0);
        break;

      case 'R':
        turnRightRelative(TURN_INCREMENT_DEG);
        break;

      case 'L':
        turnLeftRelative(TURN_INCREMENT_DEG);
        break;
    }
  } else {
    //  Do autonomous stuff
    setSpeed(MINIMUM_SPEED, MINIMUM_SPEED);

    while(true) {
      //  Change speed ramping direction if neccessary
      if (speedIncrement >= 0) {
        if (currentSpeed <= 250) {
          ;
        } else if (currentSpeed >= 250) {
          speedIncrement = -30;
        }
      } else {
        if (currentSpeed >= 110) {
          ;
        } else if (currentSpeed <= 110) {
            speedIncrement = 30;
        }
      }
  
      leftSpeedCurrent += speedIncrement;
      rightSpeedCurrent += speedIncrement;
      currentSpeed += speedIncrement;
      setSpeed(leftSpeedCurrent, rightSpeedCurrent);
    }
  }

  delay(1000);
}
