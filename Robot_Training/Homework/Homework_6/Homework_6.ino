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
#define MAX_SPEED_RATE 2.40
#define SPEED_RATE 2.45

//  Angular rate in degrees/second
#define ANGLE_RATE 333.0

//  Speed adjustments for each side so it goes more straight

//  Minimum and maximum speeds the car is allowed to go
#define MINIMUM_SPEED 110
#define MAXIMUM_SPEED 245

//  Starting speed and adjustment settings
#define LEFT_SPEED 110
#define LEFT_SPEED_ADJUST 0

#define RIGHT_SPEED 110
#define RIGHT_SPEED_ADJUST 0

#define TURN_SIZE_DEG 15

//  Trackers for the current speed of each side
unsigned int left_speed_current = 110;
unsigned int right_speed_current = 110;
unsigned int current_speed = 110;
int increment = 30;

void blink_led(uint8_t led_pin, uint16_t rate_ms) {
  digitalWrite(led_pin, HIGH);
  delay(rate_ms);
  digitalWrite(led_pin, LOW);
  delay(rate_ms);
}

void set_speed(unsigned int left_speed, unsigned int right_speed) {
  enable_motors();

  if (left_speed < MINIMUM_SPEED or left_speed + LEFT_SPEED_ADJUST > MAXIMUM_SPEED or
       right_speed < MINIMUM_SPEED or right_speed + RIGHT_SPEED_ADJUST > MAXIMUM_SPEED) {
    //  Either the left speed or right speed, or both, are out of range
    while(true) {
      blink_led(ERROR_LED, ERROR_BLINK_RATE_MS);
    }
  } else {
    //  Set the speeds
    analogWrite(LEFT_ENABLE, left_speed + LEFT_SPEED_ADJUST);
    left_speed_current = left_speed;
    analogWrite(RIGHT_ENABLE, right_speed + RIGHT_SPEED_ADJUST);
    right_speed_current = right_speed;
  }

  //  Set the speeds
  analogWrite(LEFT_ENABLE, left_speed + LEFT_SPEED_ADJUST);
  left_speed_current = left_speed;
  analogWrite(RIGHT_ENABLE, right_speed + RIGHT_SPEED_ADJUST);
  right_speed_current = right_speed;
}

void disable_motors(void) {    
  digitalWrite(LEFT_ENABLE, LOW);
  digitalWrite(RIGHT_ENABLE, LOW);
}

void enable_motors(void) {
  digitalWrite(LEFT_ENABLE, HIGH);
  digitalWrite(RIGHT_ENABLE, HIGH);
}

void all_stop(void) {
  disable_motors();

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forward(float distance) {
  float run_time_ms;
  
  enable_motors();
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(5000);
  //run_time_ms = distance / SPEED_RATE * 1000;
 
  //delay(run_time_ms); 
}

void reverse(float distance) {
  float run_time_ms;
  
  enable_motors();
  
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(5000);

  //run_time_ms = distance / SPEED_RATE * 1000;

  //delay(run_time_ms);
}

void turn_left(uint16_t turn_degrees) {
  float run_time_ms;
  
  enable_motors();

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(5000);
//  run_time_ms = turn_degrees / 345.0 * 1000.0;

  //run_time_ms = turn_degrees / ANGLE_RATE * 1000.0;
//delay(run_time_ms);

  all_stop();
}

void turn_right(uint16_t turn_degrees) {
  float run_time_ms;
  
  enable_motors();
  
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(5000);

//  run_time_ms = turn_degrees / 345.0 * 1000.0;
  //run_time_ms = turn_degrees / ANGLE_RATE * 1000.0;

//  delay(run_time_ms);

  all_stop();
}
 
void calibrate_forward() {
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  delay(5000);
  all_stop();
}

void calibrate_reverse() {
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  delay(5000);
  all_stop();
}
void calibrate_right() {
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  delay(5000);
  all_stop();
}

void calibrate_left() {
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  delay(5000);
  all_stop();
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
    blink_led(ERROR_LED, 500);
  }

  //  Enable the motors
  Serial.println("Enabling motors.");
  enable_motors();

  //  Make sure the motors are stopped
  Serial.println("Stopping all motors");
  all_stop();
  
  //  Set initial motor speeds
  Serial.println("Setting initial motor speeds");
  set_speed(LEFT_SPEED + LEFT_SPEED_ADJUST, RIGHT_SPEED + RIGHT_SPEED_ADJUST);
}

void loop(void) {  
  char command;

  blink_led(ERROR_LED, 1000);
  
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
        turn_right(TURN_SIZE_DEG);
        break;

      case 'L':
        turn_left(TURN_SIZE_DEG);
        break;
    }
  } else {
    forward(10);
//forward(5.0);
//      delay(2000);
//      reverse(5.0);
//     delay(2000);
/*
    //  Do autonomous stuff
    set_speed(MINIMUM_SPEED, MINIMUM_SPEED);

    while(true) {
      forward(5.0);
  
      delay(2000);
  
      reverse(5.0);
  
      delay(2000);

      //  Change speed ramping direction if neccessary
      if (increment >= 0) {
        if (current_speed <= 250) {
          ;
        } else if (current_speed >= 250) {
          increment = -30;
        }
      } else {
        if (current_speed >= 110) {
          ;
        } else if (current_speed <= 110) {
            increment = 30;
        }
      }
  
      left_speed_current += increment;
      right_speed_current += increment;
      current_speed += increment;
      set_speed(left_speed_current, right_speed_current);
    }
  }

  delay(1000);
*/
  }
}
