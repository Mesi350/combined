/* Simple line follower */
#include <SerialLCD.h>
#include <SoftwareSerial.h>

/* Motor constants */
const uint8_t MOTOR_LEFT_DIR_PIN = 4;
const uint8_t MOTOR_LEFT_SPEED_PIN = 5;
const uint8_t MOTOR_RIGHT_DIR_PIN = 7;
const uint8_t MOTOR_RIGHT_SPEED_PIN = 6;

const uint8_t FORWARD = LOW;
const uint8_t BACKWARD = HIGH;

const uint8_t MAX_SPEED = 230;
const uint8_t LOW_SPEED = 190;
const uint8_t BACK_SPEED = 120;

/* Line detector constants */

const uint8_t LD_LEFT_PIN = A7;
const uint8_t LD_MIDDLE_PIN = A6;
const uint8_t LD_RIGHT_PIN = A5;

int line[5];    // line[0]: min, line[1] = max, line[2]: left, line[3]: middle, line[4]: right

/* LCD constants */

const uint8_t LCD_RX_PIN = 8;
const uint8_t LCD_TX_PIN = 13;   // collide with LED

SerialLCD slcd(LCD_RX_PIN, LCD_TX_PIN);

/* Motor functions */
void motor_begin() {
    pinMode(MOTOR_LEFT_DIR_PIN, OUTPUT);
    pinMode(MOTOR_LEFT_SPEED_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_SPEED_PIN, OUTPUT); 

    digitalWrite(MOTOR_LEFT_DIR_PIN, LOW);
    digitalWrite(MOTOR_LEFT_SPEED_PIN, LOW);
    digitalWrite(MOTOR_RIGHT_DIR_PIN, LOW);
    digitalWrite(MOTOR_RIGHT_SPEED_PIN, LOW);
}

void motor_set_speed(uint8_t left, uint8_t right) {
    analogWrite(MOTOR_LEFT_SPEED_PIN, left);
    analogWrite(MOTOR_RIGHT_SPEED_PIN, right);
}

void motor_set_direction(uint8_t left, uint8_t right) {
    digitalWrite(MOTOR_LEFT_DIR_PIN, left);
    digitalWrite(MOTOR_RIGHT_DIR_PIN, !right);
}

void motor_stop() {
    motor_set_speed(0, 0);
}

void motor_go_forward() {
    motor_set_direction(FORWARD, FORWARD);
    motor_set_speed(MAX_SPEED, MAX_SPEED);
}

void motor_go_backward(unsigned long time) {
    motor_set_direction(BACKWARD, BACKWARD);
    motor_set_speed(LOW_SPEED, LOW_SPEED);
    delay(time);
}

void motor_turn(int degree) {
    if (degree >= 0) {
        motor_set_direction(FORWARD, BACKWARD);
    } else {
        motor_set_direction(BACKWARD, FORWARD);
    }
    motor_set_speed(LOW_SPEED, LOW_SPEED);
    delay(abs(degree)*5);
}

/* Line detector functions */

void read_line() {
    line[2] = analogRead(LD_LEFT_PIN);
    line[3] = analogRead(LD_MIDDLE_PIN);
    line[4] = analogRead(LD_RIGHT_PIN);
    line[0] = min(line[0], line[2]);
    line[0] = min(line[0], line[3]);
    line[0] = min(line[0], line[4]);
    line[1] = max(line[1], line[2]);
    line[1] = max(line[1], line[3]);
    line[1] = max(line[1], line[4]);
}

/* LCD functions */
void show_line(int line[5]) {
    //slcd.clear();
    // clear line without clear
    slcd.setCursor(0, 0);
    slcd.print("                ");
    slcd.setCursor(0, 0); 
    slcd.print((float)line[0], DEC);
    slcd.setCursor(5, 0);
    slcd.print((float)line[1], DEC);
    slcd.setCursor(0, 1); 
    slcd.print("                ");
    slcd.setCursor(0, 1);
    slcd.print((float)line[2], DEC);
    slcd.setCursor(5, 1);
    slcd.print((float)line[3], DEC);
    slcd.setCursor(10, 1);
    slcd.print((float)line[4], DEC);
}

void setup() {
    slcd.begin();
    motor_begin();
    motor_go_forward();
    line[0] = 1024; // min
    line[1] = 0;    // max
    line[2] = 0;
    line[3] = 0;
    line[4] = 0;
}

void loop() {
    // line read
    read_line();
    int avg = (line[0]+line[1]) / 2;     // (max+min)/2
    // assume with floor with black line
    
   /* if (line[3] = line[2] = line[4]) {
            motor_set_speed(0,0);*/
            
    } else {
      if (line[3] > avg) {
        // go forward
        motor_go_forward();
      } else {
           if (line[2] > line[4]) {
              // left higher than right, turn left
                motor_set_direction(BACKWARD, FORWARD);
                motor_set_speed(BACK_SPEED, MAX_SPEED);
          } else {
              // right higher than left, turn right
                motor_set_direction(FORWARD, BACKWARD);
                motor_set_speed(MAX_SPEED, BACK_SPEED); }
         }
           }
    
    show_line(line);
    delay(50);
  }
