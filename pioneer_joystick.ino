/*
  Program to receive data from Pioneer 3-AT joystick to control the wheels from the robot.
*/

#include <math.h>
#include <Wire.h>

//Input pins
int key_greenPin = 2;
int joystick_yPin = A1;
int joystick_xPin = A2;
int potentiometerPin = A3;

//Output pins (motors)
int motor_left1 = 5; //5
int motor_right3 = 6; //6
//Output motors logic direction
int logic2 = 12; //left
int logic4 = 13; //right
//Output FM motors
int freq6_pin = 9;
int freq6; //left
int freq8; //right

//Variables for reading pushbuttons and pins
int key_yellow = 0;
int key_green = 0;
float joystick_x = 0;
float joystick_y = 0;
float potentiometer = 0;

//Output pwm
int pwm_left = 0;
int pwm_right = 0;

//Variables for joystick position
float joy_angle;
float joy_intensity;

char str[5];
  
// the setup routine runs once when you press reset:
void setup() {
  Wire.begin();
  Serial.begin(9600);
  pinMode(key_greenPin, INPUT);
  digitalWrite(key_greenPin, HIGH);

  pinMode(motor_left1, OUTPUT);
  pinMode(logic2, OUTPUT);
  pinMode(freq6_pin, OUTPUT);
  pinMode(motor_right3, OUTPUT);
  pinMode(logic4, OUTPUT);
  //pinMode(freq8, OUTPUT);

  //TCCR2B = TCCR2B & B11111000 | B00000001; // Set PWM for D3 & D11 - set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & 0b11111000 | 0x01; // Set PWM for D3 & D11 - set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & B11111000 | B00000100; 
  
}

// the loop routine runs over and over again forever:
void loop() {
  if(digitalRead(key_greenPin) == LOW){ //if green key is pressed
    // read the input on analog pins:

    joystick_x = analogRead(joystick_xPin);
    delay(10);
    joystick_x = analogRead(joystick_xPin);
    delay(10);
    joystick_y = analogRead(joystick_yPin);
    delay(10);
    joystick_y = analogRead(joystick_yPin);
    delay(10);
    potentiometer = analogRead(potentiometerPin);
    delay(10);
    potentiometer = analogRead(potentiometerPin);
    delay(10);

    Serial.println("X");
    Serial.println(joystick_x);
    Serial.println("Y");
    Serial.println(joystick_y);
    
    //map readings to a more workable values
    joystick_x = map(joystick_x, 0, 1023, -1000, 1000);
    joystick_y = map(joystick_y, 0, 1023, -1000, 1000);
    potentiometer = 0.05 + (potentiometer/1023) * 0.95; //map (0 to 1023) to (0.05 to 1)

    joystick_x = joystick_x/1000;
    joystick_y = joystick_y/1000;
    Serial.println("XXXX");
    Serial.println(joystick_x);
    Serial.println("YYYY");
    Serial.println(joystick_y);
    //joystick_x = 0.8;
    //joystick_y = 0.3;
    //potentiometer = 1;
  
    // TODO pwm logic IF key_green is pressed
    joy_angle = atan2(joystick_y, joystick_x); //finds angle of joystick
    joy_intensity = sqrt(pow(joystick_x, 2) + pow(joystick_y, 2)); //finds euclidian distance from joystick point to center

    Serial.println();
    //Serial.println("ANGLE");
    //Serial.println(joy_angle);
    //Serial.println("INTENSITY");
    //Serial.println(joy_intensity);
    //Serial.print("POTENCIOMETRO ");
    //Serial.println(potentiometer);
    
    //pwm logics
    left_logic(joy_angle, joy_intensity);
    right_logic(joy_angle, joy_intensity);

    pwm_left = pwm_left * potentiometer;
    pwm_right = pwm_right * potentiometer;
      
    // pwm output
    analogWrite(motor_left1, pwm_left);
    analogWrite(motor_right3, pwm_right);
  
    //write logic2, logic4, freq6, freq8
    freq6 = map(pwm_left, 0, 255, 0, 20000); //20KHz //31.372KHz
    freq8 = map(pwm_right, 0, 255, 0, 20000); //20KHz //31.372KHz
    itoa(freq8, str, 10);
    Wire.beginTransmission(8);
    Wire.write(str); //send freq8 to slave arduino
    Wire.endTransmission();

    if(freq6<100) noTone(freq6_pin);
    else tone(freq6_pin, freq6);
 
    // print out the output values for testing
    Serial.println();
    Serial.println("LEFT");
    Serial.println(pwm_left);
    int lo1 = digitalRead(logic2);
    if(lo1) Serial.println("HIGH");
    else Serial.println("LOW");
    Serial.println(freq6);

    Serial.println();
    Serial.println("RIGHT");
    Serial.println(pwm_right);
    int lo2 = digitalRead(logic4);
    if(lo2) Serial.println("HIGH");
    else Serial.println("LOW");
    Serial.println(freq8);
  }
  else{
    analogWrite(motor_left1, 0);
    analogWrite(motor_right3, 0);
  }
}

void left_logic(float angle, float intensity){
  if( (angle >= 0 && angle <= 1.575) || (angle <= -1.565 && angle >= -3.145) || angle > 3.14){ //if quadrant 1 and 3
    pwm_left = 255; //50 per cent
    
  }else if(angle > 1.5707 && angle <= 3.14){ //if quadrant 2
    pwm_left = abs(angle - 2.356)*(255/0.79); //rad 2.356 = degrees 135
    
  }else if(angle < 0 && angle > -1.5707){ //if quadrant 4
    pwm_left = abs(angle + 0.785)*(255/0.79); //rad -0.785 = degrees -45
    
  } 

  if(angle > -0.785 && angle < 2.356){
    digitalWrite(logic2, LOW); //sets logic2 to 0
    delay(10);
  }
  else{
    digitalWrite(logic2, HIGH); //sets logic2 to 1
    delay(10);
  }

  pwm_left = pwm_left * intensity;
}

void right_logic(float angle, float intensity){
  if( (angle >= 1.565 && angle <= 3.145) || (angle <= 0 && angle >= -1.575)){ //if quadrant 2 and 4 
    pwm_right = 255; //50 per cent
    
  }else if(angle > 0 && angle < 1.5707){ //if quadrant 1
    pwm_right = abs(angle - 0.785)*(255/0.79); //rad 0.785 = degrees 45
    
  }else if(angle < -1.5707 && angle > -3.1415){ //if quadrant 3
    pwm_right = abs(angle + 2.356)*(255/0.79); //rad -2.356 = degrees -135
  }

  if(angle > -2.356 && angle < 0.785){
    digitalWrite(logic4, LOW); //sets logic4 to 0
    delay(10);
  }
  else{
    digitalWrite(logic4, HIGH); //sets logic4 to 1
    delay(10);
  }

  pwm_right = pwm_right * intensity;
}
