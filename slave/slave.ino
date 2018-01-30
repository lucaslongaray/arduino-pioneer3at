/*
  Slave program to receive a value to generate a tone()
*/

#include <Wire.h>

//Output FM
const byte freq8_pin = 10;
int freq8;

char str[5];

void setup() {
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);
  
  pinMode(freq8_pin, OUTPUT);
}

void loop() {
  delay(10);
} 

void receiveEvent(int howMany){
  int i = 0;
  
  if(Wire.available()){
    while(Wire.available() && i<5){
      str[i++] = Wire.read();  
    }
    str[i++] = '\0';
  }

  Serial.println("STR");
  Serial.println(str);
  freq8 = atoi(str);
  Serial.println(freq8);

  if(freq8<100) noTone(freq8_pin);
  else tone(freq8_pin, freq8);
  
}

