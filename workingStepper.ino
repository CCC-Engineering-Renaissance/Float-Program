/*   
 *   Basic example code for controlling a stepper without library
 *      
 *   by Dejan, https://howtomechatronics.com
 */
#include <Arduino.h>

#define LED 2

// defines pins
#define stepPin 18
#define dirPin 19
#define enPin 21
 
void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  digitalWrite(enPin, LOW);
  pinMode(dirPin,OUTPUT);
  pinMode(enPin, OUTPUT);
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
}

void loop() {
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  for(int x = 0; x < 1600; x++) {
    digitalWrite(stepPin,HIGH); 
    digitalWrite(LED, HIGH);
    Serial.println("LED is on");
    delayMicroseconds(750);    // by changing this time delay between the steps we can change the rotation speed
    digitalWrite(stepPin,LOW); 
    digitalWrite(LED, LOW);
    Serial.println("LED is off");
    delayMicroseconds(750); 
  }
  delay(1000); // One second delay
  
  digitalWrite(dirPin,LOW); //Changes the rotations direction
  // Makes 400 pulses for making two full cycle rotation
  for(int x = 0; x < 1600; x++) {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(750);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(750);
  }
  delay(1000);
}