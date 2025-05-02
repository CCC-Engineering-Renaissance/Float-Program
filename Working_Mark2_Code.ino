#include <HardwareSerial.h>
#include <Arduino.h>

#define LED 2

// defines pins
// LoRa
String received;
HardwareSerial LoRaSerial(2);  // accessing Uart
#define RX_PIN 16
#define TX_PIN 17
#define LORA_BAUD 115200
// Motor
int motor_cycles = 0;
#define stepPin 18
#define dirPin 19
#define enPin 21

void setup() {
  // LORA
  Serial.begin(115200);                                     // beginning baud rate
  LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);  // LoRa module UART initialization

  delay(1000);  // in case the lora module takes long to initialize
  Serial.println("\nRYLR897 Test");

  Serial.println("Try AT commands now");

  LoRaSerial.print("AT\r\n");
  delay(1000);

  LoRaSerial.print("AT+ADDRESS=116\r\n");  //needs to be unique
  delay(1000);                             //wait for module to respond

  LoRaSerial.print("AT+NETWORKID=10\r\n");  //needs to be same for receiver and transmitter
  delay(1000);                              //wait for module to respond

  LoRaSerial.print("AT+BAND=915000000\r\n");  //Bandwidth set to 868.5MHz
  delay(1000);                                //wait for module to respond

  LoRaSerial.print("AT+PARAMETER=10,7,1,7\r\n");  //For Less than 3Kms
  delay(1000);                                    //wait for module to respond

  LoRaSerial.print("AT+PARAMETER?\r\n");  //For Less than 3Kms
  //Serial.print("AT+PARAMETER=10,7,1,7\r\n");    //For More than 3Kms
  delay(500);  //wait for module to respond

  LoRaSerial.print("AT+BAND?\r\n");  //Bandwidth set to 868.5MHz
  delay(500);                        //wait for module to respond

  LoRaSerial.print("AT+NETWORKID?\r\n");  //needs to be same for receiver and transmitter
  delay(500);                             //wait for module to respond

  LoRaSerial.print("AT+ADDRESS?\r\n");  //needs to be unique
  delay(500);                           //wait for module to respond

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // MOTOR

  // Sets the two pins as Outputs
  pinMode(stepPin, OUTPUT);
  digitalWrite(enPin, LOW);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  pinMode(LED, OUTPUT);
  //delay(300000);
  /*
  digitalWrite(dirPin, HIGH);
  for (int x = 0; x < 14725; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  } 
    digitalWrite(dirPin, LOW);
  for (int x = 0; x < 7363; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(300);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(300);
  }
  */
}

void loop() {
  int motor = 1;
  // LORA
  // Forward Serial Monitor input to LoRa module sends the serial monitor input to the lora module
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    LoRaSerial.print(command + "\r\n");  // Send to LoRa module with CRLF
  }

  // reads in LoRa module response to Serial Monitor
  while (LoRaSerial.available()) {
    received = LoRaSerial.readString();
    Serial.print(received);
    if (received.startsWith("+RCV")) {
      if (received[11] == '1') {  //in this case our single received byte would always be at the 11th position
        while (motor == 1) {
          received = LoRaSerial.readString();
          Serial.print(received);
          digitalWrite(enPin, LOW);
          digitalWrite(LED, HIGH);
          digitalWrite(dirPin, LOW);
          if (received[11] == '0'){
            digitalWrite(LED, LOW);
            digitalWrite(enPin, HIGH);
            break;
          }
          for (int x = 0; x < 14725; x++) {
            digitalWrite(stepPin, HIGH);
            // LED
            digitalWrite(LED, HIGH);
            // Motor
            delayMicroseconds(500);  // by changing this time delay between the steps we can change the rotation speed
            digitalWrite(stepPin, LOW);
            delayMicroseconds(500);
          }
          //Serial.println("LED is on");
          delay(1000);  // One second delay


          digitalWrite(dirPin, HIGH);  //Changes the rotations direction
          // Makes 400 pulses for making two full cycle rotation
          for (int x = 0; x < 14725; x++) {
            digitalWrite(stepPin, HIGH);
            // Motor
            delayMicroseconds(500);
            digitalWrite(stepPin, LOW);
            delayMicroseconds(500);
          }
          //Serial.println("LED is off");
          delay(1000);

          if (received[11] == '0') {  //in this case our single received byte would always be at the 11th position
            digitalWrite(LED, LOW);
            digitalWrite(enPin, HIGH);
            motor = 0;
          }
        }
      }
    }
  }
}