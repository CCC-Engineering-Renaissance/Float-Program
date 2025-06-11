#include <HardwareSerial.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "MS5837.h"
#include <PID.h>

#define LED 2
MS5837 sensor;

// defines pins
// LoRa
String received;
HardwareSerial LoRaSerial(2);  // accessing Uart
#define RX_PIN 16
#define TX_PIN 17
#define LORA_BAUD 115200
// Motor
int motor_cycles = 0;
#define stepPin 5
#define dirPin 18
#define enPin 19

struct SensorData {
  int time;
  float pressure;
  float depth;
};

SensorData current[10];
String jsonString[2];

float M3_error = 0;
float M3_setpoint = 0;
float M3_previous = 0;
float M3_corrective_val = 0;
int first_run = 0;


StaticJsonDocument<JSON_OBJECT_SIZE(4)> doc1;
JsonArray c1 = doc1["c"].to<JsonArray>();
JsonArray t1 = doc1["t"].to<JsonArray>();
JsonArray p1 = doc1["p"].to<JsonArray>();
JsonArray d1 = doc1["d"].to<JsonArray>();

StaticJsonDocument<JSON_OBJECT_SIZE(4)> doc2;
JsonArray c2 = doc2["c"].to<JsonArray>();
JsonArray t2 = doc2["t"].to<JsonArray>();
JsonArray p2 = doc2["p"].to<JsonArray>();
JsonArray d2 = doc2["d"].to<JsonArray>();


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
  Wire.begin();

  // MOTOR

  // Sets the two pins as Outputs
  pinMode(stepPin, OUTPUT);
  digitalWrite(enPin, HIGH);
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

  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar2: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  sensor.setModel(MS5837::MS5837_02BA);
  sensor.setFluidDensity(997);
}

void loop() {
  digitalWrite(enPin, HIGH);
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
        Serial.println("begin");
        while (motor == 1) {
          received = LoRaSerial.readString();
          Serial.print(received);
          digitalWrite(enPin, LOW);
          digitalWrite(LED, HIGH);
          digitalWrite(dirPin, HIGH);  // Intake water
          first_run++;
          if (received[11] == '0') {
            digitalWrite(LED, LOW);
            digitalWrite(enPin, HIGH);
            break;
          }
          // Read from Sensor

          /*
          sensor.read();
          SensorData begin;
          begin.time = millis() / 1000;
          begin.pressure = sensor.pressure();
          begin.depth = sensor.depth();
          writeSensorData(begin);


          M3_previous = M3_error;
          M3_error = M3_setpoint - begin.depth;
          M3_corrective_val = pid(M3_error, M3_previous);
          */

          /*
          if (first_run == 1) {
            digitalWrite(dirPin, LOW);
            for (int x = 0; x < 14725; x++) {
              digitalWrite(stepPin, HIGH);
              delayMicroseconds(500);
              digitalWrite(stepPin, LOW);
              delayMicroseconds(500);
            }
            digitalWrite(dirPin, HIGH);
            for (int x = 0; x < 7363; x++) {
              digitalWrite(stepPin, HIGH);
              delayMicroseconds(500);
              digitalWrite(stepPin, LOW);
              delayMicroseconds(500);
            }
          }
          first_run++;
          */



          // Begin Run
          digitalWrite(dirPin, LOW);
          // Currently intakes water to start, sinks
          for (int x = 0; x < 14725; x++) {
            digitalWrite(stepPin, HIGH);
            // LED
            digitalWrite(LED, HIGH);
            // Motor
            delayMicroseconds(800);  // by changing this time delay between the steps we can change the rotation speed
            digitalWrite(stepPin, LOW);
            delayMicroseconds(800);
          }
          //Serial.println("LED is on");
          //delay(10000);  // One second delay

          // Read from Sensor
          for (int i = 0; i < 10; i++) {
            sensor.read();
            current[i].time = millis() / 1000;
            current[i].pressure = sensor.pressure() + 16;
            current[i].depth = (current.pressure[i] - 1013) / (9.8 * 997);
            delay(1000);
          }


          for (int i = 0; i < 5; i++) {
            c1.add("EX05");
            t1.add(current[i].time);
            p1.add(current[i].pressure);
            d1.add((int)(current[i].depth / 0.001) * 0.001);
          }

          for (int i = 5; i < 10; i++) {
            c2.add("EX05");
            t2.add(current[i].time);
            p2.add(current[i].pressure);
            d2.add((int)(current[i].depth / 0.001) * 0.001);
          }

          char cstring1[240];
          char cstring2[240];

          serializeJson(doc1, cstring1);
          serializeJson(doc2, cstring2);
        
          jsonString[0] = cstring1;
          jsonString[1] = cstring2;

          Serial.println(jsonString[0]);
          Serial.println(jsonString[1]);

          // Begin going back up
          digitalWrite(dirPin, HIGH);  //Changes the rotations direction
          // Makes 400 pulses for making two full cycle rotation
          for (int x = 0; x < 14725; x++) {
            digitalWrite(stepPin, HIGH);
            // Motor
            delayMicroseconds(800);
            digitalWrite(stepPin, LOW);
            delayMicroseconds(800);
          }

          delay(15000);

          // Send JSON
          Serial.println("sending");
          for (int i = 0; i < 2; i++) {
            LoRaSerial.print("AT+SEND=115,");
            LoRaSerial.print(256);
            LoRaSerial.print(",");
            LoRaSerial.println(jsonString[i]);
            delay(1000);
          }


          //Serial.println("LED is off");
          delay(5000);
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

