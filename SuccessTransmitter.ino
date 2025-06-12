#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <HardwareSerial.h>
#define LED 2

String received, jsonString;
HardwareSerial LoRaSerial(2);  // accessing Uart

#define RX_PIN 16
#define TX_PIN 17
#define RST_PIN 5
#define LORA_BAUD 115200

struct SensorData {
  float pressure;
  float depth;
  int time;
};

void setup() {
  Serial.begin(115200);                                     // beginning baud rate
  LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);  // LoRa module UART initialization

  delay(2000);  // in case the lora module takes long to initialize
  Serial.println("\nRYLR897 Test");

  Serial.println("Try AT commands now");

  LoRaSerial.print("AT+PARAMETER=10,7,1,7\r\n");  //For Less than 3Kms
  //Serial.print("AT+PARAMETER=10,7,1,7\r\n");    //For More than 3Kms
  delay(1000);  //wait for module to respond

  LoRaSerial.print("AT+BAND=915000000\r\n");  //Bandwidth set to 868.5MHz
  delay(1000);                                //wait for module to respond

  LoRaSerial.print("AT+ADDRESS=115\r\n");  //needs to be unique
  delay(1000);                             //wait for module to respond

  LoRaSerial.print("AT+NETWORKID=10\r\n");  //needs to be same for receiver and transmitter
  delay(1000);                              //wait for module to respond

  LoRaSerial.print("AT+PARAMETER?\r\n");  //prints the current parameters
  delay(1000);
  /*
  digitalWrite(RST_PIN, HIGH);
  delay(1000);
  digitalWrite(RST_PIN, LOW);
  delay(1000);
  digitalWrite(RST_PIN, HIGH);
  */
  //wait for module to respond
}

void loop() {
  // Forward Serial Monitor input to LoRa module sends the serial monitor input to the lora module
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    LoRaSerial.print(command + "\r\n");  // Send to LoRa module with CRLF
  }

  // reads in LoRa module response to Serial Monitor
  while (LoRaSerial.available()) {
    received = LoRaSerial.readString();
    Serial.print(received);

    int end = received.indexOf("}");
    jsonString = received.substring(12, end + 1);
    Serial.print("Decoded string: ");
    Serial.println(jsonString);  // Should print: {"t":123,"p":1.23,"d":4.56}
  }
  /*
  LoRaSerial.println("AT+SEND=116,1,1");
  delay(1000);

  LoRaSerial.println("AT+SEND=116,1,0");
  delay(1000);
  */
}