#include <HardwareSerial.h>
#define LED 2

String received;
HardwareSerial LoRaSerial(2);  // accessing Uart
#define RX_PIN 16
#define TX_PIN 17
#define LORA_BAUD 115200 

void setup() {
    Serial.begin(115200);  // beginning baud rate 
    LoRaSerial.begin(LORA_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);  // LoRa module UART initialization 

    delay(2000);  // in case the lora module takes long to initialize 
    Serial.println("\nRYLR897 Test");

  Serial.println("Try AT commands now");

  LoRaSerial.print("AT\r\n");
  delay(1000);
  
  LoRaSerial.print("AT+ADDRESS=116\r\n");   //needs to be unique
  delay(1000);   //wait for module to respond

  LoRaSerial.print("AT+NETWORKID=10\r\n");   //needs to be same for receiver and transmitter
  delay(1000);   //wait for module to respond

  LoRaSerial.print("AT+BAND=915000000\r\n");    //Bandwidth set to 868.5MHz
  delay(1000);   //wait for module to respond

  LoRaSerial.print("AT+PARAMETER=10,7,1,7\r\n");    //For Less than 3Kms
  delay(1000);   //wait for module to respond
  
  LoRaSerial.print("AT+PARAMETER?\r\n");    //For Less than 3Kms
  //Serial.print("AT+PARAMETER=10,7,1,7\r\n");    //For More than 3Kms
  delay(1000);   //wait for module to respond
  
  LoRaSerial.print("AT+BAND?\r\n");    //Bandwidth set to 868.5MHz
  delay(1000);   //wait for module to respond
  
  LoRaSerial.print("AT+NETWORKID?\r\n");   //needs to be same for receiver and transmitter
  delay(1000);   //wait for module to respond
  
  LoRaSerial.print("AT+ADDRESS?\r\n");   //needs to be unique
  delay(1000);   //wait for module to respond
   
   pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
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
    if(received.startsWith("+RCV")){
     if(received[11]=='1'){  //in this case our single received byte would always be at the 11th position
        digitalWrite(LED,HIGH);
      }
        if(received[11]=='0'){  //in this case our single received byte would always be at the 11th position
        digitalWrite(LED,LOW);
      }
  }
         }
}
