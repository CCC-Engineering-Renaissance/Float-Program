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

// SensorData object used to store time, pressure and depth
struct SensorData {
  int time;
  float pressure;
  float depth;
};


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
  digitalWrite(enPin, HIGH);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  pinMode(LED, OUTPUT);

  // SENSOR
  // initializes the Wire library and join the I2C bus as a controller or a peripheral
  Wire.begin();

  // Initialize pressure sensor
  // Returns true if initialization was successful
  // We can't continue with the rest of the program unless we can initialize the sensor
  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }

  sensor.setModel(MS5837::MS5837_02BA);
  sensor.setFluidDensity(997);  // kg/m^3 (freshwater, 1029 for seawater)
}

void loop() {
  bool stop = false;
  while (!stop) {
    int motor = 1;
    // in meters
    int aboveSeaLevel = 98;
    int surface;
    int dataCount = 0;
    int profileCount = 0;
    int arbitraryNum = 6250;
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

        // B E G I N N I N G
        if (received[11] == '1') {  //in this case our single received byte would always be at the 11th position
          // initial reading
          sensor.read();
          SensorData begin;
          begin.time = millis() / 1000;
          begin.pressure = sensor.pressure();
          begin.depth = sensor.depth() - aboveSeaLevel;

          surface = begin.depth;

          // print data (this could probably be a function)
          Serial.print("Beginning Time: ");
          Serial.print(begin.time);
          Serial.println(" s");

          Serial.print("Beginning Pressure: ");
          Serial.print(begin.pressure);
          Serial.println(" mbar");

          Serial.print("Beginning Depth: ");
          Serial.print(begin.depth);
          Serial.println(" m");

          // turn on stepper motor
          digitalWrite(enPin, LOW);
          digitalWrite(dirPin, LOW);

          // should go down to 2.5 m
          // it doesn't yet
          for (int x = 0; x < 14725; x++) {
            // Motor
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(500);  // by changing this time delay between the steps we can change the rotation speed
            digitalWrite(stepPin, LOW);
            delayMicroseconds(500);
          }

          // collecting data
          while (profileCount < 2) {
            while (dataCount < 10) {
              sensor.read();

              SensorData current;
              current.time = millis() / 1000;
              current.pressure = sensor.pressure();
              current.depth = sensor.depth() - aboveSeaLevel;

              // print data (this could probably be a function)
              Serial.print("Time: ");
              Serial.print(current.time);
              Serial.println(" s");

              Serial.print("Pressure: ");
              Serial.print(current.pressure);
              Serial.println(" mbar");

              Serial.print("Depth: ");
              Serial.print(current.depth);
              Serial.println(" m");

              // adjust higher
              if (current.depth < 2) {
                digitalWrite(dirPin, HIGH);  // direction is up
                for (int x = 0; x < arbitraryNum; x++) {
                  // Motor
                  digitalWrite(stepPin, HIGH);
                  delayMicroseconds(500);
                  digitalWrite(stepPin, LOW);
                  delayMicroseconds(500);
                }
                // adjust lower
              } else if (current.depth > 3) {
                digitalWrite(dirPin, LOW);  // direction is down
                for (int x = 0; x < arbitraryNum; x++) {
                  // Motor
                  digitalWrite(stepPin, HIGH);
                  delayMicroseconds(500);
                  digitalWrite(stepPin, LOW);
                  delayMicroseconds(500);
                }
                // within range
              } else {
                dataCount++;
              }
            }

            // go up to surface
            digitalWrite(dirPin, HIGH);  // direction is up
            while (sensor.depth() < surface - 0.2) {
              // Motor
              sensor.read();
              digitalWrite(stepPin, HIGH);
              delayMicroseconds(500);  // by changing this time delay between the steps we can change the rotation speed
              digitalWrite(stepPin, LOW);
              delayMicroseconds(500);
            }

            profileCount++;
          }
          stop = true;




          /*
          // this just makes float go up and down
      
          while (motor == 1) {
          received = LoRaSerial.readString();
          Serial.print(received);
          digitalWrite(enPin, LOW);
          digitalWrite(LED, HIGH);
          digitalWrite(dirPin, LOW);

          // 0 is stop everything
          if (received[11] == '0'){
            digitalWrite(LED, LOW);
            digitalWrite(enPin, HIGH);
            break;
          }

          // this is 1
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

        */
        }
      }
    }
  }
}