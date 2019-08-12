#include <Arduino.h>
#include <Wire.h>
#include <SigFox.h>
#include "RTCZero.h"
#include <ArduinoLowPower.h>


// defines pins numbers
const int greenPin = 2;
const int redPin = 3;
volatile int alarm_source = 0;

#define DEBUG 0
#define SLEEPTIME  15  * 1000

#define header_H    0x55 //Header
#define header_L    0xAA //Header
#define device_Addr 0x11 //Address
#define data_Length 0x00 //Data length
#define get_Dis_CMD 0x02 //Command: Read Distance
#define checksum    0x12 //Checksum

unsigned char CMD[6]={
  header_H,header_L,device_Addr,data_Length,get_Dis_CMD,checksum}; //Distance command package

void redOn(){
  digitalWrite(redPin, HIGH);
  digitalWrite(greenPin, LOW);
  delay(500);
  digitalWrite(redPin, LOW);
}

void blink(int times, int pin){
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  int i;

  for (i = 0; i < times; i++){
    digitalWrite(pin, HIGH);
    delay(100);
    digitalWrite(pin, LOW);
    delay(100);
  }
}

void flashBatteryVoltage(const float batteryVoltage){
  int integral = (int)batteryVoltage;
  float decimal = batteryVoltage - integral;
  int trimmed_ecimal = (int)(decimal*10);
  blink(integral, redPin);
  delay(500);
  blink(trimmed_ecimal, greenPin);
}

void alarmEvent0() {
  alarm_source = 0;
}

float readBatteryVoltage() {
    analogReadResolution(10);
    analogReference(AR_INTERNAL1V0); //AR_DEFAULT: the default analog reference of 3.3V // AR_INTERNAL1V0: a built-in 1.0V reference
    // read the input on analog pin 0:
    int sensorValue = analogRead(ADC_BATTERY);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
    float voltage = sensorValue * (3.25 / 1023.0);
    return voltage;
}

unsigned int readDistance(){
  unsigned int distance=0;
  int i = 0;
  unsigned char rx_data[8];
  for(i=0;i<6;i++){
    Serial1.write(CMD[i]);
  }
  delay(1000);  //Wait for the result
  i=0;
  while (Serial1.available()){  //Read the return data (Note: this demo is only for the reference, no data verification)
    rx_data[i++]=(Serial1.read());
  }
  distance=((rx_data[5]<<8)|rx_data[6]); //Read the distance value
  return distance;
}

void send_distance(uint16_t distance, uint16_t voltage) {
  // Start the module
  SigFox.begin();
  // Wait at least 30mS after first configuration (100mS before)
  delay(100);
  // Clears all pending interrupts
  SigFox.status();
  delay(1);

  SigFox.beginPacket();
  SigFox.write(distance);
  SigFox.write((unsigned short)voltage);
  int ret = SigFox.endPacket();  // send buffer to SIGFOX network

  if (DEBUG){
    if (ret > 0) {
      Serial.println("No transmission");
    } else {
      Serial.println("Transmission ok");
    }

    Serial.println(SigFox.status(SIGFOX));
    Serial.println(SigFox.status(ATMEL));
  }
  SigFox.end();
}


void setup() {
  Serial1.begin(19200);  //Serial1: Ultrasonic Sensor Communication Serial Port, Buadrate: 19200
  pinMode(greenPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(redPin, OUTPUT); // Sets the echoPin as an Input
  pinMode(4, OUTPUT); // Sets the echoPin as an Input
  digitalWrite(4, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(redPin, LOW);

  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, alarmEvent0, CHANGE);

  if (DEBUG){
    Serial.begin(9600);
    while (!Serial) {};
  }

  if (!SigFox.begin()) {
    Serial.println("Shield error or not present!");
    return;
  }
  // Enable debug led and disable automatic deep sleep
  if (DEBUG) {
    SigFox.debug();
  }
  delay(100);

  // Send the module to the deepest sleep
  SigFox.end();
}

void loop() {
  float batteryVoltage = readBatteryVoltage();
  flashBatteryVoltage(batteryVoltage);
  int distance = readDistance();
  if (DEBUG){
    // Prints the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.println(distance);
    Serial.print("Voltage: ");
    Serial.println(batteryVoltage);
  }
  send_distance(distance, batteryVoltage*1000);

  LowPower.deepSleep(SLEEPTIME);
}
