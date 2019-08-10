#include <Adafruit_Sensor.h>
#include <SigFox.h>
#include <ArduinoLowPower.h>
#include "conversions.h"

// defines pins numbers
const int trigPin = 0;
const int echoPin = 1;

#define DEBUG 1
#define SLEEPTIME  15  * 1000

uint16_t take_measurement(){
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  uint16_t distance= duration*0.034/2;

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

void alarmEvent0() {
  if (DEBUG) {
    Serial.print("alarm");
  }
}

uint16_t readVoltage(){
  analogReadResolution(10);
  analogReference(AR_INTERNAL1V0); //AR_DEFAULT: the default analog reference of 3.3V // AR_INTERNAL1V0: a built-in 1.0V reference

  // read the input on analog pin 0:
  int sensorValue = analogRead(ADC_BATTERY);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
  uint16_t voltage = (uint16_t)(sensorValue * (3.25 / 1023.0)*1000); //millivolt
  // print out the value you read:
  if (DEBUG) {
    Serial.print(voltage);
    Serial.println("mV");
  }
  return voltage;
}

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input 

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

void loop()
{
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  uint16_t distance = take_measurement();

  if (DEBUG){
    // Prints the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.println(distance);
  }

  float voltage = readVoltage();

  // send once every 24 hours
  send_distance(distance, voltage);

  LowPower.sleep(SLEEPTIME);
}
