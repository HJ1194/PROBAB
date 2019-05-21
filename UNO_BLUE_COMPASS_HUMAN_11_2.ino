#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>

SoftwareSerial BT(10, 11);  


Adafruit_LSM303 lsm;
const float Pi = 3.14159;

int heading_map;

void setup() {
  
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Wire.begin();
  Serial.begin(9600);
  BT.begin(9600);
 
 if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM303. Check your wiring!");
    while (1);
  }
}

void loop() { 
  
  lsm.read();
  int heading = (atan2((int)lsm.magData.y, (int)lsm.magData.x) * 180) / Pi;

  if(heading < 0) {
    heading = 360 + heading;
  }
  
  heading_map = map(heading,0,360,0,255);

  Serial.print("HUMAN_HEADING : : ");
  Serial.println(heading);

  Serial.print("HUNMAN_Heading_MAPPGIN : ");
  Serial.println(heading_map);

  BT.write(heading_map);

  delay(500);
}







