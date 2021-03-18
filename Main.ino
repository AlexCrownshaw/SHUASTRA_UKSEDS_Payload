#include "Data.h"
#include "BMP280.h"
#include "MPU9250AHRS.h"

unsigned long timeLast {};

//define DEBUG
#define AHRSVIS

void setup() {
  Serial.begin(115200);
  delay(1000);
  while (!Serial) {
  }
  Serial.println("//Awaiting Initilisation Command");
  while (Serial.available() == 0);
  String initialiseCommand = Serial.readString();
  Serial.println(initialiseCommand);
  if (initialiseCommand != "start") {
    Serial.print("//Error Reset System");
  }
  else  {
    Serial.println("//System Initialisation Begin");
    Serial.println("=================================================");
    delay(2000);
    bmpInitialise();
    imuInitialise();

    Serial.println("//Awaiting Program Run Command");
    while (Serial.available() == 0);
    String runCommand = Serial.readString();
    Serial.println(runCommand);
    if (runCommand == "run")  {
      Serial.println("//Program Start");
      Serial.println("=================================================");
      delay(2000);
    }
    else  {
      Serial.print("//Error Reset System");
    }
  }
}

void loop() {
  if (millis() - timeLast > 10) {
    getPacket(millis(),bmpAltitudeRaw(), bmpAltitudeFilt(), ahrsCompute());

    #if defined (DEBUG) //Prints Packet to Serial Monitor
      printPacket(packet);
    #elif defined (AHRSVIS) 
      ahrsVisual();
    #endif
    
    timeLast = millis();
  }
}
