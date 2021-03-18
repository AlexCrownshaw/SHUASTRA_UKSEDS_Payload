#include <arduino.h>
#include <array>

using namespace std;

unsigned long printLast {}, ahrsVisLast {};
const int packetSize {19};
double packet [packetSize];
double qw {}, qx {}, qy {}, qz{};

void getPacket(double timeStamp, double altBaroRaw, double altBaroFilt, std::array <double, 19> ahrsVals)  {  //Calls All Sensor Data Functions and Collects in a Packet Array
  packet[0] = timeStamp / 1000;
  packet[1] = altBaroRaw;
  packet[2] = altBaroFilt;
  for (int i {}; i <= ahrsVals.size(); i++)  {
    packet[i + 3] = ahrsVals[i];
  }
}

void printPacket(double packet [packetSize])  { //Prints Packet Array to Serial Monitor with 6 S.F.
  if (millis() - printLast > 100)   {
    for (int i {}; i < packetSize; i++)   {
      Serial.print(packet[i], 6);
      Serial.print("\t");
    }
    Serial.println();
    printLast = millis();
  }
}

void ahrsVisual() {
  if (millis() - ahrsVisLast > 100) {
    Serial.print("Quaternion: ");
    Serial.print(qw, 4);
    Serial.print(", ");
    Serial.print(qx, 4);
    Serial.print(", ");
    Serial.print(qy, 4);
    Serial.print(", ");
    Serial.println(qz, 4);
    ahrsVisLast = millis();
  }
}
