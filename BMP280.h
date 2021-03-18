#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;

//Variable declerations (Including filter Components)
double bmpPresGround {}, bmpPres {}, bmpTemp {}, bmpAltRaw {}, bmpAltFiltered {};
const int bmpAltFilterSumCount {75};
static float bmpAltFilterSum {0};
static byte bmpAltFilterIndex {0};
static float bmpAltFilterReadings[bmpAltFilterSumCount];
static byte bmpAltFilterCount {0};

void bmpInitialise()	{
	if (bmp.begin()) {
		Serial.println("//BMP280 Initialisation Successful");
   delay(1000);
		bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, //Set Sensor Modes
						Adafruit_BMP280::SAMPLING_X2, //Temperature Oversampling as recomended 
						Adafruit_BMP280::SAMPLING_X16, //Pressure Oversampling Max
            Adafruit_BMP280::FILTER_OFF, //No Onboard Filtering
						Adafruit_BMP280::STANDBY_MS_500);

		Serial.println("//BMP280 Calibration Begin"); //Calibrate Ground Pressure
		delay(5000);
		for (int i {}; i < 500; i++)	{
			bmpPresGround += bmp.readPressure();
			if (i % 50 == 0)	{
				Serial.print(bmp.readPressure()); Serial.print("Pa at ");
				Serial.print(bmp.readTemperature() + 273.15); Serial.println("K");
			}
			delay(50);
		}
		bmpPresGround /= 500;
		Serial.print("//Ground Pressure = "); Serial.print(bmpPresGround); Serial.println("Pa");
		Serial.println("//BMP280 Calibration Complete");
		Serial.println("=================================================");
	}
	else	{
		Serial.println("//BMP280 Initailisation Failed");
		while (1);	
	}
}

double bmpAltitudeRaw()	{ //Returns Raw Altitude Data
  bmpTemp = bmp.readTemperature() + 273.15;
	bmpAltRaw = (((pow(bmpPresGround / bmp.readPressure(), 0.19022256) - 1) * bmpTemp) / 0.0065);
  return bmpAltRaw;
}

double  bmpAltitudeFilt()	{ //Use Raw Data in Filter Algorithm. Returns Filtered Altitude Data
	bmpAltFilterSum -= bmpAltFilterReadings[bmpAltFilterIndex];
	bmpAltFilterReadings[bmpAltFilterIndex] = bmpAltRaw;
	bmpAltFilterSum += bmpAltFilterReadings[bmpAltFilterIndex];
	bmpAltFilterIndex++;
	bmpAltFilterIndex = bmpAltFilterIndex % bmpAltFilterSumCount;
	if (bmpAltFilterCount < bmpAltFilterSumCount)  {
		bmpAltFilterCount++;
	}
	bmpAltFiltered = bmpAltFilterSum / bmpAltFilterCount;
 return bmpAltFiltered;
}
