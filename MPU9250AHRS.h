#include "quaternionFilters.h"
#include "MPU9250.h"

using namespace std;

int intPin = 5; //Set Interupt Pin

#define I2Cclock 400000 //400KHz I2C Clock Speed
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0 //Define I2C Address

MPU9250 IMU(MPU9250_ADDRESS, I2Cport, I2Cclock); //Declare IMU

void imuInitialise()  {
  Serial.println("//IMU Initialisation Begin");
  delay(2000);
  Wire.begin();
  pinMode(intPin, INPUT); //Setup Interupt Pin and Set it Low
  digitalWrite(intPin, LOW);

  // Check Sensor register to test communication
  byte c = IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("//IMU Sensor Detected on Address 0x");
  Serial.println(c, HEX);
  delay(1500);

  if (c == 0x71)  { //WHO_AM_I should return 0x71
    Serial.println("//MPU9250 Detected and Online");
    delay(2000);

    byte d = IMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("//Magnetometer Sensor Detected on Address 0x");
    Serial.println(d, HEX);

    if (d != 0x48)  {
      Serial.println("//Failed to detect Magnotometer");
      while (1);
    }
    Serial.println();
    delay(2000);

    Serial.println("//Accelerometer Self Test Begin");
    IMU.MPU9250SelfTest(IMU.selfTest);  //Test accelerometer sensor for obvious faults
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(IMU.selfTest[0],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(IMU.selfTest[1],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(IMU.selfTest[2],1); Serial.println("% of factory value");
    Serial.println();
    delay(2000);

    Serial.println("//Gyroscope Self Test Begin");  //Test gyroscope sensor for obvious faults
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(IMU.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(IMU.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(IMU.selfTest[5],1); Serial.println("% of factory value");
    Serial.println();
    delay(2000);

    IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);

    IMU.initMPU9250();  // Initialize device for active mode read of acclerometer, gyroscope, and
                        // temperature
    Serial.println("MPU9250 Initialized for Active Data Mode");
    delay(2000);

    IMU.initAK8963(IMU.factoryMagCalibration);  //Initialise Magnetometer for active data mode
    Serial.println("AK8963 Initialized for Active Data Mode");
    delay(2000);

    IMU.magCalMPU9250(IMU.magBias, IMU.magScale);
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(IMU.magBias[0]);
    Serial.println(IMU.magBias[1]);
    Serial.println(IMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(IMU.magScale[0]);
    Serial.println(IMU.magScale[1]);
    Serial.println(IMU.magScale[2]);
    Serial.println();
    delay(2000);

    Serial.println("//Magnetometer Sensitivity Adjustment values");
    Serial.print("X = ");
    Serial.println(IMU.factoryMagCalibration[0], 2);
    Serial.print("Y = ");
    Serial.println(IMU.factoryMagCalibration[1], 2);
    Serial.print("Z = ");
    Serial.println(IMU.factoryMagCalibration[2], 2);
    Serial.println();
    delay(1000);
   
    IMU.getAres();  //Gets sensor resolutions (hard values in .h file)
    IMU.getGres();
    IMU.getMres();

    Serial.println("//IMU Initialisation Successful");
    Serial.println("=================================================");
    delay(2000);
  }
  else  {
    Serial.println("IMU Initialisation Failed");
    while (1);
  }
}

std::array <double, 19> ahrsCompute() {
  std::array <double, 19> ahrsVals;

  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) { //Check if data is ready on interupt pin
    IMU.readAccelData(IMU.accelCount);  //Reads ADC values for accelerometer
    IMU.ax = (double)IMU.accelCount[0] * IMU.aRes; //Calculates acceleration values in g's
    IMU.ay = (double)IMU.accelCount[1] * IMU.aRes;
    IMU.az = (double)IMU.accelCount[2] * IMU.aRes;
    
    IMU.readGyroData(IMU.gyroCount);  //Reads ADC values for gyroscope
    IMU.gx = (double)IMU.gyroCount[0] * IMU.gRes * DEG_TO_RAD; //Calculates angular rate values in rad/s
    IMU.gy = (double)IMU.gyroCount[1] * IMU.gRes * DEG_TO_RAD;
    IMU.gz = (double)IMU.gyroCount[2] * IMU.gRes * DEG_TO_RAD;
    
    IMU.readMagData(IMU.magCount);  //Reads ADC values for magnotometer
    IMU.mx = (double)IMU.magCount[0] * IMU.mRes * IMU.factoryMagCalibration[0] - IMU.magBias[0];
    IMU.my = (double)IMU.magCount[1] * IMU.mRes * IMU.factoryMagCalibration[1] - IMU.magBias[1];
    IMU.mz = (double)IMU.magCount[2] * IMU.mRes * IMU.factoryMagCalibration[2] - IMU.magBias[2];

    IMU.updateTime(); //Required before Madgwick update

    MadgwickQuaternionUpdate(IMU.ax, IMU.ay, IMU.az, IMU.gx, IMU.gy, IMU.gz, IMU.mx, IMU.my, IMU.mz, IMU.deltat);

    IMU.tempCount = IMU.readTempData();  // Read the adc values
    IMU.temperature = ((float) IMU.tempCount) / 333.87 + 21.0;  // Temperature in degrees Centigrade

    std:: array <double, 4> q {*getQ(), (*(getQ() + 1)), (*(getQ() + 2)), (*(getQ() + 3))}; //store quaternion to an array
    qw = q[0];
    qx = q[1];
    qy = q[2];
    qz = q[3];

    IMU.yaw   = atan2(2.0f * (q[1] * q[2] + q[0]
                    * q[3]), q[0] * q[0] + q[1]
                    * q[1] - q[2] * q[2] - q[3]
                    * q[3]);
    IMU.pitch = asin(2.0f * (q[1] * q[3] - q[0]
                    * q[2]));
    IMU.roll  = -atan2(2.0f * (q[0] * q[1] + q[2])
                    * q[3], q[0] * q[0] - q[1]
                    * q[1] - q[2] * q[2] + q[3]
                    * q[3]);
    IMU.pitch *= RAD_TO_DEG;
    IMU.yaw   *= RAD_TO_DEG;
    IMU.yaw  -= 8.5;  //Corrects for magnetic declination
    IMU.roll *= RAD_TO_DEG;

    ahrsVals[0] = IMU.yaw;
    ahrsVals[1] = IMU.pitch;
    ahrsVals[2] = IMU.roll;
    
    for (int i {3}; i <= 7; i++ )  { //Stores quaternion components to AHRS array
      ahrsVals[i] = q[i - 3];
    }
    
    ahrsVals[8] = IMU.ax; //Stores raw sensor data to AHRS array
    ahrsVals[9] = IMU.ay;
    ahrsVals[10] = IMU.az;
    ahrsVals[11] = IMU.gx;
    ahrsVals[12] = IMU.gz;
    ahrsVals[13] = IMU.gz;
    ahrsVals[14] = IMU.mx;
    ahrsVals[15] = IMU.my;
    ahrsVals[16] = IMU.mz;
      
//    Serial.print(IMU.yaw);
//    Serial.print("\t");
//    Serial.print(IMU.pitch);
//    Serial.print("\t");
//    Serial.println(IMU.roll);            
  }
  return ahrsVals;
}

std::array <double, 3> bodyToInertial() {
  std::array <double, 3> accInertial;
  
}
