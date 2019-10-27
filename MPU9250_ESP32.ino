/*SDA --21 SCL --22 intrupt--14*/
#include "Wire.h"
#include "MPU9250_REG_MAP.h"

// ADO is normaly 0
#define ADO 0

#if ADO
#define MPU9250_ADDRESS 0x69  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#endif

#define AHRS true         // set to false for basic data read
#define SerialDebug true   // set to true to get Serial output for debugging

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins

bool newData = false;
bool newMagData = false;

int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}, magScale[3]  = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the MPU9250 gyro internal chip temperature in degrees Celsius
double Temperature, Pressure; // stores MS5637 pressures sensor pressure and temperature
float SelfTest[6];            // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = PI * (4.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll;
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

void setup()
{
  Serial.begin(115200);
  delay(4000);

  //Wire.begin(); //(SDA, SCL) (21,22) are default on ESP32, 400 kHz I2C clock

  Wire.begin(21, 22); // 21/22 are default on ESP32
  Wire.setClock(400000); // choose 400 kHz I2C rate

  delay(1000);

  I2Cscan();// look for I2C devices on the bus

  // Read the WHO_AM_I register, this is a good test of communication
  Serial.println("MPU9250 9-axis motion sensor...");
  uint8_t c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x73, HEX);

  delay(1000);

  if (c == 0x73 || (c == 0x68))
  {
    Serial.println("MPU9250 is online...");

    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2], 1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5], 1); Serial.println("% of factory value");
    delay(1000);

    // get sensor resolutions, only need to do this once
    getAres();
    getGres();
    getMres();

    Serial.println(" Calibrate gyro and accel");
    accelgyrocalMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
    Serial.println("accel biases (mg)"); Serial.println(1000.*accelBias[0]); Serial.println(1000.*accelBias[1]); Serial.println(1000.*accelBias[2]);
    Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);

    delay(1000);

    initMPU9250();
    Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);

    delay(1000);

    // Get magnetometer calibration from AK8963 ROM
    initAK8963(magCalibration); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

    magcalMPU9250(magBias, magScale);
    Serial.println("AK8963 mag biases (mG)"); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]);
    Serial.println("AK8963 mag scale (mG)"); Serial.println(magScale[0]); Serial.println(magScale[1]); Serial.println(magScale[2]);
    delay(2000); // add delay to see results before serial spew of data

    if (SerialDebug) {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
    }

    delay(1000);

    attachInterrupt(intPin, myinthandler, RISING);  // define interrupt for INT pin output of MPU9250

  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while (1) ; // Loop forever if communication doesn't happen
  }

}

void loop()
{
  // If intPin goes high, all data registers have new data
  if (newData == true) { // On interrupt, read data
    newData = false;  // reset newData flag
    readMPU9250Data(MPU9250Data); // INT cleared on any read

    // Now we'll calculate the accleration value into actual g's
    ax = (float)MPU9250Data[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
    ay = (float)MPU9250Data[1] * aRes - accelBias[1];
    az = (float)MPU9250Data[2] * aRes - accelBias[2];

    // Calculate the gyro value into actual degrees per second
    gx = (float)MPU9250Data[4] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)MPU9250Data[5] * gRes;
    gz = (float)MPU9250Data[6] * gRes;

    newMagData = (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01);
    if (newMagData == true) { // wait for magnetometer data ready bit to be set
      readMagData(magCount);  // Read the x/y/z adc values

      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental corrections
      mx = (float)magCount[0] * mRes * magCalibration[0] - magBias[0]; // get actual magnetometer value, this depends on scale being set
      my = (float)magCount[1] * mRes * magCalibration[1] - magBias[1];
      mz = (float)magCount[2] * mRes * magCalibration[2] - magBias[2];
      mx *= magScale[0];
      my *= magScale[1];
      mz *= magScale[2];
    }

    for (uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
      Now = micros();
      deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
      lastUpdate = Now;

      sum += deltat; // sum for averaging filter update rate
      sumCount++;

      MadgwickQuaternionUpdate(-ax, ay, az, gx * pi / 180.0f, -gy * pi / 180.0f, -gz * pi / 180.0f,  my,  -mx, mz);
    }
  }

  // Serial print and/or display at 0.5 s rate independent of data rates
  delt_t = millis() - count;
  if (delt_t > 50) {
    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    pitch = -asin(a32);
    roll  = atan2(a31, a33);
    yaw   = atan2(a12, a22);
    pitch *= 180.0f / pi;
    yaw   *= 180.0f / pi;
    yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if (yaw < 0) yaw   += 360.0f; // Ensure yaw stays between 0 and 360
    roll  *= 180.0f / pi;
    lin_ax = ax + a31;
    lin_ay = ay + a32;
    lin_az = az - a33;
    if (SerialDebug) {
      Serial.print("Lin_ax, Lin_ay, Lin_az: ");
      Serial.print(lin_ax * 1000.0f, 2);
      Serial.print(", ");
      Serial.print(lin_ay * 1000.0f, 2);
      Serial.print(", ");
      Serial.print(lin_az * 1000.0f, 2);  Serial.println(" mg");
      /*
            Serial.print("Yaw, Pitch, Roll: ");
            Serial.print(yaw, 2);
            Serial.print(", ");
            Serial.print(pitch, 2);
            Serial.print(", ");
            Serial.println(roll, 2);
      */
    }

    count = millis();
    sumCount = 0;
    sum = 0;
  }

}


/*
  tempCount = readTempData();  // Read the gyro adc values
  temperature = ((float) tempCount) / 333.87f + 21.0f; // Gyro chip temperature in degrees Centigrade
  // Print temperature in degrees Centigrade
  // Serial.print("Gyro temperature is ");  Serial.print(temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
*/

/*
      Serial.print("ax = "); Serial.print((int)1000 * ax);
      Serial.print(" ay = "); Serial.print((int)1000 * ay);
      Serial.print(" az = "); Serial.print((int)1000 * az); Serial.println(" mg");
      Serial.print("gx = "); Serial.print( gx, 2);
      Serial.print(" gy = "); Serial.print( gy, 2);
      Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
      Serial.print("mx = "); Serial.print( (int)mx );
      Serial.print(" my = "); Serial.print( (int)my );
      Serial.print(" mz = "); Serial.print( (int)mz ); Serial.println(" mG");
*/

/*

         Serial.print("q0 = "); Serial.print(q[0]);
      Serial.print(" qx = "); Serial.print(q[1]);
      Serial.print(" qy = "); Serial.print(q[2]);
      Serial.print(" qz = "); Serial.println(q[3]);

      Serial.print("Grav_x, Grav_y, Grav_z: ");
      Serial.print(-a31 * 1000.0f, 2);
      Serial.print(", ");
      Serial.print(-a32 * 1000.0f, 2);
      Serial.print(", ");
      Serial.print(a33 * 1000.0f, 2);  Serial.println(" mg");
      Serial.print("Lin_ax, Lin_ay, Lin_az: ");
      Serial.print(lin_ax * 1000.0f, 2);
      Serial.print(", ");
      Serial.print(lin_ay * 1000.0f, 2);
      Serial.print(", ");
      Serial.print(lin_az * 1000.0f, 2);  Serial.println(" mg");

      Serial.print("sumCount = "); Serial.println(sumCount);
      Serial.print("sum = "); Serial.println(sum);

      Serial.print("rate = "); Serial.print((float)sumCount / sum, 2); Serial.println(" Hz");
*/
