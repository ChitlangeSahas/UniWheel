#include <Wire.h>
#include <Kalman.h>
#include <PID_v1.h>
#include <VescUart.h>
#define RESTRICT_PITCH          // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define SENS_ID 0x73            // Sensor Id for mpu 9250
#define VESC_SERIAL_PORT Serial3
#define MAX_DUTY_CYCLE 100      // Max Duty Cycle for the Motor. (% of Max Speed the motor is allowed to go.)
/* Kalman filter constants */
float Q_angle = 0.001; 
float Q_bias = 0.003;
float R_measure = 0.03;

/* Create the Kalman instances */
Kalman kalmanX; 
Kalman kalmanY;

/* VESC */ 
VescUart UART;
/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double gyroXangle, gyroYangle;              // Angle calculate using the gyro only
double compAngleX, compAngleY;              // Calculated angle using a complementary filter
double kalAngleX, kalAngleY;                // Calculated angle using a Kalman filter

/* Misc data variables */
uint32_t timer;
uint8_t i2cData[14];                        // Buffer for I2C data

//Define Variables we'll be connecting to
double Setpoint, Output;

//Specify the links and initial tuning parameters
//double Kp=3.5, Ki=0.005, Kd=2.05;
double Kp=1.0, Ki=0.0, Kd=0.0;
PID myPID(&kalAngleX, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() 
{
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL);                  // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2;     // Set I2C frequency to 400kHz
#endif
  setupKalmanFilter();
  setupPID();
  Setpoint = 0.1;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-MAX_DUTY_CYCLE, MAX_DUTY_CYCLE);
}
void loop()
{
  updateKalmanAnglesXY();
  myPID.Compute();
  UART.setDuty(Output/100.0);
  Serial.print(kalAngleX); Serial.print("\t"); // X 
  Serial.print(kalAngleY); Serial.print("\t"); // Not really needed
  Serial.print(Output/100.0);Serial.print("\t"); // PID controller output
  Serial.print("\n");
  delay(1);
}
/*
 * function: setupKalmanFilter
 * performs the setup for the kalman filter.
 */
void setupKalmanFilter()
{
  i2cData[0] = 7;                             // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;                          // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00;                          // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00;                          // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false));  // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true));         // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  
  // Make sure that the sensor is compatible.
  if (i2cData[0] != SENS_ID) 
  { 
    // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor. Is your sensor hardware MPU9250?"));
    while (1);
  }
  // Wait a second for sensor to stabilize.
  delay(1000); 

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  
  // Set starting angle
  kalmanX.setAngle(roll); 
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  timer = micros();

  // Set Kalman Filter constants.
  kalmanX.setQangle(Q_angle);kalmanY.setQangle(Q_angle);
  kalmanX.setQbias(Q_bias);kalmanY.setQbias(Q_bias);
  kalmanX.setRmeasure(R_measure);kalmanY.setRmeasure(R_measure);
}
void setupPID()
{
  VESC_SERIAL_PORT.begin(115200);
  while (!Serial) {;}
  UART.setSerialPort(&Serial3);
}
void updateKalmanAnglesXY() 
{
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) 
  {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
    gyroXangle = roll;
  } 
  else
  {
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);        // Calculate the angle using a Kalman filter
  }
  if (abs(kalAngleX) > 90)
  {
    gyroYrate = -gyroYrate;                                   // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) 
  {
    kalmanY.setAngle(pitch);
    kalAngleY = pitch;
    gyroYangle = pitch;
  } 
  else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);       // Calculate the Y angle using a Kalman filter

  if (abs(kalAngleY) > 90)
  {
    gyroXrate = -gyroXrate;                                   // Invert rate, so it fits the restriced accelerometer reading
  }
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);          // Calculate the X angle using a Kalman filter
#endif

//  gyroXangle += gyroXrate * dt;                             // Calculate gyro angle without any filter
//  gyroYangle += gyroYrate * dt;
  gyroXangle += kalmanX.getRate() * dt;                       // Calculate gyro angle using the unbiased rate
  gyroYangle += kalmanY.getRate() * dt;

  /* Reset the gyro angle when it has drifted too much */
  if (gyroXangle < -180 || gyroXangle > 180)                  
  {
    gyroXangle = kalAngleX;
  }
  if (gyroYangle < -180 || gyroYangle > 180)
  {
    gyroYangle = kalAngleY;
  }
}
