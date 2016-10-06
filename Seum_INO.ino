#include <Kalman.h>

#include <math.h>
#include <Wire.h>

#define DebugMode 1

#define MPU6050 0x68
#define DT 0.001

#define PWMA 11
#define PWMB 10

#define LeftMotorBlack  13  // A-IN1 -> A-OUT1 -> LeftMotorBlack
#define LeftMotorRed    12  // A-IN2 -> A-OUT2 -> LeftMotorRed
#define RightMotorBlack  8  // B-IN1 -> B-OUT1 -> RightMotorBlack
#define RightMotorRed    7  // B-IN2 -> B-OUT2 -> RightMotorRed

#define TARGET 180.0
#define RANGE 0.5

Kalman kalmanX;

/* IMU DATA */
float accXangle;//, accYangle; // Angle calculate using the accelerometer
float gyroXangle;//, gyroYangle; // Angle calculate using the gyro
float kalAngleX;//, kalAngleY; // Calculate the angle using a Kalman filter

double Kp, Ki, Kd;

double pControl, iControl, dControl;
double IntegralError, prevError;

int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

float accCurrentAngle;//, accYangle; // Angle calculate using the accelerometer
float gyroCurrentAngle;//, gyroYangle; // Angle calculate using the gyro

unsigned long timer;
uint8_t i2cData[14]; // Buffer for I2C data
float CurrentAngle;
/* IMU DATA */

void setup() {
    
  Wire.begin();
  Serial.begin(115200);

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
    
  pinMode(LeftMotorBlack, OUTPUT);
  pinMode(LeftMotorRed, OUTPUT);
  pinMode(RightMotorBlack, OUTPUT);
  pinMode(RightMotorRed, OUTPUT);

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  
  while(i2cWrite(0x19,i2cData,4,false)); // Write to all four registers at once
  while(i2cWrite(0x6B,0x01,true)); // PLL with X axis gyroscope reference and disable sleep mode 

  while(i2cRead(0x75,i2cData,1));
  
  if(i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while(1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while(i2cRead(0x3B,i2cData,6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;

  kalmanX.setAngle(accXangle); // Set starting angle
  gyroXangle = accXangle;
  timer = micros();
}

void GetIMUData()
{
  while(i2cRead(0x3B,i2cData,14));
  
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  
  tempRaw = ((i2cData[6] << 8) | i2cData[7]);  
  
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);
  
  accCurrentAngle = (atan2(accY,accZ) + PI) * RAD_TO_DEG;
  double gyroXrate = (double)gyroX/131.0;
  
  CurrentAngle = kalmanX.getAngle(accCurrentAngle, gyroXrate, (double)(micros()-timer)/1000000);
  timer = micros();
}

void GetAnalogData()
{
  Kp = ( analogRead(0) / 1023.0 ) * 10.0;
  Ki = ( analogRead(1) / 1023.0 ) * 10.0;
  Kd = ( analogRead(2) / 1023.0 ) * 10.0;
}

int PID()
{
  double error;

  int control;
  
  error = TARGET - CurrentAngle;
  pControl = error * Kp;

  IntegralError += error * 0.01;
  IntegralError = constrain(IntegralError, -15, 15);
  iControl =  IntegralError * Ki;

  dControl = Kd * (error - prevError);
  
  prevError = error;
  control = 2.128 * (pControl + iControl + dControl);

  control = constrain(control, -255, 255);

  return control;
}

void MotorStop(int LMB, int LMR, int RMB, int RMR)
{
  analogWrite(LeftMotorBlack, 0);
  analogWrite(LeftMotorRed, 0);

  analogWrite(RightMotorBlack, 0);
  analogWrite(RightMotorRed, 0);
}

void Motor(int control, int LMB, int LMR, int RMB, int RMR)
{ 
  
    digitalWrite(LMR, CurrentAngle > TARGET ? HIGH : LOW);
    digitalWrite(LMB, CurrentAngle > TARGET ? LOW : HIGH);
    digitalWrite(RMR, CurrentAngle > TARGET ? HIGH : LOW);
    digitalWrite(RMB, CurrentAngle > TARGET ? LOW : HIGH);

    if ( control < 0 )
      control = map(control, 0, -255, 0, 255);

    analogWrite(PWMA, control);
    analogWrite(PWMB, control);

}

void printAll(int control)
{
  Serial.print("CurrentAngle : ");  Serial.println(CurrentAngle);
  Serial.print("Kp : ");  Serial.print(Kp);
  Serial.print("  Ki : ");  Serial.print(Ki);
  Serial.print("  Kd : ");  Serial.print(Kd);
  Serial.print("  control : ");  Serial.print(control); 
}

void loop() {

  int control;
  
  GetIMUData();
  control = PID();
  
  GetAnalogData();
       
  Motor(control, LeftMotorBlack, LeftMotorRed, RightMotorBlack, RightMotorRed);
  

  #if DebugMode
  printAll(control);
  #endif

}
