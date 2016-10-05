// http://www.bajdi.com
// Self balancing bot:
// µController = ATmega328
// 2 Pololu micro metal gear motors with 60mm wheels + DRV8833 motor controller
// 6DOF MPU6050 sensor 

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

Kalman kalmanX; // Create the Kalman instance

/* IMU Data */
int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

float accXangle;//, accYangle; // Angle calculate using the accelerometer
float gyroXangle;//, gyroYangle; // Angle calculate using the gyro
float kalAngleX;//, kalAngleY; // Calculate the angle using a Kalman filter

unsigned long timer;
uint8_t i2cData[14]; // Buffer for I2C data
float CurrentAngle;

// Motor controller pins

#define PWMA 11
#define PWMB 10

const int AIN1 = 13;  // (pwm) pin 3 connected to pin AIN1 
const int AIN2 = 12;  // (pwm) pin 9 connected to pin AIN2 
const int BIN1 = 8; // (pwm) pin 10 connected to pin BIN1  
const int BIN2 = 7;  // (pwm) pin 11 connected to pin BIN2 

int speed;

// PID
float Kp = 4; 
float Ki = 1;
float Kd = 1;
float pTerm, iTerm, dTerm, integrated_error, last_error, error;
//const float K = 1.9*1.12;
const float K = 2.5;
#define   GUARD_GAIN   10.0
  
#define runEvery(t) for (static typeof(t) _lasttime; (typeof(t)) ((typeof(t))millis() - _lasttime) > (t); _lasttime += (t))

double runtime;

unsigned int current_time;
unsigned int prev_time;

void setup() {  
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(AIN1, OUTPUT); // set pins to output
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  Serial.begin(57600);
  Wire.begin();
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

void loop() {
  runEvery((int)runtime)  // run code @ 40 Hz
  {
    dof();
    //if (CurrentAngle <= 180.2 && CurrentAngle >= 179.8)
    //{
    //  stop();
    //}
    //else
    //{
      if (CurrentAngle < 230 && CurrentAngle > 130)
      {
        getAnalogData();
        Pid();
        Motors();
      }
      else
        stop();
         
    //}

    //printAll();
  }
}

void Motors(){
  if (speed > 0)
  { 
    //forward 
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
  }
  else
  { 
    // backward
    speed = map(speed,0,-255,0,255);
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
  }

  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void stop()
{
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void Pid(){
  error = 180 - CurrentAngle;  // 180 = level
  pTerm = Kp * error;
  integrated_error += error;
  iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
  dTerm = Kd * (error - last_error);
  last_error = error;
  speed = constrain(K*(pTerm + iTerm + dTerm), -255, 255);
}

void dof()
{
  while(i2cRead(0x3B,i2cData,14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = ((i2cData[6] << 8) | i2cData[7]);  
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);
  accXangle = (atan2(accY,accZ)+PI)*RAD_TO_DEG;
  double gyroXrate = (double)gyroX/131.0;
  CurrentAngle = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000);
  timer = micros();
}

void getAnalogData()
{
  Kp = ( analogRead(0) / 1023.0 ) * 10.0;
  Ki = ( analogRead(1) / 1023.0 ) * 10.0;
  Kd = ( analogRead(2) / 1023.0 ) * 10.0; 
  runtime = ( analogRead(3) / 1023.0 ) * 100.0;
}

void printAll()
{
  Serial.print("XANGLE : "); Serial.print(CurrentAngle);
  Serial.print("runtime : "); Serial.print(runtime);
  Serial.print("Kp : "); Serial.print(Kp);
  Serial.print("Ki : "); Serial.print(Ki);
  Serial.print("Kd : "); Serial.print(Kd);
  Serial.print("speed : "); Serial.print(speed);

  current_time = millis();
  Serial.print("time : "); Serial.println(current_time - prev_time);
  prev_time = current_time;
}

