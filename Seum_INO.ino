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

#define TARGET 90.0
#define RANGE 0.1

double Kp, Ki, Kd;
double XANGLE;

double pControl, iControl, dControl;
double IntegralError, prevError;

double PWM_MIN;

unsigned long prev_time = 0;

void setup() {
    
  Wire.begin();
  Serial.begin(115200);

  Wire.beginTransmission(MPU6050); //MPU로 데이터 전송 시작
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     //MPU-6050 시작 모드로
  Wire.endTransmission(true); 

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
    
  pinMode(LeftMotorBlack, OUTPUT);
  pinMode(LeftMotorRed, OUTPUT);
  pinMode(RightMotorBlack, OUTPUT);
  pinMode(RightMotorRed, OUTPUT);
}

void GetIMUData()
{
  int AcY,AcZ,GyX;
  double ACCX, GYROX;

  Wire.beginTransmission(MPU6050);    //데이터 전송시작
  Wire.write(0x3D);               // register 0x3B (ACCEL_XOUT_H), 큐에 데이터 기록
  Wire.endTransmission(false);    //연결유지  
  Wire.requestFrom(MPU6050, 4, true);  //MPU에 데이터 요청
  
  AcY = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  Wire.beginTransmission(MPU6050);    //데이터 전송시작
  Wire.write(0x43);               // register 0x3B (ACCEL_XOUT_H), 큐에 데이터 기록
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU6050, 2, true);
  
  GyX = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 
  ACCX = atan2(AcZ, AcY) * 180 / PI;
  GYROX = (double)GyX / 131.0;
  XANGLE = (0.93 * (XANGLE + (GYROX * DT)) + (0.07 * ACCX));
  
  //Serial.println(XANGLE);  
}

void GetAnalogData()
{
  Kp = ( analogRead(0) / 1023.0 ) * 10.0;
  Ki = ( analogRead(1) / 1023.0 ) * 10.0;
  Kd = ( analogRead(2) / 1023.0 ) * 10.0;

  PWM_MIN = ( analogRead(3) / 1023.0 ) * 100.0;
  
}

int PID()
{
  double error;

  int control;
  
  error = XANGLE - TARGET;
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
  
    digitalWrite(LMB, XANGLE > TARGET ? HIGH : LOW);
    digitalWrite(LMR, XANGLE > TARGET ? LOW : HIGH);
    digitalWrite(RMB, XANGLE > TARGET ? HIGH : LOW);
    digitalWrite(RMR, XANGLE > TARGET ? LOW : HIGH);

    if ( control < 0 )
      control = map(control, 0, -255, 0, 255);

    if ( control < PWM_MIN )
      control = PWM_MIN;

    analogWrite(PWMA, control);
    analogWrite(PWMB, control);

}

void printAll(int control)
{
  unsigned long current_time;
  
  Serial.print("XANGLE : ");  Serial.print(XANGLE);
  Serial.print("Kp : ");  Serial.print(Kp);
  Serial.print("  Ki : ");  Serial.print(Ki);
  Serial.print("  Kd : ");  Serial.print(Kd);
  Serial.print("  PWM_MIN : ");  Serial.print(PWM_MIN);
  Serial.print("  control : ");  Serial.print(control); 

  current_time = millis();
  Serial.print("  time : "); Serial.println(current_time - prev_time);
  prev_time = current_time;
}

void loop() {

  int control;
  
  GetIMUData();
  control = PID();
  
  
  if (  ( XANGLE < TARGET + RANGE && XANGLE > TARGET - RANGE ) || ( XANGLE > 130.0 || XANGLE < 50.0 ) )  
  {
    MotorStop(LeftMotorBlack, LeftMotorRed, RightMotorBlack, RightMotorRed);
    //IntegralError = 0; 
  }

  else
  { 
    GetAnalogData();
       
    Motor(control, LeftMotorBlack, LeftMotorRed, RightMotorBlack, RightMotorRed);
  }

  #if DebugMode
  printAll(control);
  #endif

}
