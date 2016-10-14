#include "Seum.h"

Kalman kalmanX;

Seum::Seum()
{
  _PWMA = 11;
  _PWMB = 10;

  _LeftMotorBlack = 13;
  _LeftMotorRed = 12;
  _RightMotorBlack = 8;
  _RightMotorRed = 7;
}

Seum::Seum(int PWMA, int PWMB, int LMB, int LMR, int RMB, int RMR)
{
  _PWMA = PWMA;
  _PWMB = PWMB;

  _LeftMotorBlack = LMB;
  _LeftMotorRed = LMR;
  _RightMotorBlack = RMB;
  _RightMotorRed = RMR;
}


void Seum::IMUSetup()
{
  Wire.begin();
  Serial.begin(115200);

  Wire.beginTransmission(MPU6050);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void Seum::GetIMUData(int *IMUDataArray)
{
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3D);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 4, true);

  IMUDataArray[0] = Wire.read() << 8 | Wire.read();
  IMUDataArray[1] = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);

  IMUDataArray[2] = Wire.read() << 8 | Wire.read();
}

double Seum::ComplementaryFilter(int *IMUDataArray)
{
    _accXangle = atan2(IMUDataArray[1], IMUDataArray[0]) * RAD_TO_DEG;
    _gyroXangle = (double)IMUDataArray[2] / 131.0;

    _currentAngle = (  0.93 * (_currentAngle + (_gyroXangle * (double)GetMicroTimeGap() / 1000000) )  + ( 0.07 * _accXangle) );

    return _currentAngle;
}

double Seum::KalmanFilter(int *IMUDataArray)
{

}

int Seum::GetControlValue(int receiveAngle)
{
  _currentError = TARGET - receiveAngle;
  _pControl = _currentError * Kp;

  _integralError += _currentError * (double)GetMicroTimeGap() / 1000000;
  _iControl = _integralError * Ki;

  _dControl = ( _currentError - _prevError ) * Kd;

  _prevError = _currentError;

  _control = _pControl + _iControl + _dControl;
  _control = constrain(_control, -MAX_PWM, MAX_PWM);

  return _control;
}

void Seum::MotorControl(int receiveControlValue)
{
  digitalWrite(_LeftMotorBlack, receiveControlValue > 0 ? HIGH : LOW);
  digitalWrite(_LeftMotorRed, receiveControlValue > 0 ? LOW : HIGH);
  digitalWrite(_RightMotorBlack, receiveControlValue > 0 ? HIGH : LOW);
  digitalWrite(_RightMotorRed, receiveControlValue > 0 ? LOW : HIGH);

  if (receiveControlValue < 0)
    _control = map(receiveControlValue, 0, -255, 0, 255);

  analogWrite(_PWMA, receiveControlValue);
  analogWrite(_PWMB, receiveControlValue);
}

unsigned long Seum::GetMicroTimeGap()
{
  _timeGap = micros() - _currentTime;
  _currentTime = micros();

  return _timeGap;
}
