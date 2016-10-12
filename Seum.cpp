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

double Seum::GetIMUData()
{
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3D);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 4, true);

  _accY = Wire.read() << 8 | Wire.read();
  _accZ = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(MPU6050);
  Wire.write(0x43);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050, 2, true);

  _gyroX = Wire.read() << 8 | Wire.read();

  _accXangle = atan2(_accZ, _accY) * 180 / PI;
  _gyroXangle = (double)_gyroX / 131.0;

  _currentAngle = (0.93 * (_currentAngle + (_gyroXangle * 0.001) ) + 0.07 * _accXangle);

  return _currentAngle;
}

int Seum::GetControlValue()
{
  _currentError = TARGET - _currentAngle;
  _pControl = _currentError * Kp;

  _integralError += _currentError;
  _iControl = _integralError * Ki;

  _dControl = ( _currentError - _prevError ) * Kd;

  _prevError = _currentError;

  _control = _pControl + _iControl + _dControl;
  _control = constrain(_control, -MAX_PWM, MAX_PWM);

  return _control;
}
