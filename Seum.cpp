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
/*
  Wire.beginTransmission(MPU6050);
  Wire.write(0x19);
  Wire.write(7);
  Wire.endTransmission(true);
*/
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
    _accXangle = (atan2(IMUDataArray[0], IMUDataArray[1]) + PI) * RAD_TO_DEG;
    _gyroXangle = (double)IMUDataArray[2] / 131.0;

    _currentAngle = (  0.98 * (_currentAngle + (_gyroXangle * (double)GetMicroTimeGap() / 1000000) )  + ( 0.02 * _accXangle) );

    return _currentAngle;
}

double Seum::KalmanFilter(int *IMUDataArray)
{

}

int Seum::GetControlValue(double receiveAngle)
{
  _currentError = TARGET - receiveAngle;
  _pControl = _currentError * _Kp;

  _integralError += _currentError * _timeGap;
  _iControl = _integralError * _Ki;

  _dControl = ( _currentError - _prevError ) * _Kd;

  _prevError = _currentError;

  _control = _pControl + _iControl + _dControl;
  _control = constrain(_control, -MAX_PWM, MAX_PWM);

  return _control;
}

int Seum::GetControlValue(double receiveAngle, double *ControlDataArr)
{
  _currentError = TARGET - receiveAngle;
  _pControl = _currentError * _Kp;
  ControlDataArr[0] = _pControl;

  _integralError += _currentError * _timeGap / 1000000;
  _integralError = constrain(_integralError, -15, 15);
  _iControl = _integralError * _Ki;

  ControlDataArr[1] = _iControl;

  _dControl = ( _currentError - _prevError ) * _Kd;
  ControlDataArr[2] = _dControl;

  _prevError = _currentError;

  _control = _pControl + _iControl + _dControl;
  //_control = constrain(_control, -MAX_PWM, MAX_PWM);

  return constrain(_control * _K, -MAX_PWM, MAX_PWM);
}

void Seum::MotorControl(int receiveControlValue)
{
  digitalWrite(_LeftMotorBlack, receiveControlValue > 0 ? HIGH : LOW);
  digitalWrite(_LeftMotorRed, receiveControlValue > 0 ? LOW : HIGH);
  digitalWrite(_RightMotorBlack, receiveControlValue > 0 ? HIGH : LOW);
  digitalWrite(_RightMotorRed, receiveControlValue > 0 ? LOW : HIGH);

  if (receiveControlValue < 0)
    receiveControlValue = map(receiveControlValue, 0, -255, 0, 255);

  analogWrite(_PWMA, receiveControlValue);
  analogWrite(_PWMB, receiveControlValue);
}

void Seum::GetPIDGainData(double *PIDGainArray)
{
  PIDGainArray[0] = ( analogRead(0) / 1023.0 ) * 10.0;
  PIDGainArray[1] = ( analogRead(1) / 1023.0 ) * 10.0;
  PIDGainArray[2] = ( analogRead(2) / 1023.0 ) * 10.0;
  PIDGainArray[3] = ( analogRead(3) / 1023.0 ) * 10.0;
}

void Seum::SetPIDGain(double *PIDGainArray)
{
  _Kp = PIDGainArray[0];
  _Ki = PIDGainArray[1];
  _Kd = PIDGainArray[2];
  _K  = PIDGainArray[3];
}

unsigned long Seum::GetMicroTimeGap()
{
  _timeGap = micros() - _currentTime;
  _currentTime = micros();

  return _timeGap;
}

void Seum::PrintAll()
{
  Serial.print("TIME : ");
  Serial.print(_timeGap / 1000);
  Serial.print("    ");

  Serial.print("Kp : ");
  Serial.print(_Kp);
  Serial.print("    ");

  Serial.print("Ki : ");
  Serial.print(_Ki);
  Serial.print("    ");

  Serial.print("Kd : ");
  Serial.print(_Kd);
  Serial.print("    ");

  Serial.print("K : ");
  Serial.println(_K);
}
