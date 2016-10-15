#ifndef Seum_h
#define Seum_h

#ifndef Kalman_h
#define Kalman_h
#include <Kalman.h>
#endif

#ifndef Wire_h
#define Wire_h
#include <Wire.h>
#endif

#ifndef I2Cdev_h
#define I2Cdev_h
#include <I2Cdev.h>
#endif

#ifndef MPU6050
#define MPU6050 0x68
#endif

#define TARGET 180
#define MAX_PWM 255

#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t))

#include "Arduino.h"

class Seum
{
  public:

    Seum();
    Seum(int PWMA, int PWMB, int LMB, int LMR, int RMB, int RMR);

    void IMUSetup();
    void GetIMUData(int IMUDataArray[]);
    void GetPIDGainData(double PIDGainArray[]);
    void SetPIDGain(double PIDGainArray[]);
    double ComplementaryFilter(int IMUDataArray[]);
    double KalmanFilter(int IMUDataArray[]);
    unsigned long GetMicroTimeGap();

    int GetControlValue(double receiveAngle);
    int GetControlValue(double receiveAngle, double ControlDataArr[]);
    void MotorControl(int receiveControlValue);
    void PrintAll();



  private:
    /* PINS */
    int _PWMA;
    int _PWMB;

    int _LeftMotorBlack;
    int _LeftMotorRed;
    int _RightMotorBlack;
    int _RightMotorRed;
    /* PINS */

    /* PID */
    double _currentError;
    double _prevError;
    double _integralError;

    double _pControl;
    double _iControl;
    double _dControl;

    int _control;

    double _Kp;
    double _Ki;
    double _Kd;
    double _K;
    /* PID */

    /* IMU */
    int _accX, _accY, _accZ;
    int _temperature;
    int _gyroX, _gyroY, _gyroZ;

    double _accXangle;
    double _gyroXangle;

    double _currentAngle;
    /* IMU */

    unsigned long _currentTime = 0;
    unsigned long _timeGap;

};

#endif
