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

#ifndef MPU6050
#define MPU6050 0x68
#endif

#define TARGET 180
#define MAX_PWM 255

#include "Arduino.h"

class Seum
{
  public:

    Seum();
    Seum(int PWMA, int PWMB, int LMB, int LMR, int RMB, int RMR);

    void IMUSetup();
    void GetIMUData(int IMUDataArray[]);
    double ComplementaryFilter(int IMUDataArray[]);
    double KalmanFilter(int IMUDataArray[]);
    unsigned long GetMicroTimeGap();

    int GetControlValue(int receiveAngle);
    void MotorControl(int receiveControlValue);



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

    int _pControl;
    int _iControl;
    int _dControl;

    int _control;

    #ifndef Kp
    double Kp;
    #endif

    #ifndef Ki
    double Ki;
    #endif

    #ifndef Kd
    double Kd;
    #endif
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
