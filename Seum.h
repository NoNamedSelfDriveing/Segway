#ifndef Seum_h
#define Seum_h

#ifndef Wire_h
#define Wire_h
#include <Wire.h>
#endif

#ifndef MPU6050
#define MPU6050 0x68
#endif

#define TARGET 180
#define MAX_PWM 255

#define runEvery(t) for (static typeof(t) _lasttime; (typeof(t)) ((typeof(t))millis() - _lasttime) > (t); _lasttime += (t))

#include "Arduino.h"

class Seum
{
  public:

    /* 생성자 */
    Seum();   // 기본적으로 세팅
    Seum(int PWMA, int PWMB, int LMB, int LMR, int RMB, int RMR);   // 자신이 원하는 대로 핀 세팅
    /* 생성자 */

    /* 기울기 값 구하기 */
    void IMUSetup();    // IMU 켜줌
    void GetIMURawData(int IMUDataArray[]);   // 기울기 센서의 필터가 적용 되지 않은 값 구하기 가속도 센서의 Y, Z 값과 자이로 센서의 X 값만
    double ComplementaryFilter(int IMUDataArray[]);   // 가공되지 않은 값들을 상보필터를 통해서 기울기로 얻는다.
    double GetGradient();   // GetIMURawData함수와 ComplementaryFilter함수를 한곳에 모아논 함수
    /* 기울기 값 구하기 */

    /* 가변저항 PID 게인 값 구하기 & 세팅 */
    void GetPIDGainData(double PIDGainArray[]);   // 가변저항 0~3 부터 읽기
    void SetPIDGain(double PIDGainArray[]);   // 해당 버퍼 순서대로 / 0 : Kp  / 1 : Ki  / 2: Kd  / 3: K /으로세팅
    /* 가변저항 PID 게인 값 구하기 & 세팅 */

    /* PID 계산과, 모터 움직임 */
    int GetControlValue(double receiveAngle);   // PID 계산을 통해 control 값 구하기
    int GetControlValue(double receiveAngle, double ControlDataArr[]);    // 위와 하는 역할은 같지만, ControlDataArr버퍼에 PID 각각의 Control 값 담기
    void MotorControl(int receiveControlValue);   // PID 계산을 통해 얻은 Control 값으로 모터 제어
    void PIDMotor(double receiveAngle);   //   GetControlValue 함수와 MotorControl 함수를 한곳에 모아논 함수
    /* PID 계산과, 모터 움직임 */

    /* 잡기능 */
    unsigned long GetMicroTimeGap();  //한 루프가 도는 속도 Micro단위로 구함 dt(delta time)구하는데 사용
    void PrintAll(); // 현재 여러가지 상태들 출력
    /* 잡기능 */

    /* 단계별 함수들 */
    void LevelOne();
    void LevelTwo();
    /* 단계별 함수들 */


  private:
    /* 핀들 */
    int _PWMA;
    int _PWMB;

    int _LeftMotorBlack;
    int _LeftMotorRed;
    int _RightMotorBlack;
    int _RightMotorRed;
    /* 핀들 */

    /* PID */
    double _currentError;   // 현재 에러
    double _prevError;    // 이전의 에러
    double _integralError;    // 적분된(누적된) 에러

    double _pControl;   // P 제어 값
    double _iControl;   // I 제어 값
    double _dControl;   // D 제어 값

    int _control;   // PID 제어 값

    double _Kp;   // P 게인 값
    double _Ki;   // I 게인 값
    double _Kd;   // D 게인 값
    double _K;    // K 게인 값
    /* PID */

    /* 기울기 센서 */
    int _accX, _accY, _accZ;
    int _temperature;
    int _gyroX, _gyroY, _gyroZ;

    double _accXangle;    // atan2를 통해 나온 가속도 기울기 값
    double _gyroXangle;   // 자이로센서로 구한 기울기 값

    double _currentAngle;  // 상보필터를 통해 구한 기울기 값
    /* 기울기 센서 */

    /* BUFFERS */
    int _IMUDataArray[3];
    /* BUFFERS */

    /* TIMER */
    unsigned long _currentTime = 0;
    unsigned long _timeGap;
    unsigned long _millisCurrentTime = 0;
    unsigned long _millisTimeGap;
    /* TIMER */
};

#endif
