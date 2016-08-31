#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// MOTOR BROWN ENCODER VCC
//       GREEN ENCODER GND


/* MOTOR PINS */
#define LMB 23  // LEFT MOTOR BLACK
#define LMR 24  // LEFT MOTOR RED
#define LEB NO USE // LEFT ENCODER BLUE
#define LEP NO USE // LEFT ENCODER PURPLE

#define RMB 19 // RIGHT MOTOR BLACK
#define RMR 26 // RIGHT MOTOR RED
#define REB NO USE // RIGHT ENCODER BLUE
#define REP NO USE // RIGHT ENCODER PURPLE
/* MOTOR PINS */


/* MOTOR DRIVE */
#define STBY 16 
#define PWMA 20
#define PWMB 21
/* MOTOR DRIVE */


/* IMU */
#define MPU9250 0x68
#define PI 3.141592
#define DT 0.001
/* IMU */


/* PWM */
#define PWM_MAX  80
#define PWM_MAX_SPEED 80
#define PWM_MIN_SPEED 1
/* PWM */


/* CONTROL */
#define Kp  4.0

#define RANGE 0.5
#define TARGET 90.0
/* CONTROL */

double err;
double Kp_term;
double control;

double XANGLE;

double complement(int data) 
{
    if( data >= 32786) {

        data = ( (data ^ 0xFFFF) ) * -1;
        return data;
    }

    else
        return data;

}

PI_THREAD(IMU)
{
    int sensor;
    int GYRO_X, ACCEL_Y, ACCEL_Z; //RAW VALUE
    double G_RE_X, A_RE_Y, A_RE_Z; //TWO COMPLEMENT
    double ACCX, GYROX;	       //CHANGE VALUE                   

    sensor = wiringPiI2CSetup(MPU9250);

    while(1) {

        GYRO_X = wiringPiI2CReadReg8(sensor, 0x43);
        GYRO_X = GYRO_X << 8 | wiringPiI2CReadReg8(sensor, 0x44);
        G_RE_X = complement(GYRO_X);

        ACCEL_Y = wiringPiI2CReadReg8(sensor, 0x3D);
        ACCEL_Y = ACCEL_Y << 8 | wiringPiI2CReadReg8(sensor, 0x3E); // ACC Y
        A_RE_Y = complement(ACCEL_Y);

        ACCEL_Z = wiringPiI2CReadReg8(sensor, 0x3F);
        ACCEL_Z = ACCEL_Z << 8 | wiringPiI2CReadReg8(sensor, 0x40); // ACC Z
        A_RE_Z = complement(ACCEL_Z);

        ACCX = atan2(A_RE_Z, A_RE_Y) * 180 / PI;

        GYROX = (double)G_RE_X / 131.0;

        XANGLE = (0.93 * (XANGLE + (GYROX * DT)) + (0.07 * ACCX));

        printf("XANGLE : %10lf\n", XANGLE);	


    }
}

void Setup() 
{
    wiringPiSetupGpio();

    pinMode(LMB, OUTPUT);
    pinMode(LMR, OUTPUT);

    pinMode(RMB, OUTPUT);
    pinMode(RMR, OUTPUT);

    pinMode(STBY, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);

    digitalWrite(STBY, 1);	
    digitalWrite(PWMA, 1);
    digitalWrite(PWMB, 1);

    piThreadCreate(IMU);

    softPwmCreate(LMB, 0, PWM_MAX);
    softPwmCreate(LMR, 0, PWM_MAX);

    softPwmCreate(RMB, 0, PWM_MAX);
    softPwmCreate(RMR, 0, PWM_MAX);
}

void MotorGo() 
{

    printf("GO\n");

    if ( control > PWM_MAX_SPEED ) { // MAX_SPEED 넘으면 제한

        softPwmWrite(LMR, PWM_MAX_SPEED);
        softPwmWrite(RMR, PWM_MAX_SPEED);
    }

    else if ( control > PWM_MIN_SPEED ) { // MIN_SPEED 아래면 MIN_SPEED 보다 빨리

        softPwmWrite(LMR, (int)control);
        softPwmWrite(RMR, (int)control);
    }

    digitalWrite(LMB, 0);
    digitalWrite(RMB, 0);
}

void MotorBack()
{

    printf("BACK\n");

    if ( control * -1 > PWM_MAX_SPEED) {

        softPwmWrite(LMB, PWM_MAX_SPEED);
        softPwmWrite(RMB, PWM_MAX_SPEED);
    }
    else if ( control * -1 > PWM_MIN_SPEED) {

        softPwmWrite(LMB, (int)control * -1);
        softPwmWrite(RMB, (int)control * -1);
    }

    digitalWrite(LMR, 0);
    digitalWrite(RMR, 0);

}

void MotorStop()
{
    digitalWrite(LMB, 0);
    digitalWrite(LMR, 0);

    digitalWrite(RMB, 0);
    digitalWrite(RMR, 0);
}

void main() 
{

    Setup();

    while(1) {


        err = (TARGET - 1.0) - XANGLE;
        Kp_term = Kp * err;

        control = Kp_term;  // PID 중 P만

        if ( XANGLE > TARGET - RANGE && XANGLE < TARGET + RANGE ) // 89.5 ~ 90.5 사이면 모터 정지 
            MotorStop();

        else if ( TARGET + RANGE > XANGLE ) // 현재 기울기가 90.5 보다 적으면 모터 전진
            MotorGo();


        else if ( TARGET - RANGE < XANGLE ) // 현재 기울기가 89.5 보다 많으면 모터 후진
            MotorBack();

    }	
}
