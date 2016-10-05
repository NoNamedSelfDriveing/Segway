#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


// MOTOR BROWN ENCODER VCC
//       GREEN ENCODER GND

// GO :   -
// BACK : +

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
#define PWM_MAX  255
/* PWM */


/* CONTROL */
#define RANGE 0.5
double TARGET = 90.0;
/* CONTROL */


/* PID */
#define dt 0.001

double Kp, Ki, Kd;

double err, I_err;
double prev_err, D_err;
double Kp_term, Ki_term, Kd_term;
double control;
/* PID */

double XANGLE;

int sensor;
int GYRO_X, ACCEL_Y, ACCEL_Z; //RAW VALUE
double G_RE_X, A_RE_Y, A_RE_Z; //TWO COMPLEMENT
double ACCX, GYROX;

int PWM_MIN_SPEED;

double complement(int data) 
{
	if( data >= 32786) {

		data = ( (data ^ 0xFFFF) ) * -1;
		return data;
	}

	else
		return data;

}

void GetData()
{

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

	XANGLE = (0.95 * (XANGLE + (GYROX * DT)) + (0.05 * ACCX));

//	printf("XANGLE : %10lf\n", XANGLE);	

}

void Setup() 
{
	wiringPiSetupGpio();

	sensor = wiringPiI2CSetup(MPU9250);

	pinMode(STBY, OUTPUT);
	pinMode(PWMA, OUTPUT);
	pinMode(PWMB, OUTPUT);

	digitalWrite(STBY, 1);	
	digitalWrite(PWMA, 1);
	digitalWrite(PWMB, 1);

	softPwmCreate(LMB, 0, PWM_MAX);
	softPwmCreate(LMR, 0, PWM_MAX);

	softPwmCreate(RMB, 0, PWM_MAX);
	softPwmCreate(RMR, 0, PWM_MAX);
}

void MotorGo() 
{

	if( (int)control > PWM_MIN_SPEED ) 
	{
		softPwmWrite(LMR, (int)(control));
		softPwmWrite(RMR, (int)(control));
	}

	else 
	{
		softPwmWrite(LMR, PWM_MIN_SPEED);
		softPwmWrite(RMR, PWM_MIN_SPEED);
	}

	softPwmWrite(LMB, 0);
	softPwmWrite(RMB, 0);
}

void MotorBack()
{

	if( (int)control > PWM_MIN_SPEED )
	{
		softPwmWrite(LMB, (int)(control));
		softPwmWrite(RMB, (int)(control));		
	}

	else
	{
		softPwmWrite(LMB, PWM_MIN_SPEED);
		softPwmWrite(RMB, PWM_MIN_SPEED);
	}

	softPwmWrite(LMR, 0);
	softPwmWrite(RMR, 0);

}

void MotorStop()
{
	softPwmWrite(LMB, 0);
	softPwmWrite(LMR, 0);

	softPwmWrite(RMB, 0);
	softPwmWrite(RMR, 0);
}

// 1 : KP
// 2 : KI
// 3 : PWM_MIN_SPEED

void PID()
{
	err = (TARGET) - XANGLE;
	Kp_term = Kp * err;

	I_err += err; 
	Ki_term = Ki * I_err;

	D_err = (err - prev_err) / 0.1 ;
	Kd_term = Kd * D_err;

	prev_err = err;

	control = fabs(Kp_term) + fabs(Ki_term);
}

void main(int argc, char *argv[]) 
{
//	FILE *fp;

	Setup();

	Kp = atof(argv[1]);
	Ki = atof(argv[2]);
//	Kd = atof(argv[3]);
	PWM_MIN_SPEED = atoi(argv[3]);

//	fp = fopen("./log", "w");

	while(1) {

		GetData();

		if ( ( XANGLE >= (TARGET - RANGE)  && XANGLE <= (TARGET + RANGE) ) || ( XANGLE > 120.0 || XANGLE < 50.0) ) {
			
			MotorStop();

			Ki_term = 0;
			I_err = 0;
		}

		else {

			PID();

			if ( TARGET - RANGE > XANGLE ) 
				MotorGo();
			

			else if ( TARGET + RANGE < XANGLE )
				MotorBack();


//			fprintf(fp, "%lf\n", control);

			printf("Kp_term : %10lf\tKi_term : %10lf\tCONTROL : %10lf\n", Kp_term, Ki_term, control);	
		}
	}	
}
