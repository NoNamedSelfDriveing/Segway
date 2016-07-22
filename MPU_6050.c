#include <stdio.h>
#include <wiringPi.h>
#include <math.h>

#define ADDRESS 0x68
#define PI 3.141592
#define DT 0.001

double GYRO_Change(int data) // Two's complement
{
	if( data >= 32786) {

		data = ( (data ^ 0xFFFF) ) * -1;
		return data;
	}

	else
		return data;
}

double ACCEL_Change(int data) // Two's complement
{
	if( data >= 35500) {

		data = ( (data ^ 0xFFFF) ) * -1;
		return data;
	}

	else
		return data;

}

void main()
{
	int GYRO_X, GYRO_Y, GYRO_Z, ACCEL_X, ACCEL_Y, ACCEL_Z;
	double G_RE_X, G_RE_Y, G_RE_Z, A_RE_X, A_RE_Y, A_RE_Z, ACCX, ACCY, ACCZ, GYROX, GYROY, GYROZ, XANGLE, YANGLE, ZANGLE;
	int sensor;

	sensor = wiringPiI2CSetup(ADDRESS);

	wiringPiI2CWriteReg8(sensor, 0x6B, 0);
	wiringPiI2CWriteReg8(sensor, 27, 0x00);

	while(1) {

		GYRO_X = wiringPiI2CReadReg8(sensor, 0x43);
		GYRO_X = GYRO_X << 8 | wiringPiI2CReadReg8(sensor, 0x44); // GYRO X
		G_RE_X = GYRO_Change(GYRO_X);

		GYRO_Y = wiringPiI2CReadReg8(sensor, 0x45);
		GYRO_Y = GYRO_Y << 8 | wiringPiI2CReadReg8(sensor, 0x46); // GYRO Y
		G_RE_Y = GYRO_Change(GYRO_Y);

		GYRO_Z = wiringPiI2CReadReg8(sensor, 0x47);
		GYRO_Z = GYRO_Z << 8 | wiringPiI2CReadReg8(sensor, 0x48); // GYRO Z
		G_RE_Z = GYRO_Change(GYRO_Z);

		ACCEL_X = wiringPiI2CReadReg8(sensor, 0x3B);
		ACCEL_X = ACCEL_X << 8 | wiringPiI2CReadReg8(sensor, 0x3C); // ACC X
		A_RE_X = ACCEL_Change(ACCEL_X);

		ACCEL_Y = wiringPiI2CReadReg8(sensor, 0x3D);
		ACCEL_Y = ACCEL_Y << 8 | wiringPiI2CReadReg8(sensor, 0x3E); // ACC Y
		A_RE_Y = ACCEL_Change(ACCEL_Y);

		ACCEL_Z = wiringPiI2CReadReg8(sensor, 0x3F);
		ACCEL_Z = ACCEL_Z << 8 | wiringPiI2CReadReg8(sensor, 0x40); // ACC Z
		A_RE_Z = ACCEL_Change(ACCEL_Z);


		ACCY = atan2(A_RE_Z, A_RE_X) * 180 / PI;
		ACCX = atan2(A_RE_Z, A_RE_Y) * 180 / PI;
		ACCZ = atan2(A_RE_Y, A_RE_X) * 180 / PI;

		GYROX = (double)G_RE_X / 131.0;
		GYROY = (double)G_RE_Y / 131.0;
		GYROZ = (double)G_RE_Z / 131.0;

		XANGLE = (0.93 * (XANGLE + (GYROX * DT)) + (0.07 * ACCX));
		YANGLE = (0.93 * (YANGLE + (GYROY * DT)) + (0.07 * ACCY));
		ZANGLE = (0.93 * (ZANGLE + (GYROZ * DT)) + (0.07 * ACCZ)); //NOT CORRECT

		printf("XANGLE : %10lf\t YANGLE : %10lf\tZANGLE : %10lf\n", XANGLE, YANGLE, ZANGLE);
		delay(5);
	}
}
