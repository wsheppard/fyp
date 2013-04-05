/*
 * main.c
 *
 *  Created on: 17 Feb 2013
 *      Author: Will
 */

#include <FreeRTOS.h>
#include <task.h>
#include <system.h>
#include <stdio.h>
#include <math.h>

#include <altera_avalon_pio_regs.h>
#include <unistd.h>
#include <io.h>
#include "i2c_driver/i2c_opencores.h"
#include "i2c_driver/i2c_opencores_regs.h"



#include "MPU6050.h"


/* Variables for PID */
//float P_err = 0;
//float I_err = 0;
//float D_err = 0;

float temp1;

#define KP0	300
#define KD0	-200
#define KI0	0

#define KP1	110
#define KD1	30
#define KI1	0




float pid0 (float sp, float pv);
float pid1 (float sp, float pv);

int i2c_write_bytes(int addr, unsigned char* data,int n);
int i2c_read_byte(int addr, unsigned char*data);
int i2c_read_bytes(int addr, unsigned char*data,int n);
int i2c_write_byte(int addr, unsigned char data);
unsigned short swap16(unsigned short data);

union{
		unsigned char data[12];
		struct {
			unsigned short X;
			unsigned short Y;
			unsigned short Z;
			unsigned short gX;
			unsigned short gY;
			unsigned short gZ;

		};
	}GYROdata;


typedef const signed char * fStr;

#define FACTOR 300
#define UPPER 75000
#define LOWER 65000

#define ACCEL_CENTRE 137

short tempX;
int PWM0 = 70000;
int PWM1 = 70000;
int KILL = 0;
int callcount = 0;
int accel = 0;
float accelf = 0;

// Clockwise
#define PWM0_IDLE	75000

// Anti-clockwise
#define PWM1_IDLE	75000

#define IDLESQUARE 5.625e9

int PWM0DISP,PWM1DISP;
int PIDDISP0 = 0;
int PIDDISP1 = 0;

int ACCELDISP;
int ERRORDISP0 = 0;
int ERRORDISP1 = 0;


int x = 0;


void vTaskDisplay (void*params){
	int a;

while(1){
	printf("ACCEL[%05d] PIDE:[%d:%d] PIDO:[%d:%d] PWM[%d:%d] Count[%d].\n",
					ACCELDISP,
					ERRORDISP0,
					ERRORDISP1,
					PIDDISP0,
					PIDDISP1,
					PWM0DISP,
					PWM1DISP,
					callcount);

	a = (int)IORD_ALTERA_AVALON_PIO_DATA(PIO_0_BASE);

	if(a==0)
		KILL=1;

	vTaskDelay(200);
}

}


/* Task to be created. */
void vTaskCode( void * pvParameters )
{

	float pid0_res, pid1_res;

  while(KILL!=1)
  {

		i2c_read_bytes(MPU6050_RA_ACCEL_XOUT_H,&tempX,2);
		tempX = swap16(tempX);


		/* Get an integer on the case */
		accelf = (float)tempX;
		accelf += 20000;

		/* Squareroot input to attempt linearisation */
		accelf = sqrtf(accelf);

		ACCELDISP = accelf;

		pid0_res = pid0(ACCEL_CENTRE,accelf);

#if 1
		PWM0 = PWM0_IDLE - pid0_res;
		PWM1 = PWM1_IDLE + pid0_res;

		PIDDISP0 = PWM0;
		PIDDISP1 = PWM1;

#else
		PWM0 = pid0_res*PWM0_IDLE;
		PWM1 = pid1_res*PWM1_IDLE;
#endif

#if 1
		PWM0DISP = PWM0;
		PWM1DISP = PWM1;
#endif

		if(PWM0<=100000 || PWM0>50000){
			IOWR(PWM1_BASE,0,PWM0);
		}

		if(PWM1<=100000 || PWM1>50000){
			IOWR(PWM1_BASE,1,PWM1);
		}

		callcount++;

		vTaskDelay(1);

  }

	IOWR(PWM1_BASE,0,50000);
	IOWR(PWM1_BASE,1,50000);


}



float pid0 (float sp, float pv)
{

	static float err_old = 0;
	static float err = 0;
	static float P_err = 0;
	static float D_err = 0;
	static float I_err = 0;

	err_old = err;
	err = sp - pv;

	ERRORDISP0 = err;

	// note
	P_err = err;
	I_err += err_old;
	D_err = err - err_old;

	return KP0*P_err + KI0*I_err + KD0*D_err;

}

float pid1 (float sp, float pv)
{

	static float err_old = 0;
	static float err = 0;
	static float P_err = 0;
	static float D_err = 0;
	static float I_err = 0;

	err_old = err;
	err = (sp - pv);

	ERRORDISP1 = err;

	// note
	P_err = err;
	I_err += err_old;
	D_err = err - err_old;

	return KP1*P_err + KI1*I_err + KD1*D_err;

}




/* Function that creates a task. */
void initFYP( void ){

	unsigned char temp;
	unsigned char mask = 7;

	I2C_init(
			I2C_OPENCORES_0_BASE,
			ALT_CPU_FREQ,
			100000);

	I2C_init(
			I2C_OPENCORES_0_BASE,
			ALT_CPU_FREQ,
			100000);

	/* Turn off sleep mode */
	i2c_write_byte(MPU6050_RA_PWR_MGMT_1,0);

	i2c_read_byte(MPU6050_RA_CONFIG,&temp);

	printf("Current CONFIG is: [%x].\n", temp);

	temp &= ~mask;
	temp |= 6;

	/* Turn on sharp Low Pass filter */
	i2c_write_byte(MPU6050_RA_CONFIG, temp);



	//static unsigned char ucParameterToPass;
	//xTaskHandle xHandle;

	/* Create the task, storing the handle.  Note that the passed parameter
	ucParameterToPass must exist for the lifetime of the task, so in this
	case is declared static.  If it was just an an automatic stack variable
	it might no longer exist, or at least have been corrupted, by the time
	the new task attempts to access it. */
	xTaskCreate( vTaskCode, (fStr)"PID", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 1 ,
			   NULL );

	/* Use the handle to delete the task. */
	//vTaskDelete( xHandle );

	xTaskCreate( vTaskDisplay, (fStr)"Display", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY,
			   NULL );

}



int main(){


	int a = 0;

	initFYP();

	a = (int)IORD_ALTERA_AVALON_PIO_DATA(PIO_0_BASE);


	if (a!=0){
		printf("Turn off the switches!\n");

		while(a!=0){
			a = (int)IORD_ALTERA_AVALON_PIO_DATA(PIO_0_BASE);
			usleep(1000000);
		}
	}

	printf("OK. Push first switch up after motor calibration.....\n");

	IOWR(PWM1_BASE,0,50000);
	IOWR(PWM1_BASE,1,50000);


	while(a == 0){

		a = (int)IORD_ALTERA_AVALON_PIO_DATA(PIO_0_BASE);

		//printf("Switches[%d]",a);

		usleep(1000000);

	}



	printf("And we're off.....\n");



	vTaskStartScheduler();

	return 0;

}



int oldmain(){

	int data = 0;

	printf("Started.\n");

	//testRotors();

	I2C_init(
			I2C_OPENCORES_0_BASE,
			ALT_CPU_FREQ,
			100000);

	I2C_init(
			I2C_OPENCORES_0_BASE,
			ALT_CPU_FREQ,
			100000);




	while(1){


#if 0

		I2C_start(I2C_OPENCORES_0_BASE,
				MPU6050_DEFAULT_ADDRESS,0);

		I2C_write(I2C_OPENCORES_0_BASE,0x3B,0);

		I2C_start(I2C_OPENCORES_0_BASE,
						MPU6050_DEFAULT_ADDRESS,1);

		 data =  I2C_read(I2C_OPENCORES_0_BASE,0);  // read the input register and send stop

		 data<<=8;

		 data |= 0xFF & I2C_read(I2C_OPENCORES_0_BASE,1);
#endif


		i2c_write_byte(MPU6050_RA_PWR_MGMT_1,0);

		//i2c_read_byte(MPU6050_RA_,&data);

		i2c_read_bytes(MPU6050_RA_ACCEL_XOUT_H,(unsigned char*)&data,12);
#if 0
		printf("Read: [%05d][%05d][%05d]  [%05d][%05d][%05d].\n",
				(signed short)swap16(data.X),
				(signed short)swap16(data.Y),
				(signed short)swap16(data.Z),
				(signed short)swap16(data.gX),
						(signed short)swap16(data.gY),
						(signed short)swap16(data.gZ)

		);
#endif

		usleep(1000000);
	}

	return 0;
}


unsigned short swap16(unsigned short data){

	unsigned short retval = 0;

	retval = (data<<8) | (data>>8);

	return retval;
}


int testRotors (){

	int a,b;

	a = 1;
	b = 50000;

	printf("Hello there %d, %d\n",a,b);

	IOWR(PWM1_BASE,0,50000);
	IOWR(PWM1_BASE,1,50000);
	IOWR(PWM1_BASE,2,50000);
	IOWR(PWM1_BASE,3,50000);

	while(1){

		a = (int)IORD_ALTERA_AVALON_PIO_DATA(PIO_0_BASE);

		printf("Switches[%d] PWM Value[%d]\n",a,b);

		if(a&1){

			b=50000;

		}
		else{

			if (a&(1<<1))
				b=55000;
			if (a&(1<<2))
				b=60000;
			if (a&(1<<3))
				b=75000;
			if (a&(1<<4))
				b=90000;

		}

		IOWR(PWM1_BASE,0,b);
		IOWR(PWM1_BASE,1,b);
		IOWR(PWM1_BASE,2,b);
		IOWR(PWM1_BASE,3,b);

		//if (b >= 60000) b=50000;

		usleep(2000000);
	}

	return 0;

}



int i2c_write_bytes(int addr, unsigned char* data,int n){

	int i;

	/* Start and write mode */
	I2C_start(I2C_OPENCORES_0_BASE,
				MPU6050_DEFAULT_ADDRESS,0);

	/* Address */
		I2C_write(I2C_OPENCORES_0_BASE,0xFF & addr,0);

	for(i=0;i<(n-1);i++){
		I2C_write(I2C_OPENCORES_0_BASE,*data++,0);
	}

	I2C_write(I2C_OPENCORES_0_BASE,*data,1);

	return 0;
}


int i2c_read_byte(int addr, unsigned char*data){

	int retval;

	i2c_read_bytes(addr, data,1);

	return retval;
}


int i2c_write_byte(int addr, unsigned char data){

	int retval;

	i2c_write_bytes(addr,&data,1);

	return retval;
}


int i2c_read_bytes(int addr, unsigned  char*data,int n){

	int i;

	/* Start and write mode */
	I2C_start(I2C_OPENCORES_0_BASE,
				MPU6050_DEFAULT_ADDRESS,0);

	/* Address */
		I2C_write(I2C_OPENCORES_0_BASE,0xFF & addr,0);

	/* Start again in read mode */
		I2C_start(I2C_OPENCORES_0_BASE,
						MPU6050_DEFAULT_ADDRESS,1);

		for(i=0;i<(n-1);i++){

			*data++ =  I2C_read(I2C_OPENCORES_0_BASE,0);  // read the input register and send stop

		}

		*data = I2C_read(I2C_OPENCORES_0_BASE,1);

		return 0;

}




