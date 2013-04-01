/*
 * main.c
 *
 *  Created on: 17 Feb 2013
 *      Author: Will
 */

#include <system.h>
#include <stdio.h>

#include <altera_avalon_pio_regs.h>
#include <unistd.h>
#include <io.h>
#include "i2c_driver/i2c_opencores.h"
#include "i2c_driver/i2c_opencores_regs.h"


#include

#include "MPU6050.h"


int i2c_write_bytes(int addr, unsigned char* data,int n);
int i2c_read_byte(int addr, unsigned char*data);
int i2c_read_bytes(int addr, unsigned char*data,int n);
int i2c_write_byte(int addr, unsigned char data);
unsigned short swap16(unsigned short data);


/* Task to be created. */
void vTaskCode( void * pvParameters )
{
  for( ;; )
  {
      /* Task code goes here. */
  }
}

/* Function that creates a task. */
void vOtherFunction( void ){

	static unsigned char ucParameterToPass;
	xTaskHandle xHandle;

	/* Create the task, storing the handle.  Note that the passed parameter
	ucParameterToPass must exist for the lifetime of the task, so in this
	case is declared static.  If it was just an an automatic stack variable
	it might no longer exist, or at least have been corrupted, by the time
	the new task attempts to access it. */
	xTaskCreate( vTaskCode, "NAME", STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY,
			   &xHandle );

	/* Use the handle to delete the task. */
	vTaskDelete( xHandle );
}







int main(){

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
		}data;

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

		printf("Read: [%05d][%05d][%05d]  [%05d][%05d][%05d].\n",
				(signed short)swap16(data.X),
				(signed short)swap16(data.Y),
				(signed short)swap16(data.Z),
				(signed short)swap16(data.gX),
						(signed short)swap16(data.gY),
						(signed short)swap16(data.gZ)

		);

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




