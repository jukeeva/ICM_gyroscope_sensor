#include <Arduino.h>
#include <Wire.h>
#include "ICM20649.h"

enum Gyr_scale { // function for more convenient degree per sec range selection writes to gyro conf1 register
  GFS_500D = 0x00, // 500 degrees per sec range, THE LAST ONE ACTIVATES THE FCHOICE, IF FCHOICE IS NOT ACTIVATED THEN THE GYRO WON'T WORK
  GFS_1000D = 0x02, // 1000 degrees per sec range 
  GFS_2000D =0x04, // 2000  degrees per sec range
  GFS_4000D =0x06 // 4000  degrees per sec range
};
Gyr_scale Rot_velocity= GFS_500D; //select a rate DEGREE  PER SECOND option from enum Gyr_scale selection and assigns is to Rot_velocity

enum Acc_scale { // function for more convenient degree per sec range selection writes to acc conf1 register
  AFS_4G = 0x00, // 4  per sec range, THE LAST ONE ACTIVATES THE FCHOICE, IF FCHOICE IS NOT ACTIVATED THEN THE GYRO WON'T WORK
  AFS_8G = 0x02, // 8 per sec range 
  AFS_16G =0x04, // 16 per sec range
  AFS_30G =0x06 // 30  per sec range fchoice is activated for all
};
Acc_scale GForce= AFS_30G;


enum GYRO_DLPF_CHOICE { // function for more convenient 
  NO_DLPF= 0X00,
  DLPF_197 = 0x01, // F_CHOICE enabled everywhere except for NO_DLPF 
  DLPF_152 = 0x09, // 152 HZ bandwidth
  DLPF_120 =0x11, //  120 HZ 
  DLPF_51 =0x19, //   51 HZ etc.
  DLPF_24 = 0x21,
  DLPF_12 = 0x29,
  DLPF_6  = 0x31,
  DLPF_361 = 0x39

};

GYRO_DLPF_CHOICE GYRO_DLPF_CONF= DLPF_6;
 
uint8_t GYRO_CONF1_DATA=Rot_velocity | GYRO_DLPF_CONF;

/**********************************************************************/
/*Function: Read a byte with the register address of ICM20649.         */
/*Parameter:-uint8_t _register,the register address  of ICM20649 to read; */
/*Return:	-int8_t,the data byte that is read from the register.		  */
int8_t ICM20649::read(uint8_t _register)
{
    int8_t data;
    Wire.beginTransmission(ICM_ADDRESS);
    Wire.write(_register);
    Wire.endTransmission();
    Wire.requestFrom(ICM_ADDRESS, 1);
    if(Wire.available() > 0)
    {
        data = Wire.read();
    }
	Wire.endTransmission();
    return data;
}
/*Function: Write a byte to the register */
void ICM20649::write(uint8_t _register, uint8_t _data)
{
	Wire.begin();
	Wire.beginTransmission(ICM_ADDRESS);
	Wire.write(_register);
	Wire.write(_data);
	Wire.endTransmission();
}

/**********************************************************************/
/*Function: Initialization for ICM20649.         					  */
void ICM20649::init()
{
	Wire.begin();
	write(ICM20649_USER_BANK, 0x00); // USER BANK SELECT  0x00 0x10 0x20 0x30 
	
	write(ICM20649_UB0_PWR_M,0x01);//!!! internal clk has to be set to the first option
	
	write(ICM20649_UB0_PWR_M2, 0x00); // enable both gyro and acc
 	
	write(ICM20649_USER_BANK, 0x30);
	write(ICM20649_UB3_I2C_MST_CTRL, 0x07); // SET I2C_MST_CLK as recommended in use notes p 80
	
 	write(ICM20649_USER_BANK, 0x20); // bank select  0x00 0x10 0x20 0x30
	write(ICM20649_UB2_GYRO_CONFIG_1, GYRO_CONF1_DATA);//degrees/s range assignation, refer to at line 5 and line 11
	write(ICM20649_UB2_GYRO_SMPLRT_DIV,0x00);//sample rate divider, select diggest frequency possible
	write(ICM20649_UB2_GYRO_CONFIG_2,0x38); // averaging 1x and self test enabled
	
	write(ICM20649_UB2_ACCEL_SMPLRT_DIV2,0x00);//
	write(ICM20649_UB2_ACCEL_CONFIG_1,GForce);//
	
	write(ICM20649_USER_BANK, 0x00); //CHANGE TO USER BANK 0
	
}

int16_t ICM20649::read(uint8_t addressh, uint8_t addressl)
{
    int data, t_data;

    Wire.beginTransmission(ICM_ADDRESS);
    Wire.write(addressh);
    Wire.endTransmission();
    Wire.requestFrom(ICM_ADDRESS, 1);
    if(Wire.available() > 0)
    {
        t_data = Wire.read();
        data = t_data << 8;
    }
    Wire.beginTransmission(ICM_ADDRESS);
    Wire.write(addressl);
    Wire.endTransmission();
	Wire.requestFrom(ICM_ADDRESS, 1);
    if(Wire.available() > 0)
    {
        data |= Wire.read();
    }
    return data;
}


void ICM20649::getXYZ(int16_t *x,int16_t *y,int16_t *z,int8_t *testX,int8_t *testY,int8_t *testZ)
{ 
	 
	*x = read(ICM20649_UB0_GX_H,ICM20649_UB0_GX_L);//+x_offset; //offset is calculated on the last lines.
	*y = read(ICM20649_UB0_GY_H,ICM20649_UB0_GY_L);//+y_offset; //offset is calculated on the last lines.
	*z = read(ICM20649_UB0_GZ_H,ICM20649_UB0_GZ_L);//+z_offset; //offset is calculated on the last lines.
	*testX=read(ICM20649_UB1_SELF_TEST_X_GYRO);
	*testY=read(ICM20649_UB1_SELF_TEST_Y_GYRO);
	*testZ=read(ICM20649_UB1_SELF_TEST_Z_GYRO);
}

void ICM20649::getACC_XYZ(int16_t *a,int16_t *b,int16_t *c)
{ 
	 
	*a = read(ICM20649_UB0_AX_H,ICM20649_UB0_AX_L);
	*b = read(ICM20649_UB0_AY_H,ICM20649_UB0_AY_L);
	*c = read(ICM20649_UB0_AZ_H,ICM20649_UB0_AZ_L);
	
}
/*Function: Get the angular velocity and its unit is degree per second.*/
void ICM20649::getAngularVelocity(float *ax,float *ay,float *az)
{
	
  int16_t x,y,z;
  
	getXYZ(&x,&y,&z);

 if(Rot_velocity==0x00){  //  FS_SEL=0 refer to lines 5, 11  and 56, easier degree per sec selection.  
		// Rot_velocity=0x00 500 degree/sec range case
  	*ax = x/65.5; 
  	*ay = y/65.5;
  	*az = z/65.5;
  }
else if(Rot_velocity==0x02){ //  FS_SEL=1 

    *ax = x/32.8;
    *ay = y/32.8;
    *az = z/32.8;

  }
else if(Rot_velocity==0x04) { // FS_SEL=2

    *ax = x/16.4;
    *ay = y/16.4;
    *az = z/16.4;
  }
else if (Rot_velocity==0x06){ // FS_SEL=3

    *ax = x/8.2;
  	*ay = y/8.2;
  	*az = z/8.2;
  }
  else {
    *ax = x;
  	*ay = y;
  	*az = z;
  }

}

void ICM20649::getG_force(float *ga,float *gb,float *gc)
{
	
  int16_t a,b,c;
 
	getACC_XYZ(&a,&b,&c);

 if(GForce==0x00){  //  FS_SEL=0 refer to lines 5, 11  and 56, easier degree per sec selection.  
		//
  	*ga = a/8192.0; 
  	*gb = b/8192.0;
  	*gc = c/8192.0;
  }
else if(GForce==0x02){ //  FS_SEL=1 

    *ga = a/4096.0;
    *gb = b/4096.0;
    *gc = c/4096.0;

  }
else if(GForce==0x04) { // FS_SEL=2

    *ga = a/2048.0;
    *gb = b/2048.0;
    *gc = c/2048.0;
  }
else if (GForce==0x06){ // FS_SEL=3

    *ga = a/1024.0;
  	*gb = b/1024.0;
  	*gc = c/1024.0;
  }
  else {
    *ga = a;
  	*gb = b;
  	*gc = c;
  }
  }
/////////////////////////////////////////////
/////////////////////////////////////////
/*
void ICM20649::zeroCalibrate(unsigned int samples, unsigned int sampleDelayMS)
{
  int16_t x_offset_temp = 0;
  int16_t y_offset_temp = 0;
  int16_t z_offset_temp = 0;
  int16_t x,y,z;
  int8_t d;
  x_offset = 0;
  y_offset = 0;
  z_offset = 0;
  getXYZ(&x,&y,&z, &d);//
  for (int i = 0;i < samples;i++){
    delay(sampleDelayMS);
    getXYZ(&x,&y,&z,&d);
    x_offset_temp += x;
    y_offset_temp += y;
    z_offset_temp += z;
  }

  x_offset = abs(x_offset_temp)/samples;
  y_offset = abs(y_offset_temp)/samples;
  z_offset = abs(z_offset_temp)/samples;
  if(x_offset_temp > 0)x_offset = -x_offset;
  if(y_offset_temp > 0)y_offset = -y_offset;
  if(z_offset_temp > 0)z_offset = -z_offset;

}
*/
/*           temperature sensor.                                */
/*Function: Get the temperature from ICM20649 that with a on-chip*/
/*double ICM20649::getTemperature()
{ 
	write(ICM20649_USER_BANK, 0x00);
	int temp;
	double temperature;
	temp = read(ICM20649_UB0_TMP_H, ICM20649_UB0_TMP_L);
	temperature = 35+ ((double) (temp + 13200)) / 280;
	return(temperature);
}*/
/*Function: Get the contents of the registers in the ICM20649*/
/*          so as to calculate the angular velocity.        */
