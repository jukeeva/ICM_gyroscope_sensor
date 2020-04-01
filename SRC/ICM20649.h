
//Nargiza - I changed all register configurations

#ifndef __ICM20649_H__
#define __ICM20649_H__


#define ICM_ADDRESS 0x68 //checked
//  Register Defines
// BANK select
#define ICM20649_USER_BANK 0x7F // bank sel register 
 
// bank 0
#define ICM20649_UB0_WHO_AM_I		0x00 //

#define ICM20649_UB0_INT_STATUS	0x19 // interrupt status
#define ICM20649_UB0_INT_PIN_CFG	0x0F // interrupt conf NOT SURE
#define ICM20649_UB0_USER_CTRL		0x03

#define	ICM20649_UB0_GX_H	0x33 //gyro registers were changed 
#define	ICM20649_UB0_GX_L	0x34

#define	ICM20649_UB0_GY_H	0x35
#define	ICM20649_UB0_GY_L	0x36

#define ICM20649_UB0_GZ_H	0x37
#define ICM20649_UB0_GZ_L	0x38

#define	ICM20649_UB0_AX_H	0x2D //ACCEL registers were changed 
#define	ICM20649_UB0_AX_L	0x2E

#define	ICM20649_UB0_AY_H	0x2F
#define	ICM20649_UB0_AY_L	0x30

#define ICM20649_UB0_AZ_H	0x31
#define ICM20649_UB0_AZ_L	0x32


#define	ICM20649_UB0_TMP_H	0x39 //Nargiza -DONT CARE but was changed
#define	ICM20649_UB0_TMP_L	0x3A //Nargiza -DONT CARE but was changed

#define ICM20649_UB0_PWR_M	0x06 // pwr_m was changed 
#define ICM20649_UB0_PWR_M2 0X07 //PWR_M2 MAKES SURE THE GYRO IS ENABLED
// bank 1
/*The value in this register indicates the self-test output generated during
manufacturing tests. This value is to be used to check against subsequent self-test
outputs performed by the end user. 
SELF-TEST RESPONSE = SENSOR OUTPUT WITH SELF-TEST ENABLED – SENSOR OUTPUT WITHOUT SELF-TEST ENABLED
TO ENABLE SELF TEST one need s to write into GYRO_CONFIG_2 register 3-5 bits*/

/*SELF_TEST FOR GYRO*/
#define ICM20649_UB1_SELF_TEST_X_GYRO	0x02  
#define ICM20649_UB1_SELF_TEST_Y_GYRO	0x03
#define ICM20649_UB1_SELF_TEST_Z_GYRO	0x04

/*SELF_TEST FOR ACCEL*/
#define ICM20649_UB1_SELF_TEST_X_ACCEL	0x0E
#define ICM20649_UB1_SELF_TEST_Y_ACCEL	0x0F
#define ICM20649_UB1_SELF_TEST_Z_ACCEL	0x10

// bank 2

#define	ICM20649_UB2_GYRO_SMPLRT_DIV	0x00 //CHANGED -BANK 2
#define ICM20649_UB2_GYRO_CONFIG_1		0x01 //dlpf_fs is for FS_SEL options and stuff
#define ICM20649_UB2_GYRO_CONFIG_2		0X02

#define	ICM20649_UB2_ACCEL_SMPLRT_DIV2	0x11
#define	ICM20649_UB2_ACCEL_CONFIG_1	0x14

// bank 3
# define ICM20649_UB3_I2C_MST_CTRL 0x01 






class ICM20649
{
private:
	int8_t read(uint8_t _register);
	int16_t x_offset;
	int16_t y_offset;
	int16_t z_offset;
public:
	void init();
	int16_t read(uint8_t addressh, uint8_t addressl);
	void write(uint8_t _register, uint8_t _data);
	double getTemperature();
	
	void getXYZ(int16_t *x,int16_t *y,int16_t *z, int8_t *testX, int8_t *testY, int8_t *testZ);
	void getACC_XYZ(int16_t *a,int16_t *b,int16_t *c);
	void getG_force(float *ga,float *gb,float *gc);
	void getAngularVelocity(float *ax,float *ay,float *az);
	void zeroCalibrate(unsigned int samples, unsigned int sampleDelayMS); 
};

#endif
