/*
 * AIS328DQ.h
 *
 * Created: 20-12-2017 13:02:31
 *  Author: Nishant Sood
 */ 


#ifndef AIS328DQ_H_
#define AIS328DQ_H_

#include <stdint.h>

typedef enum {
	
	IMU_SUCCESS,
	IMU_HW_ERROR,
	IMU_NOT_SUPPORTED,
	IMU_GENERIC_ERROR,
	IMU_OUT_OF_BOUNDS,
	IMU_ALL_ONES_WARNING,
	
	}status_t;

//This struct holds the settings the driver uses to do calculations
typedef struct {
	public:
	//Accelerometer settings
	uint8_t accelEnabled   = 0x27;
	uint8_t accelCTRL_REG4 = 0x30;
	uint8_t accelODROff;
	
	uint16_t accelRange    = 8;
	uint16_t accelSampleRate;
	uint16_t accelBandWidth;
	
	private:
	
}SensorSettings;

class AIS328DQcore {
	
	public:

	AIS328DQcore(uint8_t);
	~AIS328DQcore() = default;
	status_t beginCore( void );
	
	//The following utilities read and write to the Accel.

	//ReadRegisterRegion takes a uint8 array address as input and reads
	//  a chunk of memory into that array.
	status_t readRegisterRegion(uint8_t*, uint8_t, uint8_t );
	
	//readRegister reads one 8-bit register
	status_t readRegister8(uint8_t[], uint8_t*);
	
	//Reads two 8-bit regs, LSByte then MSByte order, and concatenates them.
	//  Acts as a 16-bit read operation
	status_t readRegisterInt16(int16_t*, uint8_t reg[], uint8_t );
	
	//Writes an 8-bit byte;
	status_t writeRegister(uint8_t, uint8_t);
		
	private:	
	//Communication stuff
	uint8_t I2CAddress;
	
	};
	

//Highest level class inherits from the core to derive driver API
class AIS328DQ : public AIS328DQcore {
		
	public:
	
    SensorSettings Accel_setting;
	AIS328DQ(uint8_t i2c_add);
	~AIS328DQ() = default;
	
	//Call to apply SensorSettings
	status_t begin(void);

	//Returns the raw bits from the sensor cast as 16-bit signed integers
	void    readRawAccelX( int16_t* );
	void    readRawAccelY( int16_t* );
    void    readRawAccelZ( int16_t* );	
	
	float calcAccel( int16_t );
	
	//Returns the values as floats.  Inside, this calls readRaw___();
	float readFloatAccelX( void );
	float readFloatAccelY( void );
	float readFloatAccelZ( void );
	
	bool XYZdataAvailable( void );
	
    private:
	
	};
	
// Device registers
#define AIS328DQ_ACC_WHOM_AM_I              0x0F
#define AIS328DQ_ACC_CTRL_REG1              0x20
#define AIS328DQ_ACC_CTRL_REG2              0x21
#define AIS328DQ_ACC_CTRL_REG3              0x22
#define AIS328DQ_ACC_CTRL_REG4              0x23
#define AIS328DQ_ACC_CTRL_REG5              0x24

//Device Address
#define AIS328DQ_add                        0x18

#define AIS328DQ_ACC_HP_FILTER_RESET        0x25
#define AIS328DQ_ACC_REF                    0x26
#define AIS328DQ_ACC_STAT_REG               0x27

#define AIS328DQ_ACC_OUTX_L_XL  			0X28
#define AIS328DQ_ACC_OUTX_H_XL  			0X29
#define AIS328DQ_ACC_OUTY_L_XL  			0X2A
#define AIS328DQ_ACC_OUTY_H_XL  			0X2B
#define AIS328DQ_ACC_OUTZ_L_XL  			0X2C
#define AIS328DQ_ACC_OUTZ_H_XL  			0X2D

//Interrupt - 1 defines
#define AIS328DQ_ACC_INT1_CFG               0x30
#define AIS328DQ_ACC_INT1_SRC               0x31
#define AIS328DQ_ACC_INT1_THS               0x32
#define AIS328DQ_ACC_INT1_DUR               0x33

//Interrupt - 2 defines
#define AIS328DQ_ACC_INT2_CFG               0x34
#define AIS328DQ_ACC_INT2_SRC               0x35
#define AIS328DQ_ACC_INT2_THS               0x36
#define AIS328DQ_ACC_INT2_DUR               0x37




#endif /* AIS328DQ_H_ */