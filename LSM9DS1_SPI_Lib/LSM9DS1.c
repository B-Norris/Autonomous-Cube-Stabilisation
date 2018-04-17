/* ========================================
 *
 * LSM9DS1 IMU APIs, Bailey Norris (114410958)
 *
 * ========================================
 */

#include "LSM9DS1.h"
#include "SPIM.h"


// SPI //

void IMU_Write(uint8 addr, uint8 data)
{
	uint16 txData = (addr << 8 | data);
	SPIM_WriteTxData(txData);
}

uint8 IMU_Read(uint8 addr)
{
	SPIM_WriteTxData((0x80 | addr) << 8);	//MSB must = 1, for read operator 
	return (SPIM_ReadRxData() >> 8);
    }


// Setup //

void IMU_Start()
{
	accel_Start();
	gyro_Start();
}

void accel_Start()
{
	//CTRL_REG5_XL
	IMU_Write(0x1F, 0x38);
	//CTRL_REG6_XL
	IMU_Write(0x20, 0xC0);
	//CTRL_REG7_XL
	IMU_Write(0x21, 0x00);
}

void gyro_Start()
{
	//CTRL_REG1_G
	IMU_Write(0x10, 0xC3);
	//CTRL_REG2_G
	IMU_Write(0x11, 0x00);
	//CTRL_REG3_G
	IMU_Write(0x12, 0x00);
	//CTRL_REG4
	IMU_Write(0x1E, 0x38);
	//ORIENT_CFG_G
	IMU_Write(0x13, 0x00);
}


// Check Availablility //

uint8 accelAvailable()
{
	uint8 check = IMU_Read(STATUS_REG_1);
	return (check & (1<<0));
}

uint8 gyro_Available()
{
	uint8 check = IMU_Read(STATUS_REG_1);
	return ((check & (1<<1)) >> 1);
}


// Get Accel and Gyro Data //

/* Test for pointers
uint16* read_Accel()
{
	uint8 temp[6];
    uint16 *accel[3];
	for (int i=0; i<6; i++)
	{
		temp[i] = IMU_Read(OUT_X_L_XL + i);
		*accel[0] = (temp[1] << 8) | temp[0];
		*accel[1] = (temp[3] << 8) | temp[2];
		*accel[2] = (temp[5] << 8) | temp[4];
	}
    return *accel;
}
*/

uint16 read_Accel_axis(uint8 axis)
{
    uint8 comp;
    uint16 accel_raw;
    uint16 accel;
    if (axis == 0)          // x axis
    {comp = 0; }
    else if (axis == 1)     // y axis
    {comp = 2; }
    else                    // z axis
    {comp = 4; }
    
    uint8 temp[2];
    for (int i=0; i<2; i++)
	{
		temp[i] = IMU_Read(OUT_X_L_XL + comp + i);
		accel_raw = (temp[1] << 8) | temp[0];
    }
    accel = calc_Accel(accel_raw);
    return accel;
}

uint16 calc_Accel(uint16 accel_raw)
{
	// Binary to m/s
	double aRes = 0.000061;
    uint16 accel = accel_raw * aRes;
	return accel;
}

uint16 read_Gyro_axis(uint8 axis)
{
    uint8 comp;
    uint16 gyro_raw;
    uint16 gyro;
    if (axis == 0)          // x axis
    {comp = 0; }
    else if (axis == 1)     // y axis
    {comp = 2; }
    else                    // z axis
    {comp = 4; }
    
    uint8 temp[2];
    for (int i=0; i<2; i++)
	{
		temp[i] = IMU_Read(OUT_X_L_XL + comp + i);
		gyro_raw = (temp[1] << 8) | temp[0];
    }
    gyro = calc_Gyro(gyro_raw);
    return gyro;
}

uint16 calc_Gyro(uint16 gyro_raw)
{
	// Binary to gs
	double gRes = 0.00875;
    uint16 gyro = gyro_raw * gRes;
	return gyro;
}