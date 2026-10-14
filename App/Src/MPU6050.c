#include "MPU6050.h"
#include "string.h"
#include "math.h"
#include "i2c.h"

// Function prototypes
void Gyro_Calibration(void);

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// MPU6050 registers
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_ACCEL_XOUT_L 0x3C
#define MPU6050_REG_ACCEL_YOUT_H 0x3D
#define MPU6050_REG_ACCEL_YOUT_L 0x3E
#define MPU6050_REG_ACCEL_ZOUT_H 0x3F
#define MPU6050_REG_ACCEL_ZOUT_L 0x40
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_GYRO_XOUT_L  0x44
#define MPU6050_REG_GYRO_YOUT_H  0x45
#define MPU6050_REG_GYRO_YOUT_L  0x46
#define MPU6050_REG_GYRO_ZOUT_H  0x47
#define MPU6050_REG_GYRO_ZOUT_L  0x48
#define MPU6050_REG_PWR_MGMT_1   0x6B

float acc_offset[3]={0};
float gyro_offset[3]={0};
int Gyro_calibration_times=0;
int Gyro_calibration_times2=0;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float exInt = 0, eyInt = 0, ezInt = 0;

float a[3]={0};
float w[3]={0};
float angle[3]={0};

static void MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

static HAL_StatusTypeDef MPU6050_ReadReg(uint8_t reg, uint8_t *data, uint8_t len)
{
	return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

void MPU6050_Init(void)
{
	MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x00);
	HAL_Delay(100);
	
	Gyro_Calibration();
}

void Gyro_Calibration(void)
{
	uint8_t data[14];
	int16_t gyro_x, gyro_y, gyro_z;
	float sum_x = 0, sum_y = 0, sum_z = 0;
	int count = 50;
	
	for(int i = 0; i < count; i++)
	{
		if(MPU6050_ReadReg(MPU6050_REG_ACCEL_XOUT_H, data, 14) == HAL_OK)
		{
			gyro_x = (data[8] << 8) | data[9];
			gyro_y = (data[10] << 8) | data[11];
			gyro_z = (data[12] << 8) | data[13];
			
			sum_x += gyro_x;
			sum_y += gyro_y;
			sum_z += gyro_z;
		}
		HAL_Delay(10);
	}
	
	gyro_offset[0] = sum_x / count;
	gyro_offset[1] = sum_y / count;
	gyro_offset[2] = sum_z / count;
}

void Gyroscopetest(void)
{
	uint8_t data[14];
	int16_t accel_x, accel_y, accel_z;
	int16_t gyro_x, gyro_y, gyro_z;
	
	if(MPU6050_ReadReg(MPU6050_REG_ACCEL_XOUT_H, data, 14) == HAL_OK)
	{
		accel_x = (data[0] << 8) | data[1];
		accel_y = (data[2] << 8) | data[3];
		accel_z = (data[4] << 8) | data[5];
		
		a[0] = accel_x / 16384.0 * 9.8;
		a[1] = accel_y / 16384.0 * 9.8;
		a[2] = accel_z / 16384.0 * 9.8;
		
		gyro_x = (data[8] << 8) | data[9];
		gyro_y = (data[10] << 8) | data[11];
		gyro_z = (data[12] << 8) | data[13];
		
		w[0] = (gyro_x - gyro_offset[0]) / 131.0;
		w[1] = (gyro_y - gyro_offset[1]) / 131.0;
		w[2] = (gyro_z - gyro_offset[2]) / 131.0;
		
		static float angle_x = 0, angle_y = 0, angle_z = 0;
		float dt = 0.005f;   /* 实际采样周期5ms（Task_IMU_Start每5ms调用一次），原0.1导致陀螺仪积分大20倍、角度漂移失准 */
		
		float accel_angle_x = atan2(a[1], a[2]) * 57.3f;
		float accel_angle_y = atan2(-a[0], sqrt(a[1] * a[1] + a[2] * a[2])) * 57.3f;
		
		/* 互补滤波：陀螺仪积分(快速) + 加速度计修正(慢速)，0.92/0.08加快姿态收敛 */
		angle_x = 0.92f * (angle_x + w[0] * dt) + 0.08f * accel_angle_x;
		angle_y = 0.92f * (angle_y + w[1] * dt) + 0.08f * accel_angle_y;
		angle_z += w[2] * dt;
		
		angle[0] = angle_x;
		angle[1] = angle_y;
		angle[2] = angle_z;
		
		while(angle[0] > 180) angle[0] -= 360;
		while(angle[0] < -180) angle[0] += 360;
		
		while(angle[1] > 180) angle[1] -= 360;
		while(angle[1] < -180) angle[1] += 360;
		
		while(angle[2] > 180) angle[2] -= 360;
		while(angle[2] < -180) angle[2] += 360;
	}
}

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
}

void MUP6050_Data_Process(void)
{
	Gyroscopetest();
}

void IMU_Init(void)
{
	for(int i=0; i<3; i++)
	{
		a[i] = 0;
		w[i] = 0;
		angle[i] = 0;
		acc_offset[i] = 0;
		gyro_offset[i] = 0;
	}
	
	Gyro_calibration_times = 0;
	Gyro_calibration_times2 = 0;
	
	q0 = 1;
	q1 = 0;
	q2 = 0;
	q3 = 0;
	
	exInt = 0;
	eyInt = 0;
	ezInt = 0;
	
	MPU6050_Init();
}
