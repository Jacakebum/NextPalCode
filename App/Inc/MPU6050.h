#include "Inverse_K.h"
#define Kp 10.0f
#define Ki 0.001f
#define halfT 0.005f

extern float a[3];
extern float w[3];
extern float angle[3];

void Gyroscopetest(void);

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);

void MUP6050_Data_Process(void);

void IMU_Init(void);
