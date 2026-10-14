#ifndef __I2C_H
#define __I2C_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"

#define I2C1_SCL_PIN GPIO_PIN_6
#define I2C1_SCL_PORT GPIOB
#define I2C1_SDA_PIN GPIO_PIN_7
#define I2C1_SDA_PORT GPIOB

extern I2C_HandleTypeDef hi2c1;

void MX_I2C1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H */
