#ifndef __ASC5600__H_
#define __ASC5600__H_

#include "gpio.h"

#define I2C_SCL  LL_GPIO_PIN_15
#define I2C_SDA  LL_GPIO_PIN_14
#define I2C_SCL_PORT GPIOB
#define I2C_SDA_PORT GPIOB

#define I2C_SCL_UP()   LL_GPIO_SetOutputPin(I2C_SCL_PORT, I2C_SCL)
#define I2C_SCL_DOWM() LL_GPIO_ResetOutputPin(I2C_SCL_PORT, I2C_SCL)
#define I2C_SDA_UP()   LL_GPIO_SetOutputPin(I2C_SDA_PORT, I2C_SDA)
#define I2C_SDA_DOWM() LL_GPIO_ResetOutputPin(I2C_SDA_PORT, I2C_SDA)

#define I2C_SDA_READ LL_GPIO_IsInputPinSet(I2C_SDA_PORT, I2C_SDA)

void I2C_write_data_u8(uint8_t data);
uint8_t I2C_read_data_u8(void);

void AS5600_Init(void);
uint8_t AS_read_reg_u8(uint8_t reg);
uint16_t AS_read_reg_u16(uint8_t reg);
uint16_t AS_angle_read(void);
uint16_t AS_angle_read_filter(void);

void AS_write_reg_u8(uint8_t reg, uint8_t data);
void AS_write_reg_u16(uint8_t reg, uint16_t data);


#endif

