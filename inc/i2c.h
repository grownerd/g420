/**
 * @filename rtc_i2c.h
 * @description I2C support library for stm32f4 board
 * @author Nicholas Shrake, <shraken@gmail.com>
 */

#ifndef RTC_I2C_H
#define RTC_I2C_H

/**
 * CONSTANTS
 */

/**
 * PROTOTYPES
 */

void deinit_I2C1(void);
void i2c_bus_reset(void);
void init_I2C1(void);
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
void I2C_stop(I2C_TypeDef* I2Cx);

uint8_t fdc1004_init(void);
void fdc1004_read(uint32_t * capsense_data);

extern uint8_t i2c_errors;

#endif
