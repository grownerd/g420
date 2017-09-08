#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_gpio.h>

#include "defines.h"
#include "tm_stm32f4_rtc.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_onewire.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_ds18b20.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_watchdog.h"
#include "tm_stm32f4_pwm.h"
#include "tm_stm32f4_pwmin.h"
#include "tm_stm32f4_exti.h"
#include "tm_stm32f4_adc.h"
 
#include "bme280.h"
#include "gpio.h"
#include "adc.h"
#include "main.h"
#include "i2c.h"
#include "onewire.h"
#include "rtc.h"
#include "command_parser.h"
#include "flash.h"


#define	I2C_BUFFER_LEN 8

extern uint8_t capsense_chb;
extern uint8_t capsense_capdac;
extern uint16_t capsense_offset;
extern uint16_t capsense_gain;


void i2c_bus_reset(void){
  GPIO_InitTypeDef GPIO_InitStruct;
  uint8_t i = 0;
    
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
   
  GPIO_WriteBit(GPIOB, GPIO_Pin_7, 0);
  for (i=0; i<17; i++){
    GPIO_WriteBit(GPIOB, GPIO_Pin_6, 1);
    Delay(10);
    GPIO_WriteBit(GPIOB, GPIO_Pin_6, 0);
    Delay(10);
  }
}

void deinit_I2C1(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  I2C_Cmd(I2C1, DISABLE);
  I2C_DeInit(I2C1);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, DISABLE);

  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_WriteBit(GPIOB, GPIO_Pin_6, 0);
  GPIO_WriteBit(GPIOB, GPIO_Pin_7, 0);
}

void init_I2C1(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  I2C_InitTypeDef I2C_InitStruct;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  // PC1 = I2C VCC
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_WriteBit(GPIOC, GPIO_Pin_1, 1);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
   
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
  
  I2C_InitStruct.I2C_ClockSpeed = 10000;
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_OwnAddress1 = 0x00;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C1, &I2C_InitStruct);
  
  I2C_Cmd(I2C1, ENABLE);
}

/* This function issues a start condition and 
 * transmits the slave address + R/W bit
 * 
 * Parameters:
 *      I2Cx --> the I2C peripheral e.g. I2C1
 *      address --> the 7 bit slave address
 *      direction --> the tranmission direction can be:
 *                      I2C_Direction_Tranmitter for Master transmitter mode
 *                      I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction) {
  uint32_t start = TM_Time;
  // wait until I2C1 is not busy anymore
  while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)){
    if (TM_Time > (start + misc_settings.i2c_timeout)) {
      global_state.i2c_errors++;
      //TM_USART_Puts(USART2, "timeout: I2C_FLAG_BUSY\r\n");
      if (misc_settings.i2c_break_enabled) break;
    }
  }

  // Send I2C1 START condition 
  I2C_GenerateSTART(I2Cx, ENABLE);
    
  // wait for I2C1 EV5 --> Slave has acknowledged start condition
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)){
    if (TM_Time > (start + misc_settings.i2c_timeout)) {
      global_state.i2c_errors++;
      //TM_USART_Puts(USART2, "timeout: I2C_EVENT_MASTER_MODE_SELECT\r\n");
      if (misc_settings.i2c_break_enabled) break;
    }
  }
      
  // Send slave Address for write 
  I2C_Send7bitAddress(I2Cx, address, direction);
    
  /* wait for I2C1 EV6, check if 
   * either Slave has acknowledged Master transmitter or
   * Master receiver mode, depending on the transmission
   * direction
   */ 
  if(direction == I2C_Direction_Transmitter){
      while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
        if (TM_Time > (start + misc_settings.i2c_timeout)) {
          global_state.i2c_errors++;
          //TM_USART_Puts(USART2, "timeout: I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED\r\n");
          if (misc_settings.i2c_break_enabled) break;
        }
    }
  }
  else if(direction == I2C_Direction_Receiver){
      while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
        if (TM_Time > (start + misc_settings.i2c_timeout)) {
          global_state.i2c_errors++;
          //TM_USART_Puts(USART2, "timeout: I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED\r\n");
          if (misc_settings.i2c_break_enabled) break;
        }
      }
  }
}

/* This function transmits one byte to the slave device
 * Parameters:
 *      I2Cx --> the I2C peripheral e.g. I2C1 
 *      data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data) {
  uint32_t start = TM_Time;
  I2C_SendData(I2Cx, data);
  // wait for I2C1 EV8_2 --> byte has been transmitted
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
    if (TM_Time > (start + misc_settings.i2c_timeout)) {
      global_state.i2c_errors++;
      //TM_USART_Puts(USART2, "timeout: I2C_EVENT_MASTER_BYTE_TRANSMITTED\r\n");
      if (misc_settings.i2c_break_enabled) break;
    }
  }
}

/* This function reads one byte from the slave device 
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx) {
  uint32_t start = TM_Time;
  // enable acknowledge of recieved data
  I2C_AcknowledgeConfig(I2Cx, ENABLE);
  // wait until one byte has been received
  while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) ){
    if (TM_Time > (start + misc_settings.i2c_timeout)) {
      global_state.i2c_errors++;
      //TM_USART_Puts(USART2, "timeout: I2C_EVENT_MASTER_BYTE_RECEIVED\r\n");
      if (misc_settings.i2c_break_enabled) break;
    }
  }
  // read data from I2C data register and return data byte
  uint8_t data = I2C_ReceiveData(I2Cx);
  return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data 
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx) {
  uint32_t start = TM_Time;
  // disabe acknowledge of received data
  // nack also generates stop condition after last byte received
  // see reference manual for more info
  I2C_AcknowledgeConfig(I2Cx, DISABLE);
  I2C_GenerateSTOP(I2Cx, ENABLE);
  // wait until one byte has been received
  while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) ){
    if (TM_Time > (start + misc_settings.i2c_timeout)) {
      global_state.i2c_errors++;
      //TM_USART_Puts(USART2, "timeout: I2C_EVENT_MASTER_BYTE_RECEIVED\r\n");
      if (misc_settings.i2c_break_enabled) break;
    }
  }
  // read data from I2C data register and return data byte
  uint8_t data = I2C_ReceiveData(I2Cx);
  return data;
}

/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx) {
    // Send I2C1 STOP Condition 
    I2C_GenerateSTOP(I2Cx, ENABLE);
}


uint8_t i2c_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = 0;
	u8 array[I2C_BUFFER_LEN] = {0};
	u8 stringpos = 0;
	array[0] = reg_addr;

  I2C_start(I2C1, (dev_addr << 1), I2C_Direction_Transmitter);
  I2C_write(I2C1, reg_addr);
  I2C_stop(I2C1);
  I2C_start(I2C1, (dev_addr << 1), I2C_Direction_Receiver);

	for (stringpos = 0; stringpos < cnt - 1; stringpos++) {
    array[stringpos] = I2C_read_ack(I2C1);
  }
  array[stringpos] = I2C_read_nack(I2C1);

	for (stringpos = 0; stringpos < cnt; stringpos++) {
		*(reg_data + stringpos) = array[stringpos];
	}
	return (s8)iError;
}


uint8_t i2c_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = 0;
	u8 array[I2C_BUFFER_LEN];
	u8 stringpos = 0;
	array[0] = *reg_data;


	for (stringpos = 0; stringpos < cnt; stringpos++) {
		array[stringpos + 1] = *(reg_data + stringpos);
	}

  I2C_start(I2C1, (dev_addr << 1), I2C_Direction_Transmitter);
  I2C_write(I2C1, reg_addr);

	for (stringpos = 0; stringpos < cnt; stringpos++) {
    I2C_write(I2C1, array[stringpos]);
  }
  I2C_stop(I2C1);

	return (s8)iError;
}


uint16_t i2c_read16(u8 dev_addr, u8 reg_addr) {
	u16 reg_data = 0;

  I2C_start(I2C1, (dev_addr << 1), I2C_Direction_Transmitter);
  I2C_write(I2C1, reg_addr);
  I2C_stop(I2C1);
  I2C_start(I2C1, (dev_addr << 1), I2C_Direction_Receiver);

  reg_data = I2C_read_ack(I2C1);
  reg_data <<= 8;
  reg_data |= I2C_read_nack(I2C1);

	return reg_data;
}

void i2c_write16(u8 dev_addr, u8 reg_addr, u16 data) {
  I2C_start(I2C1, (dev_addr << 1), I2C_Direction_Transmitter);
  I2C_write(I2C1, reg_addr);

  I2C_write(I2C1, (uint8_t) data >> 8);
  I2C_write(I2C1, (uint8_t) data);
  I2C_stop(I2C1);

}

uint8_t fdc1004_init(void){
  if (i2c_read16(0x50, 0xfe) == 0x5449 && i2c_read16(0x50, 0xff) == 0x1004){
    i2c_write16(0x50, 0x0c, (uint16_t)(0x01 << 15));
    return 1;
  }else{
    return 0;
  }
}

uint32_t fdc_read_channel(uint8_t cha, uint8_t chb, uint8_t capdac, uint16_t offset, uint16_t gain){
  uint16_t conf_reg = i2c_read16(0x50, 0x0c);

  // set Measurement Configuration Register: CHA = cha, CHB = CAPDAC, no offset
  i2c_write16(0x50, 0x08 + cha, (uint16_t)(cha << 13 | chb << 10 | capdac << 5));
  i2c_write16(0x50, 0x0d + cha, offset);
  //i2c_write16(0x50, 0x11 + cha, gain);

  i2c_write16(0x50, 0x0c, (uint16_t)(0x01 << 10 | 0x00 << 8 | 1 << (7 - cha)));

  while (!conf_reg & (1 << (3 - cha))){
    conf_reg = i2c_read16(0x50, 0x0c);
  }

  uint16_t msb = i2c_read16(0x50, cha * 2);
  uint16_t lsb = i2c_read16(0x50, (cha * 2) + 1);

  return (uint32_t) ((msb << 16) | lsb) >> 8;
  
}

void fdc1004_read(uint32_t * raw){
  uint8_t i;
  for (i=0; i<2; i++) {
    raw[i] = fdc_read_channel(i, capsense_chb, capsense_capdac, capsense_offset, capsense_gain);
  }
}
