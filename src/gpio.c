#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_gpio.h>

#include "defines.h"
#include "tm_stm32f4_rtc.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_watchdog.h"
 
#include "gpio.h"
#include "main.h"
#include "rtc.h"
#include "command_parser.h"
#include "flash.h"



void gpio_init() {

  GPIO_InitTypeDef GPIO_InitStruct;
  
  gpio_outputs[GPIO_OUTPUT_FILL_PUMP].gpio_port = GPIOE;
  gpio_outputs[GPIO_OUTPUT_FILL_PUMP].gpio_pin = GPIO_Pin_9;
  gpio_outputs[GPIO_OUTPUT_FILL_PUMP].desired_state = 0;
  gpio_outputs[GPIO_OUTPUT_FILL_PUMP].run_for_ms = 0;
  gpio_outputs[GPIO_OUTPUT_DRAIN_PUMP].gpio_port = GPIOE;
  gpio_outputs[GPIO_OUTPUT_DRAIN_PUMP].gpio_pin = GPIO_Pin_14;
  gpio_outputs[GPIO_OUTPUT_DRAIN_PUMP].desired_state = 0;
  gpio_outputs[GPIO_OUTPUT_DRAIN_PUMP].run_for_ms = 0;
 

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOE, &GPIO_InitStruct);
   
}



