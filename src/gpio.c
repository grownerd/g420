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

TM_PWM_TIM_t TIM1_Data, TIM3_Data, TIM4_Data, TIM9_Data, TIM12_Data;
TM_PWMIN_t PWMIN1_Data, PWMIN2_Data;


void gpio_init() {

  relays[RELAY_LIGHT].gpio_pin = GPIO_Pin_9;
  relays[RELAY_LIGHT].gpio_port = GPIOD;
  relays[RELAY_EXHAUST].gpio_pin = GPIO_Pin_10;
  relays[RELAY_EXHAUST].gpio_port = GPIOD;
  relays[RELAY_AUX].gpio_pin = GPIO_Pin_11;
  relays[RELAY_AUX].gpio_port = GPIOD;

  GPIO_InitTypeDef GPIO_InitStruct;
  
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
  // Pin 8 is used for the EC meter
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOD, &GPIO_InitStruct);
   
#if 1

  gpio_outputs[GPIO_OUTPUT_FILL_PUMP].gpio_port = GPIOE;
  gpio_outputs[GPIO_OUTPUT_FILL_PUMP].gpio_pin = GPIO_Pin_9;
  gpio_outputs[GPIO_OUTPUT_FILL_PUMP].desired_state = 0;
  gpio_outputs[GPIO_OUTPUT_FILL_PUMP].run_for_ms = 0;
  gpio_outputs[GPIO_OUTPUT_DRAIN_PUMP].gpio_port = GPIOE;
  gpio_outputs[GPIO_OUTPUT_DRAIN_PUMP].gpio_pin = GPIO_Pin_14;
  gpio_outputs[GPIO_OUTPUT_DRAIN_PUMP].desired_state = 0;
  gpio_outputs[GPIO_OUTPUT_DRAIN_PUMP].run_for_ms = 0;
  gpio_outputs[GPIO_OUTPUT_COOLANT_PUMP].gpio_port = GPIOB;
  gpio_outputs[GPIO_OUTPUT_COOLANT_PUMP].gpio_pin = GPIO_Pin_4;
  gpio_outputs[GPIO_OUTPUT_COOLANT_PUMP].desired_state = 0;
  gpio_outputs[GPIO_OUTPUT_COOLANT_PUMP].run_for_ms = 0xffffffff;
  gpio_outputs[GPIO_OUTPUT_SEWAGE_PUMP].gpio_port = GPIOB;
  gpio_outputs[GPIO_OUTPUT_SEWAGE_PUMP].gpio_pin = GPIO_Pin_5;
  gpio_outputs[GPIO_OUTPUT_SEWAGE_PUMP].desired_state = 0;
  gpio_outputs[GPIO_OUTPUT_SEWAGE_PUMP].run_for_ms = 0;
  gpio_outputs[GPIO_OUTPUT_DEHUMI_PUMP].gpio_port = GPIOC;
  gpio_outputs[GPIO_OUTPUT_DEHUMI_PUMP].gpio_pin = GPIO_Pin_8;
  gpio_outputs[GPIO_OUTPUT_DEHUMI_PUMP].desired_state = 0;
  gpio_outputs[GPIO_OUTPUT_DEHUMI_PUMP].run_for_ms = 0;
  gpio_outputs[GPIO_OUTPUT_PHDOWN_PUMP].gpio_port = GPIOC;
  gpio_outputs[GPIO_OUTPUT_PHDOWN_PUMP].gpio_pin = GPIO_Pin_9;
  gpio_outputs[GPIO_OUTPUT_PHDOWN_PUMP].desired_state = 0;
  gpio_outputs[GPIO_OUTPUT_PHDOWN_PUMP].run_for_ms = 0;
  gpio_outputs[GPIO_OUTPUT_NUTRIENT1_PUMP].gpio_port = GPIOB;
  gpio_outputs[GPIO_OUTPUT_NUTRIENT1_PUMP].gpio_pin = GPIO_Pin_8;
  gpio_outputs[GPIO_OUTPUT_NUTRIENT1_PUMP].desired_state = 0;
  gpio_outputs[GPIO_OUTPUT_NUTRIENT1_PUMP].run_for_ms = 0;
  gpio_outputs[GPIO_OUTPUT_NUTRIENT2_PUMP].gpio_port = GPIOE;
  gpio_outputs[GPIO_OUTPUT_NUTRIENT2_PUMP].gpio_pin = GPIO_Pin_5;
  gpio_outputs[GPIO_OUTPUT_NUTRIENT2_PUMP].desired_state = 0;
  gpio_outputs[GPIO_OUTPUT_NUTRIENT2_PUMP].run_for_ms = 0;
  gpio_outputs[GPIO_OUTPUT_NUTRIENT3_PUMP].gpio_port = GPIOE;
  gpio_outputs[GPIO_OUTPUT_NUTRIENT3_PUMP].gpio_pin = GPIO_Pin_6;
  gpio_outputs[GPIO_OUTPUT_NUTRIENT3_PUMP].desired_state = 0;
  gpio_outputs[GPIO_OUTPUT_NUTRIENT3_PUMP].run_for_ms = 0;
  gpio_outputs[GPIO_OUTPUT_DEEP_RED_LEDS].gpio_port = GPIOB;
  gpio_outputs[GPIO_OUTPUT_DEEP_RED_LEDS].gpio_pin = GPIO_Pin_14;
  gpio_outputs[GPIO_OUTPUT_DEEP_RED_LEDS].desired_state = 0;
  gpio_outputs[GPIO_OUTPUT_DEEP_RED_LEDS].run_for_ms = 0;
  gpio_outputs[GPIO_OUTPUT_STIRRER_MOTORS].gpio_port = GPIOB;
  gpio_outputs[GPIO_OUTPUT_STIRRER_MOTORS].gpio_pin = GPIO_Pin_15;
  gpio_outputs[GPIO_OUTPUT_STIRRER_MOTORS].desired_state = 0;
  gpio_outputs[GPIO_OUTPUT_STIRRER_MOTORS].run_for_ms = 0;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
   
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
   
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_9 | GPIO_Pin_14;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOE, &GPIO_InitStruct);
   
#endif
}


void exti_init(){
  irqs[SWITCH_BLUE_BUTTON].gpio_port = GPIOA;
  irqs[SWITCH_BLUE_BUTTON].gpio_pin = GPIO_Pin_0;

  // no pin 1

  irqs[SWITCH_RES_MAX].gpio_port = GPIOE;
  irqs[SWITCH_RES_MAX].gpio_pin = GPIO_Pin_2;

  // no pin 3

  irqs[SWITCH_RES_MIN].gpio_port = GPIOE;
  irqs[SWITCH_RES_MIN].gpio_pin = GPIO_Pin_4;

  irqs[SWITCH_RES_ALARM].gpio_port = GPIOC;
  irqs[SWITCH_RES_ALARM].gpio_pin = GPIO_Pin_5;

  irqs[SWITCH_UNUSED_6].gpio_port = GPIOC;
  irqs[SWITCH_UNUSED_6].gpio_pin = GPIO_Pin_6;

  irqs[SWITCH_DEHUMI_MAX].gpio_port = GPIOE;
  irqs[SWITCH_DEHUMI_MAX].gpio_pin = GPIO_Pin_7;

  irqs[SWITCH_DEHUMI_MIN].gpio_port = GPIOE;
  irqs[SWITCH_DEHUMI_MIN].gpio_pin = GPIO_Pin_8;

  // no pin 9

  irqs[SWITCH_SEWAGE_MAX].gpio_port = GPIOE;
  irqs[SWITCH_SEWAGE_MAX].gpio_pin = GPIO_Pin_10;

  irqs[SWITCH_SEWAGE_MIN].gpio_port = GPIOE;
  irqs[SWITCH_SEWAGE_MIN].gpio_pin = GPIO_Pin_11;

  irqs[SWITCH_UNUSED_12].gpio_port = GPIOE;
  irqs[SWITCH_UNUSED_12].gpio_pin = GPIO_Pin_12;

  irqs[SWITCH_UNUSED_13].gpio_port = GPIOE;
  irqs[SWITCH_UNUSED_13].gpio_pin = GPIO_Pin_13;

  // no pin 14

  irqs[SWITCH_WATER_EMPTY].gpio_port = GPIOE;
  irqs[SWITCH_WATER_EMPTY].gpio_pin = GPIO_Pin_15;

  TM_EXTI_Attach(GPIOA, GPIO_Pin_0, TM_EXTI_Trigger_Rising_Falling); // Blue Discovery Button
  // no pin 1
  TM_EXTI_Attach(GPIOE, GPIO_Pin_2, TM_EXTI_Trigger_Rising_Falling); // Res max
  // no pin 3
  TM_EXTI_Attach(GPIOE, GPIO_Pin_4, TM_EXTI_Trigger_Rising_Falling); // Res min
  TM_EXTI_Attach(GPIOC, GPIO_Pin_5, TM_EXTI_Trigger_Rising_Falling); // Res Alarm
  TM_EXTI_Attach(GPIOC, GPIO_Pin_6, TM_EXTI_Trigger_Rising_Falling);
  TM_EXTI_Attach(GPIOE, GPIO_Pin_7, TM_EXTI_Trigger_Rising_Falling); // Dehumi full
  TM_EXTI_Attach(GPIOE, GPIO_Pin_8, TM_EXTI_Trigger_Rising_Falling); // Dehumi empty
  // no pin 9
  TM_EXTI_Attach(GPIOE, GPIO_Pin_10, TM_EXTI_Trigger_Rising_Falling); // sewage tank full
  TM_EXTI_Attach(GPIOE, GPIO_Pin_11, TM_EXTI_Trigger_Rising_Falling); // sewage tank empty
  TM_EXTI_Attach(GPIOE, GPIO_Pin_12, TM_EXTI_Trigger_Rising_Falling);
  TM_EXTI_Attach(GPIOE, GPIO_Pin_13, TM_EXTI_Trigger_Rising_Falling);
  // no pin 14
  TM_EXTI_Attach(GPIOE, GPIO_Pin_15, TM_EXTI_Trigger_Rising_Falling);
}


void pwmin_init(){

  TM_PWMIN_InitTimer(TIM2, &PWMIN1_Data, TM_PWMIN_Channel_1, TM_PWMIN_PinsPack_3, 1, TIM2_IRQn); // PA15
  TM_PWMIN_InitTimer(TIM2, &PWMIN2_Data, TM_PWMIN_Channel_2, TM_PWMIN_PinsPack_1, 1, TIM2_IRQn); // PA1

}

void TIM2_IRQHandler(void) {
    /* Interrupt request, don't forget! */
    TM_PWMIN_InterruptHandler(&PWMIN1_Data);
    TM_PWMIN_InterruptHandler(&PWMIN2_Data);
}

#if 0

void timer_interrupt_init (void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  RCC_ClocksTypeDef RCC_Clocks;
  /* Enable the timer global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init (&amp;NVIC_InitStructure);
}

void timer_init (void)
{
  
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq (&amp;RCC_Clocks);
  uint32_t multiplier;
  if (RCC_Clocks.PCLK1_Frequency == RCC_Clocks.SYSCLK_Frequency) {
    multiplier = 1;
  } else {
    multiplier = 2;
  }
  uint32_t TIM3CLK_Frequency = multiplier * RCC_Clocks.PCLK1_Frequency;
  uint32_t TIM3COUNTER_Frequency = 1000000;
  uint16_t prescaler = (TIM3CLK_Frequency / TIM3COUNTER_Frequency) - 1;
  uint16_t reload = (25) - 1;
  
  /* make sure the peripheral is clocked */
  RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3, ENABLE);
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  /* set everything back to default values */
  TIM_TimeBaseStructInit (&amp;TIM_TimeBaseStructure);
  /* only changes from the defaults are needed */
  TIM_TimeBaseStructure.TIM_Period = reload;
  TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
  TIM_TimeBaseInit (TIM3, &amp;TIM_TimeBaseStructure);
}

void timer_start (void)
{
  TIM_Cmd (TIM3, ENABLE);
}

void timer_stop (void)
{
  TIM_Cmd (TIM3, DISABLE);
}

void timer_interrupt_enable (void)
{
  /*
   * It is important to clear any pending interrupt flags since the timer
   * has been free-running since we last used it and that will generate
   * interrupts on overflow even though the associated interrupt event has
   * not been enabled.
   */
  TIM_ClearITPendingBit (TIM3, TIM_IT_Update);
  /* put the counter into a known state */
  TIM_SetCounter (TIM3, 0);
  TIM_ITConfig (TIM3, TIM_IT_Update, ENABLE);
}

void timer_interrupt_disable (void)
{
  TIM_ITConfig (TIM3, TIM_IT_Update, DISABLE);
}



void dosing_pump_timer_init(){
  /* make sure the peripheral is clocked */
  RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM3, ENABLE);
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq (&RCC_Clocks);
  uint32_t multiplier;
  if (RCC_Clocks.PCLK1_Frequency == RCC_Clocks.SYSCLK_Frequency) {
    multiplier = 1;
  } else {
    multiplier = 2;
  }
  uint32_t TIM3CLK_Frequency = multiplier * RCC_Clocks.PCLK1_Frequency;
  uint32_t TIM3COUNTER_Frequency = 1024;
  uint16_t prescaler = (TIM3CLK_Frequency / TIM3COUNTER_Frequency) - 1;
  uint16_t reload = (25) - 1;    // Tevt = 25 us
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  /* set everything back to default values */
  TIM_TimeBaseStructInit (&TIM_TimeBaseStructure);
  /* only changes from the defaults are needed */
  TIM_TimeBaseStructure.TIM_Period = reload;
  TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
  TIM_TimeBaseInit (TIM3, &TIM_TimeBaseStructure);
}
#endif

void TIM3_IRQHandler(void) {
  uint8_t i, j = 0;
  if (TIM_GetITStatus (TIM3, TIM_IT_Update) != RESET) {
    for (i=0; i<NUM_NUTRIENT_PUMPS; i++){
      uint8_t j = nutrient_pumps[i].gpio_output;
      gpio_outputs[j].run_for_ms = 0;
      gpio_outputs[j].desired_state = 0;
      GPIO_WriteBit(gpio_outputs[j].gpio_port, gpio_outputs[j].gpio_pin, 0);
    }
    gpio_outputs[GPIO_OUTPUT_PHDOWN_PUMP].run_for_ms = 0;
    gpio_outputs[GPIO_OUTPUT_PHDOWN_PUMP].desired_state = 0;
    GPIO_WriteBit(gpio_outputs[GPIO_OUTPUT_PHDOWN_PUMP].gpio_port, gpio_outputs[j].gpio_pin, 0);
    TIM_ClearITPendingBit (TIM3, TIM_IT_Update);
  }
}

void switch_relay(output_relay_struct_t * relay, uint8_t action){
  
  uint8_t desired_state = !GPIO_ReadInputDataBit(relay->gpio_port, relay->gpio_pin);
  switch(action){
    case ON:
      GPIO_WriteBit(relay->gpio_port, relay->gpio_pin, 1);
      break;

    case OFF:
      GPIO_WriteBit(relay->gpio_port, relay->gpio_pin, 0);
      break;

    case TOGGLE:
      GPIO_WriteBit(relay->gpio_port, relay->gpio_pin, desired_state);
      break;

  }

}
