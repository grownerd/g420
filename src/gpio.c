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

TM_PWM_TIM_t TIM1_Data, TIM3_Data, TIM4_Data, TIM9_Data;
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
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStruct);
   
}


void exti_init(){
  irqs[SWITCH_BLUE_BUTTON].gpio_port = GPIOA;
  irqs[SWITCH_BLUE_BUTTON].gpio_pin = GPIO_Pin_0;

  irqs[SWITCH_RES_MIN].gpio_port = GPIOD;
  irqs[SWITCH_RES_MIN].gpio_pin = GPIO_Pin_1;

  irqs[SWITCH_RES_MAX].gpio_port = GPIOE;
  irqs[SWITCH_RES_MAX].gpio_pin = GPIO_Pin_2;

  irqs[SWITCH_RES_ALARM].gpio_port = GPIOD;
  irqs[SWITCH_RES_ALARM].gpio_pin = GPIO_Pin_3;

  irqs[SWITCH_UNUSED_4].gpio_port = GPIOE;
  irqs[SWITCH_UNUSED_4].gpio_pin = GPIO_Pin_4;

  irqs[SWITCH_UNUSED_5].gpio_port = GPIOC;
  irqs[SWITCH_UNUSED_5].gpio_pin = GPIO_Pin_5;

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

  irqs[SWITCH_UNUSED_15].gpio_port = GPIOE;
  irqs[SWITCH_UNUSED_15].gpio_pin = GPIO_Pin_15;

  TM_EXTI_Attach(GPIOA, GPIO_Pin_0, TM_EXTI_Trigger_Rising_Falling); // Blue Discovery Button
  TM_EXTI_Attach(GPIOD, GPIO_Pin_1, TM_EXTI_Trigger_Rising_Falling); // Res min
  TM_EXTI_Attach(GPIOE, GPIO_Pin_2, TM_EXTI_Trigger_Rising_Falling); // Res max
  TM_EXTI_Attach(GPIOD, GPIO_Pin_3, TM_EXTI_Trigger_Rising_Falling); // Res Alarm
  TM_EXTI_Attach(GPIOE, GPIO_Pin_4, TM_EXTI_Trigger_Rising_Falling);
  TM_EXTI_Attach(GPIOC, GPIO_Pin_5, TM_EXTI_Trigger_Rising_Falling);
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

void reservoir_level_irq_handler(uint16_t GPIO_Pin)
{

  switch (GPIO_Pin) {
    // tank min
    case GPIO_PIN_1:
      // nc switch to VCC closed -> GPIO_Pin == HIGH
      if (global_state.reservoir_state == NORMAL_IDLE) {
        if (GPIO_ReadInputDataBit(irqs[SWITCH_RES_MIN].gpio_port, GPIO_Pin)){
          pwms[PWM_FILL_PUMP].duty_percent = 100;
          global_state.reservoir_state = NORMAL_FILLING;
        }
      }
      break;

    // tank max
    case GPIO_PIN_2:
      // nc switch to VCC opened -> GPIO_Pin == LOW
      if (!misc_settings.fill_to_alarm_level) {
        if (!GPIO_ReadInputDataBit(irqs[SWITCH_RES_MAX].gpio_port, GPIO_Pin)){
          if (global_state.reservoir_state == DRAIN_CYCLE_FILLING)
            global_state.reservoir_state == DRAIN_CYCLE_FULL;

          pwms[PWM_FILL_PUMP].duty_percent = 0;
        }
      }
      break;

    // tank alarm
    case GPIO_PIN_3:
      // nc switch to VCC opened -> GPIO_Pin == LOW
      if (!GPIO_ReadInputDataBit(irqs[SWITCH_RES_ALARM].gpio_port, GPIO_Pin)){
        if (global_state.reservoir_state == DRAIN_CYCLE_FILLING)
          global_state.reservoir_state == DRAIN_CYCLE_FULL;

        pwms[PWM_FILL_PUMP].duty_percent = 0;
      }
      break;
  }
}

void sewage_level_irq_handler(uint16_t GPIO_Pin)
{

  switch (GPIO_Pin) {
    // tank full
    case GPIO_PIN_10:
      // nc switch to VCC opened -> GPIO_Pin == LOW
      if (!GPIO_ReadInputDataBit(irqs[SWITCH_SEWAGE_MAX].gpio_port, GPIO_Pin))
        pwms[PWM_SEWAGE_PUMP].duty_percent = 100;
      break;

    // tank empty
    case GPIO_PIN_11:
      // nc switch to VCC closed -> GPIO_Pin == HIGH
      if (GPIO_ReadInputDataBit(irqs[SWITCH_SEWAGE_MIN].gpio_port, GPIO_Pin))
        pwms[PWM_SEWAGE_PUMP].duty_percent = 0;
        pwms[PWM_SEWAGE_PUMP].run_for_ms = 0;
        global_state.sewage_tank_empty = 1;
      break;
  }
}

void dehumidifier_level_irq_handler(uint16_t GPIO_Pin)
{

  switch (GPIO_Pin) {
    // tank full
    case GPIO_PIN_7:
      // nc switch to VCC opened -> GPIO_Pin == LOW
      if (!GPIO_ReadInputDataBit(irqs[SWITCH_DEHUMI_MAX].gpio_port, GPIO_Pin))
        pwms[PWM_DEHUMI_PUMP].duty_percent = 100;
      break;

    // tank empty
    case GPIO_PIN_8:
      // nc switch to VCC closed -> GPIO_Pin == HIGH
      if (GPIO_ReadInputDataBit(irqs[SWITCH_DEHUMI_MIN].gpio_port, GPIO_Pin))
        pwms[PWM_DEHUMI_PUMP].duty_percent = 0;
      break;
  }
}


void TM_EXTI_Handler(uint16_t GPIO_Pin){
  uint8_t pin;
  switch(GPIO_Pin){
    case GPIO_PIN_0: // Blue Button (Discovery Board)
      pin = 0;
      break;

    case GPIO_PIN_1: // Reservoir Min
      pin = 1;
      reservoir_level_irq_handler(GPIO_Pin);
      break;

    case GPIO_PIN_2: // Reservoir Max
      pin = 2;
      reservoir_level_irq_handler(GPIO_Pin);
      break;

    case GPIO_PIN_3: // Reservoir Alarm
      pin = 3;
      reservoir_level_irq_handler(GPIO_Pin);
      break;

    case GPIO_PIN_4: // 
      pin = 4;
      break;

    case GPIO_PIN_5: // 
      pin = 5;
      break;

    case GPIO_PIN_6: // 
      pin = 6;
      break;

    case GPIO_PIN_7: // Dehumidifier tank full
      pin = 7;
      dehumidifier_level_irq_handler(GPIO_Pin);
      break;

    case GPIO_PIN_8: // Dehumidifier tank empty
      pin = 8;
      dehumidifier_level_irq_handler(GPIO_Pin);
      break;

    // no pin 9

    case GPIO_PIN_10: // Sewage tank full
      pin = 10;
      sewage_level_irq_handler(GPIO_Pin);
      break;

    case GPIO_PIN_11: // Sewage tank empty
      pin = 11;
      sewage_level_irq_handler(GPIO_Pin);
      break;

    case GPIO_PIN_12: //
      pin = 12;
      break;

    case GPIO_PIN_13: //
      pin = 13;
      break;

    // no pin 14

    case GPIO_PIN_15: //
      pin = 15;
      break;

  default:
    break;
  }
  char buf[128];
  sprintf(buf, "{\"event\": \"%s turned %s\", \"time\": \"%s\"}\r\n", irq_input_names[pin], GPIO_ReadInputDataBit(irqs[pin].gpio_port, GPIO_Pin) ? "on" : "off", global_state.datestring);
  TM_USART_Puts(USART2, buf);
  print_irqs();
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

void pwm_init(){

  pwms[PWM_FILL_PUMP].tim_data = &TIM1_Data;
  pwms[PWM_FILL_PUMP].pwm_channel = TM_PWM_Channel_1;
  pwms[PWM_FILL_PUMP].duty_percent = 0;
  pwms[PWM_FILL_PUMP].run_for_ms = 0;
  pwms[PWM_DRAIN_PUMP].tim_data = &TIM1_Data;
  pwms[PWM_DRAIN_PUMP].pwm_channel = TM_PWM_Channel_4;
  pwms[PWM_DRAIN_PUMP].duty_percent = 0;
  pwms[PWM_DRAIN_PUMP].run_for_ms = 0;
  pwms[PWM_COOLANT_PUMP].tim_data = &TIM3_Data;
  pwms[PWM_COOLANT_PUMP].pwm_channel = TM_PWM_Channel_1;
  pwms[PWM_COOLANT_PUMP].duty_percent = 0;
  pwms[PWM_COOLANT_PUMP].run_for_ms = 0xffffffff;
  pwms[PWM_SEWAGE_PUMP].tim_data = &TIM3_Data;
  pwms[PWM_SEWAGE_PUMP].pwm_channel = TM_PWM_Channel_2;
  pwms[PWM_SEWAGE_PUMP].duty_percent = 0;
  pwms[PWM_SEWAGE_PUMP].run_for_ms = 0;
  pwms[PWM_DEHUMI_PUMP].tim_data = &TIM3_Data;
  pwms[PWM_DEHUMI_PUMP].pwm_channel = TM_PWM_Channel_3;
  pwms[PWM_DEHUMI_PUMP].duty_percent = 0;
  pwms[PWM_DEHUMI_PUMP].run_for_ms = 0;
  pwms[PWM_PHDOWN_PUMP].tim_data = &TIM3_Data;
  pwms[PWM_PHDOWN_PUMP].pwm_channel = TM_PWM_Channel_4;
  pwms[PWM_PHDOWN_PUMP].duty_percent = 0;
  pwms[PWM_PHDOWN_PUMP].run_for_ms = 0;
  pwms[PWM_NUTRIENT1_PUMP].tim_data = &TIM4_Data;
  pwms[PWM_NUTRIENT1_PUMP].pwm_channel = TM_PWM_Channel_3;
  pwms[PWM_NUTRIENT1_PUMP].duty_percent = 0;
  pwms[PWM_NUTRIENT1_PUMP].run_for_ms = 0;
  pwms[PWM_NUTRIENT2_PUMP].tim_data = &TIM9_Data;
  pwms[PWM_NUTRIENT2_PUMP].pwm_channel = TM_PWM_Channel_1;
  pwms[PWM_NUTRIENT2_PUMP].duty_percent = 0;
  pwms[PWM_NUTRIENT2_PUMP].run_for_ms = 0;
  pwms[PWM_NUTRIENT3_PUMP].tim_data = &TIM9_Data;
  pwms[PWM_NUTRIENT3_PUMP].pwm_channel = TM_PWM_Channel_2;
  pwms[PWM_NUTRIENT3_PUMP].duty_percent = 0;
  pwms[PWM_NUTRIENT3_PUMP].run_for_ms = 0;

  // 14kHz =~ 71us cycle time
  TM_PWM_InitTimer(TIM1, &TIM1_Data, 14000);
  TM_PWM_InitTimer(TIM3, &TIM3_Data, 14000);
  
  TM_PWM_InitChannel(&TIM1_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_2); // PE9
  TM_PWM_InitChannel(&TIM1_Data, TM_PWM_Channel_4, TM_PWM_PinsPack_2); // PE14
  TM_PWM_InitChannel(&TIM3_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_2); // PB4
  TM_PWM_InitChannel(&TIM3_Data, TM_PWM_Channel_2, TM_PWM_PinsPack_2); // PB5
  TM_PWM_InitChannel(&TIM3_Data, TM_PWM_Channel_3, TM_PWM_PinsPack_2); // PC8
  TM_PWM_InitChannel(&TIM3_Data, TM_PWM_Channel_4, TM_PWM_PinsPack_2); // PC9
  TM_PWM_InitChannel(&TIM4_Data, TM_PWM_Channel_3, TM_PWM_PinsPack_1); // PB8
  TM_PWM_InitChannel(&TIM9_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_2); // PE5
  TM_PWM_InitChannel(&TIM9_Data, TM_PWM_Channel_2, TM_PWM_PinsPack_2); // PE6
    
  TM_PWM_SetChannel(pwms[PWM_FILL_PUMP].tim_data, pwms[PWM_FILL_PUMP].pwm_channel, 0);
  TM_PWM_SetChannel(pwms[PWM_DRAIN_PUMP].tim_data, pwms[PWM_DRAIN_PUMP].pwm_channel, 0);
  TM_PWM_SetChannel(pwms[PWM_COOLANT_PUMP].tim_data, pwms[PWM_COOLANT_PUMP].pwm_channel, 0);
  TM_PWM_SetChannel(pwms[PWM_SEWAGE_PUMP].tim_data, pwms[PWM_SEWAGE_PUMP].pwm_channel, 0);
  TM_PWM_SetChannel(pwms[PWM_DEHUMI_PUMP].tim_data, pwms[PWM_DEHUMI_PUMP].pwm_channel, 0);
  TM_PWM_SetChannel(pwms[PWM_PHDOWN_PUMP].tim_data, pwms[PWM_PHDOWN_PUMP].pwm_channel, 0);
  TM_PWM_SetChannel(pwms[PWM_NUTRIENT1_PUMP].tim_data, pwms[PWM_PHDOWN_PUMP].pwm_channel, 0);
  TM_PWM_SetChannel(pwms[PWM_NUTRIENT2_PUMP].tim_data, pwms[PWM_PHDOWN_PUMP].pwm_channel, 0);
  TM_PWM_SetChannel(pwms[PWM_NUTRIENT3_PUMP].tim_data, pwms[PWM_PHDOWN_PUMP].pwm_channel, 0);

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
