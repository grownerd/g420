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

#define NUM_PH_READINGS 1024
#define NUM_PH_AVG_VALS 16 // must be a multipoe of 4!

int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);


void adc_init(void)
{
  TM_ADC_Init(ADC1, TM_ADC_Channel_8); // PB0
  TM_ADC_Init(ADC1, TM_ADC_Channel_9); // PB1
}


void read_ph(float* ph_val)
{
  float adc_v = 0;
  uint32_t adc_raw = 0;
  uint32_t i = 0;
  uint32_t j = 0;
  uint32_t tmp_val = 0;
  static uint32_t ph_arr[NUM_PH_AVG_VALS] = {0};
  static uint32_t arr_idx = 0;

  for (i=0; i<NUM_PH_READINGS; i++)
    adc_raw += TM_ADC_Read(ADC1, TM_ADC_Channel_9);

  adc_v = adc_raw * misc_settings.vcc_v / 4096.0f / NUM_PH_READINGS;

  if (arr_idx < NUM_PH_AVG_VALS) {
    ph_arr[arr_idx++] = adc_v;
  } else {
    arr_idx = 0;

    // bubble sort the array
    for (i = 0; i < num; i++) {
      for (j = 0; j < (num - i - 1); j++) {
        if (ph_arr[j] > ph_arr[j + 1]) {
          tmp_val = ph_arr[j];
          ph_arr[j] = ph_arr[j + 1];
          ph_arr[j + 1] = tmp_val;
        }
      }
    }

    for (i=NUM_PH_AVG_VALS/4; i<(NUM_PH_AVG_VALS/4*3); i++)
      adv_v += ph_arr[i];

    adv_v /= (NUM_PH_AVG_VALS/2);
    *ph_val = 7 + ((misc_settings.ph7_v - adc_v) / misc_settings.ph_step);
  }
}


void read_ec(float* ec_val)
{
  uint16_t adc_raw = 0;
  float v_drop = 0;
  float rc = 0;
  float ec = 0;
  static uint32_t next_time = 0;

  if (!next_time) next_time = TM_Time;

  if (TM_Time >= next_time){
    GPIO_WriteBit(GPIOC, GPIO_Pin_1, 1);
    Delayms(10);
    TM_GPIO_SetPinHigh(EC_GPIO_PORT, EC_GPIO_PIN);
    adc_raw = TM_ADC_Read(ADC1, TM_ADC_Channel_8);
    TM_GPIO_SetPinLow(EC_GPIO_PORT, EC_GPIO_PIN);
    GPIO_WriteBit(GPIOC, GPIO_Pin_1, 0);
    
    v_drop = (V_IN * adc_raw) / 4096.0;
    rc = (v_drop * misc_settings.ec_r1_ohms) / (V_IN - v_drop);
    rc = rc - misc_settings.ec_ra_ohms;
    ec = 1000 / (rc * misc_settings.ec_k);
    *ec_val = (float)ec / (1 + misc_settings.ec_temp_coef * (ds18b20_sensors[TEMP_RES].value - 25.0));

    next_time = TM_Time + (misc_settings.ec_read_interval_s * 1000);
  }
}


int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

