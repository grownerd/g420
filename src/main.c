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
 
#include "i2c.h"
#include "onewire.h"
#include "bme280.h"
#include "rtc.h"
#include "command_parser.h"


s32 bme280_data_readout_template(void);


int main(void) {
    
  SystemInit();
  
  TM_USART_Init(USART2, TM_USART_PinsPack_1, 115200);
  TM_USART_Puts(USART2, "System Startup.\r\n");

  struct bme280_t bme280;
  init_I2C1();

  TM_DELAY_Init();
  
  TM_DISCO_LedInit();
  TM_DISCO_LedOn(LED_ORANGE);
  
  TM_RTC_Init(TM_RTC_ClockSource_Internal);

  if (TM_WATCHDOG_Init(TM_WATCHDOG_Timeout_4s)) {
    //System was reset by watchdog
    TM_DISCO_LedOn(LED_RED);
  } else {
    //System was not reset by watchdog
    TM_DISCO_LedOn(LED_GREEN);
  }
    
  char buf[50];
  TM_RTC_Interrupts(TM_RTC_Int_1s);


  onewire_init();
    
  while (1) {

    
    bme280_data_readout_template();

    ds18b20_read_temp();
    //Delayms(1000);

#if 1
    command_parser();
#else
    // 01.02.15;00:33:12\n
    if (TM_USART_Gets(USART2, buf, sizeof(buf))) {
      /* Try to set date and time */
      if (TM_RTC_SetDateTimeString(buf) == TM_RTC_Result_Ok) {
        /* Date and time written OK */
        
        /* Send OK back */
        TM_USART_Puts(USART2, "Written OK\n");
      } else {
        /* Send error sring */
        TM_USART_Puts(USART2, "Error\n");
      }
    }
#endif


    //Reset watchdog
    TM_WATCHDOG_Reset();
  }
}
