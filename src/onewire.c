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
 
#include "gpio.h"
#include "main.h"
#include "onewire.h"


char buf[MAX_STR_LEN];
uint8_t devices, i, j, count, alarm_count;
uint8_t device[DS18B20_NUM_DEVICES][8];
uint8_t alarm_device[DS18B20_NUM_DEVICES][8];


/* OneWire working struct */
TM_OneWire_t OneWire1;



void onewire_init(void) {
    
  /* Initialize OneWire on pin PD0 */
  TM_OneWire_Init(&OneWire1, GPIOD, GPIO_Pin_0);
  

  /* Checks for any device on 1-wire */
  count = 0;
  devices = TM_OneWire_First(&OneWire1);
  while (devices) {
    /* Increase counter */
    count++;
    
    /* Get full ROM value, 8 bytes, give location of first byte where to save */
    TM_OneWire_GetFullROM(&OneWire1, device[count - 1]);
    
    /* Get next device */
    devices = TM_OneWire_Next(&OneWire1);
  }
  
  /* If any devices on 1wire */
  if (count > 0) {
    TM_DS18B20_StartAll(&OneWire1);
    while (!TM_DS18B20_AllDone(&OneWire1));
    sprintf(buf, "{\"event\": \"Devices found on 1-wire\", \"number\": %d, \"content\":[\r\n", count);
    TM_USART_Puts(USART2, buf);
    /* Display 64bit rom code for each device */
    for (j = 0; j < count; j++) {
      TM_USART_Puts(USART2, "\t{\"address\": \"");
      for (i = 0; i < 8; i++) {
        sprintf(buf, "0x%02X ", device[j][i]);
        TM_USART_Puts(USART2, buf);
      }
      TM_USART_Puts(USART2, "\"}");
      if (j < (count - 1))
        TM_USART_Puts(USART2, ",\r\n");
      else
        TM_USART_Puts(USART2, "]\r\n");
        
    }
    TM_USART_Puts(USART2, "}\r\n");
  } else {
    TM_USART_Puts(USART2, "{\"event\": \"No devices on OneWire.\"}\r\n");
  }
  
  /* Go through all connected devices and set resolution to 12bits */
  for (i = 0; i < count; i++) {
      /* Set resolution to 12bits */
      TM_DS18B20_SetResolution(&OneWire1, device[i], TM_DS18B20_Resolution_12bits);
  }
  
  /* Set high temperature alarm on device number 0, 25 degrees celcius */
  //TM_DS18B20_SetAlarmHighTemperature(&OneWire1, device[0], 25);
  
  /* Disable alarm temperatures on device number 1 */
  TM_DS18B20_DisableAlarmTemperature(&OneWire1, device[0]);
  TM_DS18B20_DisableAlarmTemperature(&OneWire1, device[1]);

}
    

void ds18b20_read_temp() {

  static int conversion_running = 0;

  if (!conversion_running){
    TM_DS18B20_StartAll(&OneWire1);
    conversion_running = 1;
  }else{
    
    if (TM_DS18B20_AllDone(&OneWire1)){
      
      for (i = 0; i < count; i++) {

        if (!TM_DS18B20_Read(&OneWire1, device[i], &ds18b20_sensors[i].value)) {
          ds18b20_sensors[i].error_count++;
        }
      }
      
      conversion_running = 0;
    }
  }
#if 0
    
      /* Reset alarm count */
      alarm_count = 0;
    /* Check if any device has alarm flag set */
    while (TM_DS18B20_AlarmSearch(&OneWire1)) {
        /* Store ROM of device which has alarm flag set */
        TM_OneWire_GetFullROM(&OneWire1, alarm_device[alarm_count]);
        /* Increase count */
        alarm_count++;
    }
    
    if (alarm_count > 0) {
      sprintf(buf, "Devices with alarm: %d\r\n", alarm_count);
      TM_USART_Puts(USART2, buf);
    }
    
    /* Any device has alarm flag set? */
    if (alarm_count > 0) {
        /* Show rom of this devices */
        for (j = 0; j < alarm_count; j++) {
            TM_USART_Puts(USART2, "Device with alarm: ");
            for (i = 0; i < 8; i++) {
                sprintf(buf, "0x%02X ", alarm_device[j][i]);
                TM_USART_Puts(USART2, buf);
            }
            TM_USART_Puts(USART2, "\r\n    ");
        }
        TM_USART_Puts(USART2, "ALARM devices recognized!\r\n\r");
    }
    
#endif
    /* Print separator */
    //TM_USART_Puts(USART2, "----------\r\n");
    
}
