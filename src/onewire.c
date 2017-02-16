#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_gpio.h>

#include "defines.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_onewire.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_ds18b20.h"
#include "tm_stm32f4_disco.h"
#include <stdio.h>
 
#include "onewire.h"


char buf[40];
uint8_t devices, i, j, count, alarm_count;
uint8_t device[DS18B20_NUM_DEVICES][8];
uint8_t alarm_device[DS18B20_NUM_DEVICES][8];


/* OneWire working struct */
TM_OneWire_t OneWire1;

float temps[DS18B20_NUM_DEVICES];


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
        sprintf(buf, "Devices found on 1-wire: %d\r\n", count);
        TM_USART_Puts(USART2, buf);
        /* Display 64bit rom code for each device */
        for (j = 0; j < count; j++) {
            for (i = 0; i < 8; i++) {
                sprintf(buf, "0x%02X ", device[j][i]);
                TM_USART_Puts(USART2, buf);
            }
            TM_USART_Puts(USART2, "\r\n");
        }
    } else {
        TM_USART_Puts(USART2, "No devices on OneWire.\r\n");
    }
    
    /* Go through all connected devices and set resolution to 12bits */
    for (i = 0; i < count; i++) {
        /* Set resolution to 12bits */
        TM_DS18B20_SetResolution(&OneWire1, device[i], TM_DS18B20_Resolution_12bits);
    }
    
    /* Set high temperature alarm on device number 0, 25 degrees celcius */
    TM_DS18B20_SetAlarmHighTemperature(&OneWire1, device[0], 25);
    
    /* Disable alarm temperatures on device number 1 */
    TM_DS18B20_DisableAlarmTemperature(&OneWire1, device[1]);

}
    

void ds18b20_read_temp(void) {

    /* Start temperature conversion on all devices on one bus */
    TM_DS18B20_StartAll(&OneWire1);
    
    /* Wait until all are done on one onewire port */
    while (!TM_DS18B20_AllDone(&OneWire1));
    
    /* Read temperature from each device separatelly */
    for (i = 0; i < count; i++) {
        /* Read temperature from ROM address and store it to temps variable */
        if (TM_DS18B20_Read(&OneWire1, device[i], &temps[i])) {
            /* Print temperature */
            sprintf(buf, "Temp %d: %3.5f; \r\n", i, temps[i]);
            TM_USART_Puts(USART2, buf);
        } else {
            /* Reading error */
            TM_USART_Puts(USART2, "Reading error;\r\n");
        }
    }
    
    /* Reset alarm count */
    alarm_count = 0;
    
    /* Check if any device has alarm flag set */
    while (TM_DS18B20_AlarmSearch(&OneWire1)) {
        /* Store ROM of device which has alarm flag set */
        TM_OneWire_GetFullROM(&OneWire1, alarm_device[alarm_count]);
        /* Increase count */
        alarm_count++;
    }
    
    /* Format string and send over USART for debug */
    sprintf(buf, "Devices with alarm: %d\r\n", alarm_count);
    TM_USART_Puts(USART2, buf);
    
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
    
    /* Print separator */
    TM_USART_Puts(USART2, "----------\r\n");
    
}
