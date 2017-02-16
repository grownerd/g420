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
#include "rtc.h"

char buf[50];
TM_RTC_Time_t datatime;
TM_RTC_Time_t Time;

void set_time(char * timestring) {
  if (TM_RTC_SetDateTimeString(timestring) == TM_RTC_Result_Ok) {
    TM_USART_Puts(USART2, "Time set OK\r\n");
  } else {
    TM_USART_Puts(USART2, "Time set Error\r\n");
  }

}


void set_alarm(TM_RTC_Alarm_t alarm, uint8_t hours, uint8_t minutes) {
   
  TM_RTC_AlarmTime_t AlarmTime;

    AlarmTime.hours = hours;
    AlarmTime.minutes = minutes;
    AlarmTime.seconds = 0;
    AlarmTime.day = 1;
    AlarmTime.alarmtype = TM_RTC_AlarmType_DayInMonth;
    
    TM_RTC_SetAlarm(alarm, &AlarmTime, TM_RTC_Format_BIN);

}


void print_time(void) {
    /* Get time */
    TM_RTC_GetDateTime(&Time, TM_RTC_Format_BIN);
    
    /* Format time */
    sprintf(buf, "%02d.%02d.%04d %02d:%02d:%02d  Unix: %u\r\n",
                Time.date,
                Time.month,
                Time.year + 2000,
                Time.hours,
                Time.minutes,
                Time.seconds,
                Time.unix
    );
    
    /* Send to USART */
    TM_USART_Puts(USART2, buf);
}
    

void TM_RTC_RequestHandler() {
    TM_DISCO_LedToggle(LED_BLUE | LED_ORANGE);
}
 
/* Custom request handler function */
/* Called on alarm A interrupt */
void TM_RTC_AlarmAHandler(void) {
    /* Show user to USART */
    TM_USART_Puts(USART2, "Alarm A triggered\r\n");
    
    /* Disable Alarm so it will not trigger next week at the same time */
    //TM_RTC_DisableAlarm(TM_RTC_Alarm_A);
}
 
/* Custom request handler function */
/* Called on alarm B interrupt */
void TM_RTC_AlarmBHandler(void) {
    /* Show user to USART */
    TM_USART_Puts(USART2, "Alarm B triggered\r\n");
    
    //TM_RTC_DisableAlarm(TM_RTC_Alarm_A);
}


