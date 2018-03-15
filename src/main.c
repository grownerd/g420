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


// Global Vars

char buf[MAX_STR_LEN];
char datestring[DATESTR_LEN] = "\"date\": \"00.00.0000\", \"time\": \"00:00:00\"";

gpio_output_struct_t gpio_outputs[NUM_GPIO_OUTPUTS];


// Helper functions

void TM_RTC_RequestHandler() {

  TM_DISCO_LedToggle(LED_BLUE | LED_ORANGE);

  TM_RTC_Time_t Time;
  TM_RTC_GetDateTime(&Time, TM_RTC_Format_BIN);

  snprintf(datestring, DATESTR_LEN, "\"date\": \"%02d.%02d.%04d\", \"time\": \"%02d:%02d:%02d\"",
    Time.date,
    Time.month,
    Time.year + 2000,
    Time.hours,
    Time.minutes,
    Time.seconds
  );
}
 

uint8_t slot_is_empty(uint32_t index) {
  if (timer_events[index].on_time == timer_events[index].off_time)
    return 1;
  else
    return 0;

}

void set_defaults(void) {
  init_timer_events();
}


void init_timer_events(void){
  uint32_t i = 0;

  for (i = 0; i < MAX_TIMER_EVENTS; i++) {
    timer_events[i].on_time = 0;
    timer_events[i].off_time = 0;
  }
}


void gpio_scheduler(void){
  uint32_t i = 0;
  uint8_t desired_state = 0;
  uint32_t time = 0;

  static uint32_t ts = 0;

  TM_RTC_Time_t Time;
  TM_RTC_GetDateTime(&Time, TM_RTC_Format_BIN);

  time = (Time.hours * 3600) + (Time.minutes * 60) + Time.seconds;

  // Loop through all possible timer events
  for (i = 0; i < MAX_TIMER_EVENTS; i++) {

    // only act on non-empty slots
    if (!slot_is_empty(i)) {

      // Event starts and ends before midnight
      if (timer_events[i].on_time < timer_events[i].off_time) {
        if (time >= timer_events[i].on_time) {
          desired_state = 1;
        } 

        if (time >= timer_events[i].off_time) {
          desired_state = 0;
        }

      // Event starts before, but ends after midnight
      } else {
        if (time >= timer_events[i].off_time) {
          desired_state = 0;
        }

        if (time >= timer_events[i].on_time) {
          desired_state = 1;
        } 

      }
    }
  }

  if (TM_DISCO_ButtonPressed())
    desired_state = 1;

  
  if (GPIO_ReadInputDataBit(gpio_outputs[GPIO_OUTPUT_FEED_PUMP].gpio_port, gpio_outputs[GPIO_OUTPUT_FEED_PUMP].gpio_pin) != desired_state){

    GPIO_WriteBit(gpio_outputs[GPIO_OUTPUT_FEED_PUMP].gpio_port, gpio_outputs[GPIO_OUTPUT_FEED_PUMP].gpio_pin, desired_state);

    snprintf(buf, MAX_STR_LEN, "{\"event\": \"Output changed\", \"value\": \"%d\", %s}\r\n", desired_state, datestring);
    TM_USART_Puts(USART2, buf);

    if (desired_state) {
      ts = Time.unix;
    } else {
      snprintf(buf, MAX_STR_LEN, "{\"event\": \"Seconds elapsed\", \"value\": \"%d\", %s}\r\n", Time.unix - ts, datestring);
      TM_USART_Puts(USART2, buf);
      
    }
  }
}

void main(void) {

  gpio_init();

  TM_DISCO_ButtonInit();
  TM_DISCO_LedInit();
  TM_DISCO_LedOn(LED_BLUE);

  TM_RTC_Init(TM_RTC_ClockSource_Internal); // This is actually the HSE, not the LSI!

  TM_USART_Init(USART2, TM_USART_PinsPack_1, 115200);
  TM_USART_Puts(USART2, "{\"event\": \"System Startup\"}\r\n");

  read_flash();

  while (1) {
    gpio_scheduler();

  }
}
