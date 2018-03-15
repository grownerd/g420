#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stm32f4xx.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_gpio.h>

#include "defines.h"
#include "tm_stm32f4_rtc.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_usart.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_watchdog.h"
 
#include "command_parser.h"
#include "rtc.h"
#include "gpio.h"
#include "main.h"
#include "flash.h"


char buf[128];


void TM_USART2_ReceiveHandler(uint8_t c){
  static uint8_t uart2_str[MAX_STR_LEN] = {};
  static uint8_t uart2_idx = 0;

  uart2_str[uart2_idx++] = c;

  if ((c == '\n') || (c == '\r')) {
    uart2_str[uart2_idx++] = '\0';
    uart2_idx = 0;
    command_parser(uart2_str);
    memset(uart2_str, 0, 10);
  }
}


void command_parser(char * command){

  char cmd[3][64];
  uint8_t cmd_counter = 0, j = 0;

  for (int i=0; i < strlen(command); i++){
    char t = command[i];
    if (t == ' ' && cmd_counter < 2) {
      cmd[cmd_counter][j] = '\0';
      cmd[cmd_counter++];
      j = 0;
    } else {
      cmd[cmd_counter][j++] = t;
    }
  }
  cmd[cmd_counter][j] = '\0';

  if (strncmp(cmd[0], "load", 4) == 0){
    read_flash();
  } else if (strncmp(cmd[0], "save", 4) == 0){
    save_data_to_flash();
  } else if (strncmp(cmd[0], "get", 3) == 0){
    host_cmd_get(cmd[1]);
  }else	if(strncmp(cmd[0], "set", 3) == 0){
    host_cmd_set(cmd[1], cmd[2]);
  }else	if(strncmp(cmd[0], "del", 3) == 0){
    host_cmd_del(cmd[1], cmd[2]);
  }
}

void host_cmd_get(char * item) {
  if (strncmp(item, "timers", 6) == 0) {
    print_timers();
  } else if (strncmp(item, "rtc", 3) == 0) {
    print_rtc();
  }
}

void host_cmd_set(char * item, char * val) {
  if (strncmp(item, "timer", 5) == 0) {
    set_timer(val);
    print_timers();
  } else if (strncmp(item, "rtc", 3) == 0) {
    set_rtc(val);
    print_rtc();
  }
}

void host_cmd_del(char * item, char * val) {
  if (strncmp(item, "timer", 5) == 0) {
    del_timer(val);
    print_timers();
  }
}

void del_timer(char * index){
  uint32_t i = 0;
  i = atoi(index);
  timer_events[i].on_time = 0;
  timer_events[i].off_time = 0;
}

void set_rtc(char * timestring) {
  if (TM_RTC_SetDateTimeString(timestring) == TM_RTC_Result_Ok) {
    TM_USART_Puts(USART2, "{\"event\": \"Time set OK\"}\r\n");
  } else {
    TM_USART_Puts(USART2, "{\"error\": \"Time set Error\"}\r\n");
  }
}


void set_timer(char * on_to_off){
  char parts[6][3];
  uint8_t part_counter = 0, i = 0, j = 0;
  uint32_t on_time = 0, off_time = 0, timer_index = 0;

  for (i=0; i < strlen(on_to_off); i++){
    char t = on_to_off[i];
    if (((t == ' ' ) || (t == '-' ) || (t == ':' )) && (part_counter < 6)) {
      parts[part_counter][j] = '\0';
      parts[part_counter++];
      j = 0;
    } else {
      parts[part_counter][j++] = t;
    }
  }
  parts[part_counter][j] = '\0';

  on_time = (atoi(parts[0]) * 3600) + (atoi(parts[1]) * 60) + atoi(parts[2]);
  off_time = (atoi(parts[3]) * 3600) + (atoi(parts[4]) * 60) + atoi(parts[5]);

  for (i = 0; i < MAX_TIMER_EVENTS; i++) {
    if (slot_is_empty(i)){
      timer_events[i].on_time = on_time;
      timer_events[i].off_time = off_time;
      break;
    }
  }
  
}

void print_rtc(void) {

  TM_RTC_Time_t Time;
  TM_RTC_GetDateTime(&Time, TM_RTC_Format_BIN);
    
  snprintf(buf, MAX_STR_LEN, "{\"name\":\"datetime\",\"content\":[{\"date\":\"%02d.%02d.%04d\"}, {\"time\":\"%02d:%02d:%02d.%06d\"}, {\"unix\":%u}]}\r\n",
              Time.date,
              Time.month,
              Time.year + 2000,
              Time.hours,
              Time.minutes,
              Time.seconds,
              Time.subseconds,
              Time.unix
  );
  
  TM_USART_Puts(USART2, buf);
}
    

void print_timers(void) {
  uint32_t i = 0;
  
  for (i = 0; i < MAX_TIMER_EVENTS; i++) {

    // only act on non-empty slots
    if (!slot_is_empty(i)) {
      snprintf(buf, MAX_STR_LEN, "{\"timer index\": \"%03d\", \"on\": \"%02d:%02d:%02d\",  \"off\": \"%02d:%02d:%02d\"}\r\n",
        i,
        timer_events[i].on_time / 3600,
        timer_events[i].on_time % 3600 / 60,
        timer_events[i].on_time % 60,
        timer_events[i].off_time / 3600,
        timer_events[i].off_time % 3600 / 60,
        timer_events[i].off_time % 60
      );
    TM_USART_Puts(USART2, buf);
    }
  }
}
