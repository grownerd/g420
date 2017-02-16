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
 
#include "command_parser.h"
#include "rtc.h"

#define MAX_STR_LEN 64

void host_cmd_get(char * item);
void host_cmd_set(char * item, char * val);
void set_light_time(char * val);

void command_parser(void){

  char command[50];
  char cmd[3][20];
  uint8_t cmd_counter, j = 0;

  if (TM_USART_Gets(USART2, command, sizeof(command))) {
    for (int i=0; i < strlen(command); i++){
      char t = command[i];
      if (t == ' ') {
        cmd[cmd_counter][j] = '\0';
        cmd[cmd_counter++];
        j = 0;
      } else {
        cmd[cmd_counter][j++] = t;
      }
    }

    if        (strncmp(cmd[0], "get", 3) == 0){
      host_cmd_get(cmd[1]);
    }else 	if(strncmp(cmd[0], "set", 3) == 0){
      host_cmd_set(cmd[1], cmd[2]);
    }
  }
}

void host_cmd_get(char * item) {
  if (strncmp(item, "rtc", 3) == 0)
  {
    print_time();
  }
  else if (strncmp(item, "light", 5) == 0)
  {
#if 0
    char buf[MAX_STR_LEN];

    main_light.state = GPIO_ReadInputDataBit(main_light.gpio_port, main_light.gpio_pin);
    sprintf((char *) &buf[0], "Main Light:\r\n\tOn Time: %02d:%02d\r\n\tOff Time: %02d:%02d\r\n\tCurrent state: %s\r\n",
      main_light.on_hour,
      main_light.on_minutes,
      main_light.off_hour,
      main_light.off_minutes,
      main_light.state ? "On" : "Off"
    );
    TM_USART_Puts(USART2, (char *) &buf[0]);
#endif
  }
}

void host_cmd_set(char * item, char * val) {
  if (strncmp(item, "rtc", 3) == 0)
  {
    set_time(val);
    print_time();
  }
  else if (strncmp(item, "light", 3) == 0)
  {
    set_light_time(val);
    host_cmd_get("light");
  }
}

// should be hh:mm-hh:mm, but all combinations of : and - are allowed
void set_light_time(char * val) {
  uint8_t on_hour = 0;
  uint8_t on_minutes = 0;
  uint8_t off_hour = 0;
  uint8_t off_minutes = 0;

  char part[4][3];

  uint8_t part_counter = 0;
  uint8_t j = 0;

  for (int i=0; i < strlen(val); i++){
    char t = val[i];
    if ((t == '-') || (t == ':')) {
      part[part_counter][j] = '\0';
      part[part_counter++];
      j = 0;
    } else {
      part[part_counter][j++] = t;
    }
  }

  on_hour = atoi(part[0]);
  on_minutes = atoi(part[1]);
  off_hour = atoi(part[2]);
  off_minutes = atoi(part[3]);

  set_alarm(TM_RTC_Alarm_A, on_hour, on_minutes);
  set_alarm(TM_RTC_Alarm_B, off_hour, off_minutes);

}

