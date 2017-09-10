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
 
#include "command_parser.h"
#include "rtc.h"
#include "gpio.h"
#include "main.h"
#include "flash.h"


void host_cmd_get(char * item);
void host_cmd_set(char * item, char * val);
void host_cmd_reset(char * item, char * val);
void set_light_time(char * val);
void set_minmax(char * val, uint8_t minmax);
void set_pwm(char * val);
void set_relay(char * val);
void set_ph(char * val);
void set_nutrients(char * val);
void set_misc(char * val);

void command_parser(void){

  char command[256];
  char cmd[3][64];
  uint8_t cmd_counter = 0, j = 0;

  for (int i=0; i< sizeof(command); i++)
    command[i] = '\0';

  if (TM_USART_Gets(USART2, command, sizeof(command))) {
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

    if (strncmp(cmd[0], "release", 7) == 0){
      emergency_stop(1);
    } else if (strncmp(cmd[0], "cycle", 5) == 0){
      global_state.drain_cycle_active = 1;
    } else if (strncmp(cmd[0], "drain", 5) == 0){
      global_state.reservoir_state = MANUAL_DRAIN;
    } else if (strncmp(cmd[0], "reset", 5) == 0){
      host_cmd_reset(cmd[1], cmd[2]);
    } else if (strncmp(cmd[0], "idle", 4) == 0){
      global_state.reservoir_state = NORMAL_IDLE;
    } else if (strncmp(cmd[0], "fill", 4) == 0){
      global_state.reservoir_state = MANUAL_FILL;
    } else if (strncmp(cmd[0], "save", 4) == 0){
      save_data_to_flash();
    } else if (strncmp(cmd[0], "load", 4) == 0){
      read_flash();
    } else if (strncmp(cmd[0], "stop", 4) == 0){
      emergency_stop(0);
    } else if (strncmp(cmd[0], "get", 3) == 0){
      host_cmd_get(cmd[1]);
    }else 	if(strncmp(cmd[0], "set", 3) == 0){
      host_cmd_set(cmd[1], cmd[2]);
    }
  }
}

void host_cmd_get(char * item) {
  if (strncmp(item, "nutrients", 9) == 0)
  {
    print_nutrients();
  }
  else if (strncmp(item, "capsense", 8) == 0)
  {
    print_capsense();
  }
  else if (strncmp(item, "settings", 8) == 0)
  {
    print_settings();
  }
  else if (strncmp(item, "coolant", 7) == 0)
  {
    print_coolant();
  }
  else if (strncmp(item, "exhaust", 7) == 0)
  {
    print_exhaust();
  }
  else if (strncmp(item, "relays", 6) == 0)
  {
    print_relays();
  }
  else if (strncmp(item, "errors", 5) == 0)
  {
    print_errors();
  }
  else if (strncmp(item, "light", 5) == 0)
  {
    print_light();
  }
  else if (strncmp(item, "state", 5) == 0)
  {
    print_state();
  }
  else if (strncmp(item, "pwmin", 5) == 0)
  {
    print_pwmin(PWMIN_RES_DRAIN, PWMIN1_Data.Frequency);
  }
  else if (strncmp(item, "irqs", 4) == 0)
  {
    print_irqs();
  }
  else if (strncmp(item, "env", 3) == 0)
  {
    print_env();
  }
  else if (strncmp(item, "pwm", 3) == 0)
  {
    print_pwm();
  }
  else if (strncmp(item, "rtc", 3) == 0)
  {
    print_time();
  }
  else if (strncmp(item, "all", 3) == 0)
  {
    print_time();
    print_env();
    print_light();
    print_exhaust();
    print_coolant();
    print_errors();
    print_pwm();
    print_relays();
    print_capsense();
    print_pwmin(PWMIN_RES_DRAIN, PWMIN1_Data.Frequency);
    print_irqs();
    print_settings();
    print_state();
  }
  else if (strncmp(item, "ph", 2) == 0)
  {
    print_ph();
  }
  else if (strncmp(item, "ec", 2) == 0)
  {
    print_ec();
  }
}

void host_cmd_reset(char * item, char * val) {
  if (strncmp(item, "errors", 6) == 0)
  {
    reset_errors();
  }
  else if (strncmp(item, "i2c", 3) == 0)
  {
    //reset_i2c();
  }
}

void host_cmd_set(char * item, char * val) {
  if (strncmp(item, "nutrients", 9) == 0)
  {
    set_nutrients(val);
  }
  else if (strncmp(item, "errors", 6) == 0)
  {
    reset_errors();
  }
  else if (strncmp(item, "light", 5) == 0)
  {
    set_light_time(val);
    host_cmd_get("light");
  }
  else if (strncmp(item, "relay", 5) == 0)
  {
    set_relay(val);
  }
  else if (strncmp(item, "misc", 4) == 0)
  {
    set_misc(val);
    print_settings();
  }
  else if (strncmp(item, "rtc", 3) == 0)
  {
    set_time(val);
    print_time();
  }
  else if (strncmp(item, "min", 3) == 0)
  {
    set_minmax(val, 0);
  }
  else if (strncmp(item, "max", 3) == 0)
  {
    set_minmax(val, 1);
  }
  else if (strncmp(item, "pwm", 3) == 0)
  {
    set_pwm(val);
    pwm_ctrl();
    print_pwm();
  }
  else if (strncmp(item, "ph", 2) == 0)
  {
    set_ph(val);
    print_ph();
  }
}


void set_nutrients(char * val) {
  char part[3][16];

  uint8_t part_counter = 0;
  uint8_t j = 0;

  char buf[MAX_STR_LEN];


  for (int i=0; i < strlen(val); i++){
    char t = val[i];
    if (t == ' ') {
      part[part_counter][j] = '\0';
      part[part_counter++];
      j = 0;
    } else {
      part[part_counter][j++] = t;
    }
  }
  part[part_counter][j] = '\0';

  uint8_t pump_number = atoi(part[0]);

  if (strncmp(part[1], "ms", 2) == 0) {
      nutrient_pumps[pump_number].ms_per_ml = atoi(part[2]);
  } else if (strncmp(part[1], "ml", 2) == 0) {
      nutrient_pumps[pump_number].ml_per_10l = atof(part[2]);
  }

}


void set_ph(char * val) {
  char part[2][16];

  uint8_t part_counter = 0;
  uint8_t j = 0;

  char buf[MAX_STR_LEN];


  for (int i=0; i < strlen(val); i++){
    char t = val[i];
    if (t == ' ') {
      part[part_counter][j] = '\0';
      part[part_counter++];
      j = 0;
    } else {
      part[part_counter][j++] = t;
    }
  }
  part[part_counter][j] = '\0';

  if (strncmp(part[0], "min", 3) == 0) {
      ph_setpoints.min_ph = atof(part[1]);

  } else if (strncmp(part[0], "max", 3) == 0) {
      ph_setpoints.max_ph = atof(part[1]);

  } else if (strncmp(part[0], "ms", 2) == 0) {
      ph_setpoints.ms_per_ml = atof(part[1]);

  } else if (strncmp(part[0], "ml", 2) == 0) {
      ph_setpoints.ml_per_ph_per_10l = atof(part[1]);

  }

}


void set_misc(char * val) {
  char part[2][16];

  uint8_t part_counter = 0;
  uint8_t j = 0;

  char buf[MAX_STR_LEN];


  for (int i=0; i < strlen(val); i++){
    char t = val[i];
    if (t == ' ') {
      part[part_counter][j] = '\0';
      part[part_counter++];
      j = 0;
    } else {
      part[part_counter][j++] = t;
    }
  }
  part[part_counter][j] = '\0';

  if (strncmp(part[0], "res_settling_s", 14) == 0) {
    misc_settings.res_settling_time_s = atoi(part[1]);

  } else if (strncmp(part[0], "fill_to_alarm", 13) == 0) {
    misc_settings.fill_to_alarm_level = atoi(part[1]);

  } else if (strncmp(part[0], "sewage_pause", 12) == 0) {
    misc_settings.sewage_pump_pause_s = atof(part[1]);

  } else if (strncmp(part[0], "ec_temp_coef", 12) == 0) {
    misc_settings.ec_temp_coef = atof(part[1]);

  } else if (strncmp(part[0], "sewage_run", 10) == 0) {
    misc_settings.sewage_pump_run_s = atof(part[1]);

  } else if (strncmp(part[0], "ph401", 5) == 0) {
    misc_settings.ph_cal401 = atoi(part[1]);

  } else if (strncmp(part[0], "ph686", 5) == 0) {
    misc_settings.ph_cal686 = atoi(part[1]);

  } else if (strncmp(part[0], "ec_ra", 5) == 0) {
    misc_settings.ec_ra_ohms = atoi(part[1]);

  } else if (strncmp(part[0], "ec_r1", 5) == 0) {
    misc_settings.ec_r1_ohms = atoi(part[1]);

  } else if (strncmp(part[0], "ec_k", 5) == 0) {
    misc_settings.ec_k = atof(part[1]);

  }

}


// should be hh:mm-hh:mm, but all combinations of : and - are allowed
void set_light_time(char * val) {
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
  part[part_counter][j] = '\0';

  light_timer.on_hour = (uint8_t)atoi(part[0]);
  light_timer.on_minutes = (uint8_t)atoi(part[1]);
  light_timer.off_hour = (uint8_t)atoi(part[2]);
  light_timer.off_minutes = (uint8_t)atoi(part[3]);
}

// set min/max temp/humi for exhaust control
void set_minmax(char * val, uint8_t minmax) {
  char part[2][16];

  uint8_t part_counter = 0;
  uint8_t j = 0;
  char buf[MAX_STR_LEN];

  for (int i=0; i < strlen(val); i++){
    char t = val[i];
    if (t == ' ') {
      part[part_counter][j] = '\0';
      part[part_counter++];
      j = 0;
    } else {
      part[part_counter][j++] = t;
    }
  }
  part[part_counter][j] = '\0';

  float setpoint = atof(part[1]);
  sprintf(buf, "{\"event\": \"Setting %s %s to %2.2f\", \"time\": \"%s\"}\r\n", minmax ? "max" : "min", part[0], setpoint, global_state.datestring);

  //snprintf(buf, MAX_STR_LEN, "Setting \"%s\" to %s\r\n",part[0], setpoint);
  TM_USART_Puts(USART2, buf);

  if (strncmp(part[0], "temp_res", 8) == 0) {
    if (minmax)
      coolant_setpoints.max_temp = setpoint;
    else
      coolant_setpoints.min_temp = setpoint;
    print_coolant();
  } else if (strncmp(part[0], "temp_air", 8) == 0) {
    if (minmax)
      exhaust_setpoints.max_temp = setpoint;
    else
      exhaust_setpoints.min_temp = setpoint;
    print_exhaust();
  } else if (strncmp(part[0], "humi", 4) == 0) {
    if (minmax)
      exhaust_setpoints.max_humi = setpoint;
    else
      exhaust_setpoints.min_humi = setpoint;
    print_exhaust();
  }
}

void set_pwm(char * val) {
  char part[2][16];

  uint8_t part_counter = 0;
  uint8_t j = 0;

  char buf[MAX_STR_LEN];


  for (int i=0; i < strlen(val); i++){
    char t = val[i];
    if (t == ' ') {
      part[part_counter][j] = '\0';
      part[part_counter++];
      j = 0;
    } else {
      part[part_counter][j++] = t;
    }
  }
  part[part_counter][j] = '\0';

  uint8_t pwm_val = atoi(part[1]);
  if (pwm_val > 100)
    pwm_val = 100;

    sprintf(buf, "{\"event\": \"Setting PWM Channel %s to %d%%\", \"time\": \"%s\"}\r\n", part[0], pwm_val, global_state.datestring);

  //snprintf(buf, MAX_STR_LEN, "Setting PWM Channel \"%s\" to %d%%\r\n",part[0], pwm_val);
  TM_USART_Puts(USART2, buf);

  if (strncmp(part[0], "fill", 4) == 0) {
    pwms[PWM_FILL_PUMP].duty_percent = pwm_val;
  } else if (strncmp(part[0], "drain", 5) == 0) {
    pwms[PWM_DRAIN_PUMP].duty_percent = pwm_val;
  } else if (strncmp(part[0], "coolant", 7) == 0) {
    pwms[PWM_COOLANT_PUMP].duty_percent = pwm_val;
  } else if (strncmp(part[0], "sewage", 6) == 0) {
    pwms[PWM_SEWAGE_PUMP].duty_percent = pwm_val;
  } else if (strncmp(part[0], "dehumi", 6) == 0) {
    pwms[PWM_DEHUMI_PUMP].duty_percent = pwm_val;
  } else if (strncmp(part[0], "phdown", 6) == 0) {
    pwms[PWM_PHDOWN_PUMP].duty_percent = pwm_val;
  } else if (strncmp(part[0], "nute1", 6) == 0) {
    pwms[PWM_NUTRIENT1_PUMP].duty_percent = pwm_val;
  } else if (strncmp(part[0], "nute2", 6) == 0) {
    pwms[PWM_NUTRIENT2_PUMP].duty_percent = pwm_val;
  } else if (strncmp(part[0], "nute3", 6) == 0) {
    pwms[PWM_NUTRIENT3_PUMP].duty_percent = pwm_val;
  }
}

void set_relay(char * val) {
  char part[2][16];

  uint8_t part_counter = 0;
  uint8_t j = 0;

  char buf[MAX_STR_LEN];


  for (int i=0; i < strlen(val); i++){
    char t = val[i];
    if (t == ' ') {
      part[part_counter][j] = '\0';
      part[part_counter++];
      j = 0;
    } else {
      part[part_counter][j++] = t;
    }
  }
  part[part_counter][j] = '\0';

  uint8_t desired_state = atoi(part[1]);
  if (desired_state > 0)
    desired_state = 1;

    sprintf(buf, "{\"event\": \"Setting Relay %s to %d\", \"time\": \"%s\"}\r\n", part[0], desired_state ? "on" : "off", global_state.datestring);

    //snprintf(buf, MAX_STR_LEN, "Setting Relay \"%s\" to %d\r\n",part[0], desired_state);
    TM_USART_Puts(USART2, buf);

  if (strncmp(part[0], "light", 4) == 0) {
    GPIO_WriteBit(relays[RELAY_LIGHT].gpio_port, relays[RELAY_LIGHT].gpio_pin, desired_state);
  } else if (strncmp(part[0], "exhaust", 7) == 0) {
    GPIO_WriteBit(relays[RELAY_EXHAUST].gpio_port, relays[RELAY_EXHAUST].gpio_pin, desired_state);
  } else if (strncmp(part[0], "aux", 3) == 0) {
    GPIO_WriteBit(relays[RELAY_AUX].gpio_port, relays[RELAY_AUX].gpio_pin, desired_state);
  }
}

