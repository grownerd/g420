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

output_relay_struct_t relays[NUM_RELAYS];
output_pwm_struct_t pwms[NUM_PWM_OUTPUTS];
irq_switch_struct_t irqs[NUM_IRQ_PINS];
uint32_t capsense_data[2];

char dummy_datestring[] = "00.00.0000 00:00:00";
volatile global_state_struct_t global_state = {
  .sewage_tank_empty = 0,
  .drain_cycle_active = 0,
  .reservoir_state = NORMAL_IDLE,
  .datestring = dummy_datestring,
};

uint8_t capsense_chb = 0x3;
uint8_t capsense_capdac = 0x0;
uint16_t capsense_offset = 0x0;
uint16_t capsense_gain = 0x7fff;

char irq_input_names[NUM_IRQ_PINS][MAX_SENSOR_NAME_LENGTH + 1] = {
  "Blue Button",
  "Reservoir min. level",
  "Reservoir max. level",
  "Reservoir Alarm level",
  "",
  "",
  "",
  "Dehumidifier tank full",
  "Dehumidifier tank empty",
  "", // no pin 9
  "Sewage tank full",
  "Sewage tank empty",
  "",
  "",
  "", // no pin 14
  "",
};

char pwm_output_names[NUM_PWM_OUTPUTS][MAX_OUTPUT_NAME_LENGTH + 1] = {
  "Fill Pump",
  "Drain Pump",
  "Coolant Pump",
  "Sewage Pump",
  "Dehumidifier Pump",
  "PH Down Pump",
  "Flora Micro Pump",
  "Flora Gro Pump",
  "Flora Bloom Pump",
};

// list onewire sensors first!
char sensor_names[NUM_SENSORS][MAX_SENSOR_NAME_LENGTH + 1] = {
  "Reservoir Water Temperature",
  "Reservoir Air Temperature",
  "Flowering Chamber Temperature",
  "Flowering Chamber rel. Humidity",
  "Flowering Chamber Air Pressure",
  "Reservoir pH",
  "Reservoir EC",
};

char unit_names[NUM_UNITS][MAX_UNIT_NAME_LENGTH + 1] = {
  "°C",
  "mBar",
  "%",
  "l",
  "pH",
  "S/cm",
  "V",
  "A",
};

void init_bme280(struct bme280_t * bme280);
void read_bme280(struct bme280_t * bme280, float * p_temp, float * p_humi, float * p_press);

uint8_t watchdog_barked;

sensor_t bme280_temp;
sensor_t bme280_humi;
sensor_t bme280_press;
sensor_t adc_ph;
sensor_t adc_ec;

sensor_t ds18b20_sensors[DS18B20_NUM_DEVICES];


int main(void) {

  watchdog_barked = 0;

  SystemInit();
  
  TM_USART_Init(USART2, TM_USART_PinsPack_1, 115200);
  TM_USART_Puts(USART2, "System Startup.\r\n");


  TM_DELAY_Init();
  
  TM_DISCO_LedInit();
  TM_DISCO_LedOn(LED_ORANGE);
  
  TM_RTC_Init(TM_RTC_ClockSource_Internal);

  if (TM_WATCHDOG_Init(TM_WATCHDOG_Timeout_4s)) {
    //System was reset by watchdog
    TM_DISCO_LedOn(LED_RED);
    watchdog_barked = 1;
  } else {
    //System was not reset by watchdog
    TM_DISCO_LedOn(LED_GREEN);
  }
    
  char buf[50];
  TM_RTC_Interrupts(TM_RTC_Int_1s);

  onewire_init();

  init_I2C1();
  struct bme280_t bme280_1;

  init_bme280(&bme280_1); // this is not the driver internal init function!

  bme280_temp.name = sensor_names[TEMP_MAIN];
  bme280_temp.unit = DEG_C;

  bme280_humi.name = sensor_names[HUMI_MAIN];
  bme280_humi.unit = PERCENT;

  bme280_press.name = sensor_names[PRES_MAIN];
  bme280_press.unit = MBAR;

  adc_ph.name = sensor_names[ADC_PH];
  adc_ph.unit = PH;

  adc_ec.name = sensor_names[ADC_EC];
  adc_ec.unit = S_CM;

  nutrient_pumps[0].name = pwm_output_names[PWM_NUTRIENT1_PUMP];
  nutrient_pumps[0].pwm_output = PWM_NUTRIENT1_PUMP;
  nutrient_pumps[1].name = pwm_output_names[PWM_NUTRIENT2_PUMP];
  nutrient_pumps[1].pwm_output = PWM_NUTRIENT2_PUMP;
  nutrient_pumps[2].name = pwm_output_names[PWM_NUTRIENT3_PUMP];
  nutrient_pumps[2].pwm_output = PWM_NUTRIENT3_PUMP;

  uint8_t i = 0;

  for (i=0; i < DS18B20_NUM_DEVICES; i++) {
    ds18b20_sensors[i].name = sensor_names[i];
    ds18b20_sensors[i].unit = DEG_C;
    ds18b20_sensors[i].value = 0;
  }

  reset_errors();

  gpio_init();  
  pwm_init();
  adc_init();
  exti_init();
    

  coolant_setpoints.max_temp = 0.00f;
  coolant_setpoints.min_temp = 0.00f;

  exhaust_setpoints.max_temp = 0.00f;
  exhaust_setpoints.max_humi = 0.00f;
  exhaust_setpoints.min_temp = 0.00f;
  exhaust_setpoints.min_humi = 0.00f;

  misc_settings.fill_to_alarm_level = 0;
  misc_settings.ec_k = 2.88;
  misc_settings.ec_r1_ohms = 1000;
  misc_settings.ec_ra_ohms = 25;
  misc_settings.ec_temp_coef = 0.019;

  read_flash();

  if (!fdc1004_init())
    TM_DISCO_LedOn(LED_RED);


  uint32_t last_time = TM_Time;

  while (1) {

    if ((TM_Time - last_time) >= 1000) {
      last_time = TM_Time;
      //TM_USART_Puts(USART2, "tick\r\n");
      update_datestring();
    }

    // Read Sensors
    fdc1004_read(capsense_data);
    ds18b20_read_temp();
    read_ec(&adc_ec.value);
    read_ph(&adc_ph.value);
    read_bme280(&bme280_1, &bme280_temp.value, &bme280_humi.value, &bme280_press.value);

    // Execute Commands
    command_parser();
    light_scheduler();
    res_temp_ctrl();
    pwm_scheduler();
    pwm_ctrl();
    nutrient_pump_ctrl();
    sewage_pump_ctrl();
    ph_ctrl();
    reservoir_drain_cycle_ctrl();

    //Reset Watchdog
    TM_WATCHDOG_Reset();
  }
}

void emergency_stop(uint8_t release){
  uint32_t runfor = 0xffffffff;

  if (release){
    runfor = 0;
    global_state.sewage_pump_blocked = 0;
  } else {
    global_state.drain_cycle_active = 0;
    global_state.adjusting_ph = 0;
    global_state.adding_nutrients = 0;
    global_state.sewage_pump_blocked = 1;
  }


  uint8_t i;
  for (i=0; i < NUM_PWM_OUTPUTS; i++){
    pwms[i].run_for_ms = runfor;
    pwms[i].duty_percent = 0;
  }
}

void reservoir_drain_cycle_ctrl()
{
  char buf[128];
  if (global_state.drain_cycle_active){
    switch(global_state.reservoir_state){
      case NORMAL_IDLE:
        if (global_state.sewage_tank_empty){
          global_state.reservoir_state = DRAIN_CYCLE_DRAINING;
          pwms[PWM_DRAIN_PUMP].duty_percent = 100;
          sprintf(buf, "{\"event\": \"Reservoir Draining started\", \"time\": \"%s\"}\r\n", global_state.datestring);
        } else {
          sprintf(buf, "{\"error\": \"Sewage Tank full\", \"time\": \"%s\"}\r\n", global_state.datestring);
          global_state.drain_cycle_active = 0;
        }
        break;

      case DRAIN_CYCLE_DRAINING:
        TM_PWMIN_Get(&PWMIN1_Data);
        if (PWMIN1_Data.Frequency > 0) {
          print_pwmin(PWMIN_RES_DRAIN, PWMIN1_Data.Frequency);
        } else {
          global_state.reservoir_state = DRAIN_CYCLE_EMPTY;
          pwms[PWM_DRAIN_PUMP].duty_percent = 0;
          sprintf(buf, "{\"event\": \"Reservoir Draining complete\", \"time\": \"%s\"}\r\n", global_state.datestring);
        }
        break;

      case DRAIN_CYCLE_EMPTY:
        global_state.reservoir_state = DRAIN_CYCLE_FILLING;
        sprintf(buf, "{\"event\": \"Reservoir Filling started\", \"time\": \"%s\"}\r\n", global_state.datestring);
        pwms[PWM_FILL_PUMP].duty_percent = 100;
        break;

      case DRAIN_CYCLE_FULL:
        global_state.adding_nutrients = 1;
        global_state.reservoir_state = DRAIN_CYCLE_NUTRIENTS;
        break;

    }
  } else {
    pwms[PWM_DRAIN_PUMP].duty_percent = 0;
    pwms[PWM_FILL_PUMP].duty_percent = 0;
  }
  TM_USART_Puts(USART2, buf);
}

void nutrient_pump_ctrl(void){
  if (global_state.adding_nutrients){
    static uint32_t last_time = 0;
    if (!last_time) last_time = TM_Time;

    if ((TM_Time - last_time) >= (misc_settings.res_settling_time_s * 1000)) {
      static uint8_t i = 0;

      if (i < 3) {
        uint8_t pwm_out = nutrient_pumps[i].pwm_output;
        pwms[pwm_out].run_for_ms = nutrient_pumps[i].dosage_ml * nutrient_pumps[i].ms_per_ml;
      } else{
        if (global_state.reservoir_state == DRAIN_CYCLE_NUTRIENTS)
          global_state.reservoir_state = DRAIN_CYCLE_PHDOWN;
        global_state.adding_nutrients = 0;
        i = 0;
      }

      i++;
    }
  }
}

void sewage_pump_ctrl(void){
  if (!global_state.sewage_tank_empty && !global_state.sewage_pump_blocked){
    static uint32_t last_time = 0;
    if (!last_time) last_time = TM_Time;

    if ((TM_Time - last_time) >= (misc_settings.sewage_pump_pause_s * 1000)) {
        pwms[PWM_SEWAGE_PUMP].run_for_ms = misc_settings.sewage_pump_run_s;
    }
    last_time = TM_Time;
  }
}

void ph_ctrl(void){
  static uint32_t last_time = 0;
  if (!last_time) last_time = TM_Time;

  if ((TM_Time - last_time) >= (misc_settings.res_settling_time_s * 1000)) {
    float diff_to_min = adc_ph.value - ph_setpoints.min_ph;

    if ((adc_ph.value >= ph_setpoints.max_ph) ||
      ((global_state.adjusting_ph) && (adc_ph.value >= ph_setpoints.min_ph))) {

      float ml_to_add = diff_to_min * ph_setpoints.ml_per_ph;
      uint32_t ms_to_run = ml_to_add * ph_setpoints.ms_per_ml;
      pwms[PWM_PHDOWN_PUMP].run_for_ms = ms_to_run;
    }
    last_time = TM_Time;
  }

  if (((global_state.reservoir_state == DRAIN_CYCLE_PHDOWN) ||
    (global_state.reservoir_state == NORMAL_PHDOWN)) &&
    (pwms[PWM_PHDOWN_PUMP].run_for_ms == 0))
      global_state.reservoir_state = NORMAL_IDLE;
}

void pwm_ctrl(void){
  uint8_t i;
  for (i=0; i<NUM_PWM_OUTPUTS; i++) {
    TM_PWM_SetChannelPercent(pwms[i].tim_data, pwms[i].pwm_channel, pwms[i].duty_percent);
  }
}

void exhaust_ctrl(void){

  uint8_t desired_state = 0;
  uint8_t current_state = GPIO_ReadInputDataBit(relays[RELAY_EXHAUST].gpio_port, relays[RELAY_EXHAUST].gpio_pin);

  if (bme280_temp.value > exhaust_setpoints.max_temp || bme280_humi.value > exhaust_setpoints.max_humi){
    desired_state = 1;
  }
  if (bme280_temp.value < exhaust_setpoints.min_temp && bme280_humi.value < exhaust_setpoints.min_humi){
    desired_state = 0;
  }

  if (desired_state != current_state) {
    GPIO_WriteBit(relays[RELAY_EXHAUST].gpio_port, relays[RELAY_EXHAUST].gpio_pin, desired_state);
    char buf[128];
    sprintf(buf, "{\"event\": \"Exhaust turned %s\", \"time\": \"%s\"}\r\n", desired_state ? "on" : "off", global_state.datestring);
    TM_USART_Puts(USART2, buf);
  }
}

void res_temp_ctrl(void){

  uint8_t current_state = pwms[PWM_COOLANT_PUMP].duty_percent;
  uint8_t desired_state = current_state;

  if (ds18b20_sensors[TEMP_RES].value > coolant_setpoints.max_temp)
    desired_state = 100;

  if (ds18b20_sensors[TEMP_RES].value < coolant_setpoints.min_temp)
    desired_state = 0;

  if (desired_state != current_state) {
    pwms[PWM_COOLANT_PUMP].duty_percent = desired_state;
    pwms[PWM_COOLANT_PUMP].run_for_ms = 0xffffffff;
    char buf[256];
    sprintf(buf, "{ \"event\": \"Coolant pump turned %s\", \"time\": \"%s\"}\r\n", desired_state ? "on" : "off", global_state.datestring);
    TM_USART_Puts(USART2, buf);
  }
}

void pwm_scheduler(void) {
  static uint32_t last_time = 0;
  if (last_time == 0) last_time = TM_Time;

  if ((TM_Time - last_time) >= 1) {
    uint8_t i;
    for (i=0; i<NUM_PWM_OUTPUTS; i++) {
      uint32_t run_for_ms = pwms[i].run_for_ms;
      if (run_for_ms != 0xffffffff) {
        if (run_for_ms > 0) {
          pwms[i].duty_percent = 100;
          pwms[i].run_for_ms -= (TM_Time - last_time);
          if (pwms[i].run_for_ms > run_for_ms)
            pwms[i].run_for_ms = 0;
        } else {
          pwms[i].duty_percent = 0;
        }
      }
    }
    last_time = TM_Time;
  }
}

void light_scheduler(void) {

  uint8_t desired_state = 0;

  light_timer.state = GPIO_ReadInputDataBit(relays[RELAY_LIGHT].gpio_port, relays[RELAY_LIGHT].gpio_pin);

  TM_RTC_Time_t Time;
  TM_RTC_GetDateTime(&Time, TM_RTC_Format_BIN);

  uint16_t current_time = Time.hours << 8 | Time.minutes;
  uint16_t on_time = light_timer.on_hour << 8 | light_timer.on_minutes;
  uint16_t off_time = light_timer.off_hour << 8 | light_timer.off_minutes;

  if (on_time > off_time) {
    if (current_time >= on_time || current_time < off_time)
      desired_state = 1;

  } else if (on_time < off_time) {
    if (current_time >= on_time && current_time < off_time)
      desired_state = 1;
  }

  if (light_timer.state != desired_state) {
    GPIO_WriteBit(relays[RELAY_LIGHT].gpio_port, relays[RELAY_LIGHT].gpio_pin, desired_state);
    char buf[32];
    sprintf(buf, "{\"event\": \"Main Light turned %s\", \"time\": \"%s\"}\r\n", desired_state ? "on" : "off", global_state.datestring);
    TM_USART_Puts(USART2, buf);
  }
}

void print_env(void) {
  char buf[MAX_STR_LEN];

  TM_USART_Puts(USART2, "{\"name\":\"Sensors\",\"content\":[\r\n");
  sprintf(buf, "\t{\"name\":\"%s\", \"value\":%.2f, \"unit\":\"%s\"},\r\n\t{\"name\":\"%s\", \"value\":%.2f, \"unit\":\"%s\"}, \r\n\t{\"name\":\"%s\", \"value\":%.2f, \"unit\":\"%s\"},\r\n",
    bme280_temp.name,
    bme280_temp.value,
    unit_names[bme280_temp.unit],
    bme280_humi.name,
    bme280_humi.value,
    unit_names[bme280_humi.unit],
    bme280_press.name,
    bme280_press.value,
    unit_names[bme280_press.unit]
  );
  TM_USART_Puts(USART2, buf);

  uint8_t i;
  for (i=0; i< DS18B20_NUM_DEVICES; i++) {
    sprintf(buf, "\t{\"name\":\"%s\",\"value\":%.2f, \"unit\":\"%s\"}",
      ds18b20_sensors[i].name,
      ds18b20_sensors[i].value,
      unit_names[DEG_C]
    );
    TM_USART_Puts(USART2, buf);
    if (i == DS18B20_NUM_DEVICES -1)
      TM_USART_Puts(USART2, "\r\n");
    else
      TM_USART_Puts(USART2, ",\r\n");
  }
  TM_USART_Puts(USART2, "]}\r\n");
}

void print_light(void) {
  char buf[MAX_STR_LEN];

  light_timer.state = GPIO_ReadInputDataBit(relays[RELAY_LIGHT].gpio_port, relays[RELAY_LIGHT].gpio_pin);
  sprintf(buf, "{\"name\":\"Main Light\",\"content\":{\r\n\t\"on_time\":\"%02d:%02d\",\r\n\t\"off_time\":\"%02d:%02d\",\r\n\t\"state\":\"%s\"\r\n}}\r\n",
    light_timer.on_hour,
    light_timer.on_minutes,
    light_timer.off_hour,
    light_timer.off_minutes,
    light_timer.state ? "On" : "Off"
  );
  TM_USART_Puts(USART2, buf);

}

void print_coolant(void) {
  char buf[MAX_STR_LEN];

  coolant_setpoints.state = pwms[PWM_COOLANT_PUMP].duty_percent;
  sprintf(buf, "{\"name\":\"Coolant Control\",\"content\":{\r\n\t\"max_temp\":%.2f,\r\n\t\"min_temp\":%.2f,\r\n\t\"state\":\"%s\"\r\n}}\r\n",
    coolant_setpoints.max_temp,
    coolant_setpoints.min_temp,
    coolant_setpoints.state ? "On" : "Off"
  );
  TM_USART_Puts(USART2, buf);

}

void print_exhaust(void) {
  char buf[MAX_STR_LEN];

  exhaust_setpoints.state = GPIO_ReadInputDataBit(relays[RELAY_EXHAUST].gpio_port, relays[RELAY_EXHAUST].gpio_pin);
  sprintf(buf, "{\"name\":\"Main Exhaust\",\"content\":{\r\n\t\"max_temp\":%.2f,\r\n\t\"max_humi\":%.2f,\r\n\t\"min_temp\":%.2f,\r\n\t\"min_humi\":%.2f,\r\n\t\"state\":\"%s\"\r\n}}\r\n",
    exhaust_setpoints.max_temp,
    exhaust_setpoints.max_humi,
    exhaust_setpoints.min_temp,
    exhaust_setpoints.min_humi,
    exhaust_setpoints.state ? "On" : "Off"
  );
  TM_USART_Puts(USART2, buf);

}

void print_relays(void) {
  char buf[MAX_STR_LEN];

  TM_USART_Puts(USART2, "{\"name\":\"Relays\",\"content\":[\r\n");
  uint8_t i;
  for (i=0; i < NUM_RELAYS; i++) {
    uint8_t state = GPIO_ReadInputDataBit(relays[i].gpio_port, relays[i].gpio_pin);

    sprintf((char *) buf, "\t{\"relay_id\":%d, \"state\":%d}",
      i, state
    );
    TM_USART_Puts(USART2, buf);
    if (i == NUM_RELAYS -1)
      TM_USART_Puts(USART2, "\r\n");
    else
      TM_USART_Puts(USART2, ",\r\n");
  }
  TM_USART_Puts(USART2, "]}\r\n");

}

void print_errors(void){
  char buf[MAX_STR_LEN];
  uint8_t i;

  TM_USART_Puts(USART2, "{\"name\":\"Errors\",\"content\":[\r\n");
  
  for (i=0; i<DS18B20_NUM_DEVICES; i++) {
    
    sprintf((char *) buf, "\t{\"name\":\"%s\", \"count\":%d}", ds18b20_sensors[i].name, ds18b20_sensors[i].error_count);
    TM_USART_Puts(USART2, buf);
    if (i == DS18B20_NUM_DEVICES -1)
      TM_USART_Puts(USART2, "\r\n");
    else
      TM_USART_Puts(USART2, ",\r\n");
  }
  TM_USART_Puts(USART2, "]}\r\n");

}

void reset_errors(void) {
  uint8_t i;
  for (i=0; i<DS18B20_NUM_DEVICES; i++) {
    ds18b20_sensors[i].error_count = 0;
  }
}

void print_pwm(void){
  char buf[MAX_STR_LEN];
  uint8_t i;

  TM_USART_Puts(USART2, "{\"name\":\"PWM Channels\",\"content\":[\r\n");
  
  for (i=0; i<NUM_PWM_OUTPUTS; i++) {

    uint16_t chan = pwms[i].pwm_channel;
    
    sprintf(buf, "\t{\"name\":\"%s\", \"percent\":%d,\"period\":%d}", pwm_output_names[i], pwms[i].duty_percent, pwms[i].tim_data->CH_Periods[chan]);
    TM_USART_Puts(USART2, buf);
    if (i == NUM_PWM_OUTPUTS -1)
      TM_USART_Puts(USART2, "\r\n");
    else
      TM_USART_Puts(USART2, ",\r\n");
  }
  TM_USART_Puts(USART2, "]}\r\n");

}


void print_capsense(void){
  char buf[MAX_STR_LEN];
  uint8_t i;

  TM_USART_Puts(USART2, "{\"name\":\"Capsense Channels\",\"content\":[\r\n");
  
  for (i=0; i<2; i++) {

    sprintf(buf, "\t{\"id\":%d, \"capacitance\":%d}", i, capsense_data[i]);
    TM_USART_Puts(USART2, buf);
    if (i == 1)
      TM_USART_Puts(USART2, "\r\n");
    else
      TM_USART_Puts(USART2, ",\r\n");
  }
  TM_USART_Puts(USART2, "]}\r\n");

}


void print_pwmin(uint8_t id, float freq){
  char buf[MAX_STR_LEN];
  sprintf(buf, "{\"name\": \"PWM Input\", \"id\":%d, \"frequency\":%7.2f}\r\n", id, freq);
  TM_USART_Puts(USART2, buf);

}


void print_irqs(void){
  char buf[MAX_STR_LEN];
  uint8_t i;

  TM_USART_Puts(USART2, "{\"name\":\"IRQ Inputs\",\"content\":[\r\n");
  
  for (i=0; i<NUM_IRQ_PINS; i++) {

    uint8_t pinstate = GPIO_ReadInputDataBit(irqs[i].gpio_port, irqs[i].gpio_pin);
    sprintf(buf, "\t{\"name\":\"%s\", \"state\":%d}", irq_input_names[i], pinstate);
    TM_USART_Puts(USART2, buf);
    if (i == NUM_IRQ_PINS -1)
      TM_USART_Puts(USART2, "\r\n");
    else
      TM_USART_Puts(USART2, ",\r\n");
  }
  TM_USART_Puts(USART2, "]}\r\n");
}


void print_ph(void){
  char buf[MAX_STR_LEN];
  sprintf(buf, "{\"name\": \"%s\", \"value\":%f}\r\n", sensor_names[ADC_PH], adc_ph.value);
  TM_USART_Puts(USART2, buf);
}


void print_ec(void){
  char buf[MAX_STR_LEN];
  sprintf(buf, "{\"name\": \"%s\", \"value\":%f}\r\n", sensor_names[ADC_EC], adc_ec.value);
  TM_USART_Puts(USART2, buf);
}


void print_settings(void){
  char buf[MAX_STR_LEN];
  uint8_t i;

  sprintf(buf, "{\"name\":\"Misc. Settings\",\"content\":{\r\n\t\
\"ec_k\":%1.4f,\r\n\t\
\"ec_temp_coef\":%1.4f,\r\n\t\
\"ec_r1_ohms\":%d,\r\n\t\
\"ec_ra_ohms\":%d,\r\n\t\
\"ph_cal401_mv\":%d,\r\n\t\
\"ph_cal686_mv\":%d,\r\n\t\
\"res_settling_time\":%d,\r\n\t\
\"sewage_pump_pause_s\":%d,\r\n\t\
\"sewage_pump_run_s\":%d,\r\n\t\
\"fill_to_alarm_level\":%d}\r\n\
}\r\n",
    misc_settings.ec_k,
    misc_settings.ec_temp_coef,
    misc_settings.ec_r1_ohms,
    misc_settings.ec_ra_ohms,
    misc_settings.ph_cal401_mv,
    misc_settings.ph_cal686_mv,
    misc_settings.res_settling_time_s,
    misc_settings.sewage_pump_pause_s,
    misc_settings.sewage_pump_run_s,
    misc_settings.fill_to_alarm_level
  );
  TM_USART_Puts(USART2, buf);
}

