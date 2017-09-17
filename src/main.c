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
gpio_output_struct_t gpio_outputs[NUM_GPIO_OUTPUTS];
irq_switch_struct_t irqs[NUM_IRQ_PINS];
uint32_t capsense_data[2];

char dummy_datestring[] = "00.00.0000 00:00:00";
volatile global_state_struct_t global_state = {
  .sewage_tank_empty = 1,
  .drain_cycle_active = 0,
  .reservoir_state = NORMAL_IDLE,
  .datestring = dummy_datestring,
};

uint8_t capsense_chb = 0x3;
uint8_t capsense_capdac = 0x0;
uint16_t capsense_offset = 0x0;
uint16_t capsense_gain = 0x7fff;

/* TODO: extend all name arrays that will have RRDs by a column "ds_name"
  and output them accordingly.

  Also add unix timestamps to all events and make them RRD compatible
*/

char irq_input_names[NUM_IRQ_PINS][MAX_SENSOR_NAME_LENGTH + 1] = {
  "Blue Button",
  "", // no pin 1
  "Reservoir max. level",
  "", // no pin 3
  "Reservoir min. level",
  "Reservoir Alarm level",
  "unused 6",
  "Dehumidifier tank full",
  "Dehumidifier tank empty",
  "", // no pin 9
  "Sewage tank full",
  "Sewage tank empty",
  "unused 12",
  "unused 13",
  "", // no pin 14
  "Water tank empty",
};

char res_state_names[NUM_RES_STATES][MAX_SENSOR_NAME_LENGTH + 1] = {
  "DRAIN_CYCLE_DRAINING",
  "DRAIN_CYCLE_EMPTY",
  "DRAIN_CYCLE_FILLING",
  "DRAIN_CYCLE_FULL",
  "DRAIN_CYCLE_NUTRIENTS",
  "NORMAL_MIN",
  "NORMAL_FILLING",
  "NORMAL_MAX",
  "NORMAL_NUTRIENTS",
  "NORMAL_IDLE",
  "MANUAL_DRAIN",
  "MANUAL_FILL",
  "LEVEL_ERROR",
  "EMERGENCY STOP",
};

char gpio_output_names[NUM_GPIO_OUTPUTS][MAX_OUTPUT_NAME_LENGTH + 1] = {
  "Fill Pump",
  "Drain Pump",
  "Coolant Pump",
  "Sewage Pump",
  "Dehumidifier Pump",
  "PH Down Pump",
  "Flora Micro Pump",
  "Flora Gro Pump",
  "Flora Bloom Pump",
  "Deep Red Leds",
  "Nutrient Stirrers",
};

// list onewire sensors first!
char sensor_names[NUM_SENSORS][2][MAX_SENSOR_NAME_LENGTH + 1] = {
  { "Water Tank Temperature", "storage_temp" },
  { "Reservoir Temperature", "res_temp" },
  { "Tent Temperature", "tent_temp" },
  { "Tent rel. Humidity", "tent_humi" },
  { "Tent Pressure", "tent_press" },
  { "Reservoir pH", "res_ph" },
  { "Reservoir EC", "res_ec" },
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

s8 init_bme280(struct bme280_t * bme280);
void read_bme280(struct bme280_t * bme280, float * p_temp, float * p_humi, float * p_press);

uint8_t watchdog_barked;

sensor_t bme280_temp;
sensor_t bme280_humi;
sensor_t bme280_press;
sensor_t adc_ph;
sensor_t adc_ec;

sensor_t ds18b20_sensors[DS18B20_NUM_DEVICES];


int main(void) {
  char buf[MAX_STR_LEN];

  watchdog_barked = 0;

  set_defaults();

  SystemInit();
  TM_RTC_Init(TM_RTC_ClockSource_Internal);
  update_datestring();
  
  TM_USART_Init(USART2, TM_USART_PinsPack_1, 115200);
  sprintf(buf, "{\"event\": \"System Startup\", \"time\": \"%s\"}\r\n", global_state.datestring);
  TM_USART_Puts(USART2, buf);


  gpio_init();  
  //pwm_init();
  pwmin_init();
  adc_init();
  exti_init();
    
  TM_DELAY_Init();
  TM_DISCO_LedInit();
  TM_DISCO_LedOn(LED_ORANGE);
  TM_RTC_Interrupts(TM_RTC_Int_1s);
  
  // this can take some time, so do it before the watchdog init
  onewire_init();

  if (TM_WATCHDOG_Init(TM_WATCHDOG_Timeout_4s)) {
  //if (1){
    //System was reset by watchdog
    TM_DISCO_LedOn(LED_RED);
    watchdog_barked = 1;
  } else {
    //System was not reset by watchdog
    TM_DISCO_LedOn(LED_GREEN);
  }
    


  struct bme280_t bme280_1;

  s8 bme280_init_result = -1;
  i2c_bus_reset();
  init_I2C1();
  bme280_init_result = init_bme280(&bme280_1); // this is not the driver internal init function!

  if (bme280_init_result == 0) {
    sprintf(buf, "{\"event\": \"BME280 Initialized\", \"time\": \"%s\"}\r\n", global_state.datestring);
    TM_USART_Puts(USART2, buf);
    TM_WATCHDOG_Reset();
  } else {
    sprintf(buf, "{\"error\": \"BME280 Initialization Error!\", \"time\": \"%s\"}\r\n", global_state.datestring);
    TM_USART_Puts(USART2, buf);
  }
    

  bme280_temp.name = sensor_names[TEMP_MAIN][0];
  bme280_temp.unit = DEG_C;

  bme280_humi.name = sensor_names[HUMI_MAIN][0];
  bme280_humi.unit = PERCENT;

  bme280_press.name = sensor_names[PRES_MAIN][0];
  bme280_press.unit = MBAR;

  adc_ph.name = sensor_names[ADC_PH][0];
  adc_ph.unit = PH;

  adc_ec.name = sensor_names[ADC_EC][0];
  adc_ec.unit = S_CM;


  fdc1004_init();


  uint8_t i = 0;
  for (i=0; i < DS18B20_NUM_DEVICES; i++) {
    ds18b20_sensors[i].name = sensor_names[i][0];
    ds18b20_sensors[i].unit = DEG_C;
    ds18b20_sensors[i].value = 0;
  }

  reset_errors();


  read_flash();

  uint32_t last_time = TM_Time;
  while (1) {

    // Five second loop
    if ((TM_Time - last_time) >= 5000) {
      last_time = TM_Time;

      TM_PWMIN_Get(&PWMIN1_Data);
      if (PWMIN1_Data.Frequency > 0)
        print_pwmin(PWMIN_RES_DRAIN, PWMIN1_Data.Frequency);

    }
    // End of 5 second loop

    // Read Sensors
    fdc1004_read(capsense_data);
    ds18b20_read_temp();
    read_ec(&adc_ec.value);
    read_ph(&adc_ph.value);
    read_bme280(&bme280_1, &bme280_temp.value, &bme280_humi.value, &bme280_press.value);

    // If we had any timeouts on the I2C bus, try to recover by clocking out the remaining garbage and restarting the peripheral
    if (global_state.i2c_errors > 0){
      bme280_init_result = -1;
      while (bme280_init_result != 0){
        TM_WATCHDOG_Reset();
        deinit_I2C1();
        TM_WATCHDOG_Reset();

        //sprintf(buf, "{\"event\": \"I2C Bus Reset\", \"time\": \"%s\"}\r\n", global_state.datestring);
        //TM_USART_Puts(USART2, buf);

        i2c_bus_reset();
        init_I2C1();
        bme280_init_result = init_bme280(&bme280_1); // this is not the driver internal init function!
        TM_WATCHDOG_Reset();

        if (bme280_init_result != 0) {
          sprintf(buf, "{\"event\": \"BME280 Initialization Error!\", \"time\": \"%s\"}\r\n", global_state.datestring);
          TM_USART_Puts(USART2, buf);
          // force a reboot if the bus has been restarted too often
          if ((global_state.i2c_restarts > misc_settings.i2c_max_restarts)
            && (!global_state.adjusting_ph)
            && (!global_state.adding_nutrients)
            && (!global_state.drain_cycle_active)
            && (global_state.reservoir_state == NORMAL_IDLE)){
            sprintf(buf, "{\"event\": \"Too many I2C Errors - Forcing Reboot\", \"time\": \"%s\"}\r\n", global_state.datestring);
            TM_USART_Puts(USART2, buf);
            while (1);
          }
        } else {
          fdc1004_init();
          //sprintf(buf, "{\"event\": \"BME280 Initialized\", \"time\": \"%s\"}\r\n", global_state.datestring);
          //TM_USART_Puts(USART2, buf);
        }
      }
      global_state.i2c_errors = 0;
      global_state.i2c_restarts++;
    }

    // Execute Commands
    command_parser();
    light_scheduler();
    res_temp_ctrl();
    nutrient_pump_ctrl();
    sewage_pump_ctrl();
    ph_ctrl();
    reservoir_level_ctrl();
    gpio_ctrl();


    //Reset Watchdog
    TM_WATCHDOG_Reset();
  }
}

void set_defaults(){
  nutrient_pumps[0].name = gpio_output_names[GPIO_OUTPUT_NUTRIENT1_PUMP];
  nutrient_pumps[0].gpio_output = GPIO_OUTPUT_NUTRIENT1_PUMP;
  nutrient_pumps[0].ms_per_ml = 947;
  nutrient_pumps[0].ml_per_10l = 2.5;
  nutrient_pumps[1].name = gpio_output_names[GPIO_OUTPUT_NUTRIENT2_PUMP];
  nutrient_pumps[1].gpio_output = GPIO_OUTPUT_NUTRIENT2_PUMP;
  nutrient_pumps[1].ms_per_ml = 1219;
  nutrient_pumps[1].ml_per_10l = 2.5;
  nutrient_pumps[2].name = gpio_output_names[GPIO_OUTPUT_NUTRIENT3_PUMP];
  nutrient_pumps[2].gpio_output = GPIO_OUTPUT_NUTRIENT3_PUMP;
  nutrient_pumps[2].ms_per_ml = 1020;
  nutrient_pumps[2].ml_per_10l = 2.5;

  ph_setpoints.min_ph = 5.8f;
  ph_setpoints.max_ph = 6.0f;
  ph_setpoints.ms_per_ml = 970;
  ph_setpoints.ml_per_ph_per_10l = 0.8f;

  coolant_setpoints.max_temp = 16.49f;
  coolant_setpoints.min_temp = 16.41f;

  exhaust_setpoints.max_temp = 26.00f;
  exhaust_setpoints.max_humi = 60.00f;
  exhaust_setpoints.min_temp = 20.00f;
  exhaust_setpoints.min_humi = 40.00f;

  light_timer.on_hour = 10;
  light_timer.on_minutes = 0;
  light_timer.off_hour = 22;
  light_timer.off_minutes = 0;

  misc_settings.nutrient_pause_ms = 5000;
  misc_settings.res_liters_min = 10.0f;
  misc_settings.res_liters_max = 12.0f;
  misc_settings.res_liters_alarm = 14.0f;
  misc_settings.nutrient_factor = 0.5f;
  misc_settings.flow_sensor_lag = 3000;
  misc_settings.i2c_max_restarts = 3;
  misc_settings.i2c_timeout = 10;
  misc_settings.i2c_break_enabled = 1;
  misc_settings.fill_to_alarm_level = 0;
  misc_settings.ec_k = 1.316f;
  misc_settings.ec_r1_ohms = 470;
  misc_settings.ec_ra_ohms = 25;
  misc_settings.ec_temp_coef = 0.019f;
  misc_settings.ec_read_interval_s = 60;
  misc_settings.sewage_pump_run_s = 60; // run for one minute
  misc_settings.sewage_pump_pause_s = 7200; // then pause for two hours
  misc_settings.ph4_ph = 4.01f;
  misc_settings.ph7_ph = 6.86f;
  misc_settings.ph4_v = 2.91;
  misc_settings.ph7_v = 2.60f;
  misc_settings.vcc_v = 2.94f;
  //misc_settings.ph_step = 0.10877193f;
  misc_settings.res_settling_time_s = 300;

}

void emergency_stop(uint8_t release){
  uint32_t runfor = 0xffffffff;

  char buf[MAX_STR_LEN];
  memset(buf, 0, sizeof(buf));

  if (release){
    sprintf(buf, "{\"event\": \"Emergency Stop Release\", \"time\": \"%s\"}\r\n", global_state.datestring);
    runfor = 0;
    global_state.sewage_pump_blocked = 0;
    global_state.reservoir_state = NORMAL_IDLE;
  } else {
    sprintf(buf, "{\"event\": \"Emergency STOP!\", \"time\": \"%s\"}\r\n", global_state.datestring);
    global_state.drain_cycle_active = 0;
    global_state.adjusting_ph = 0;
    global_state.stirring_nutrients = 0;
    global_state.adding_nutrients = 0;
    global_state.sewage_pump_blocked = 1;
    global_state.reservoir_state = EMERGENCY_STOP;
  }

  uint8_t i;
  for (i=0; i < NUM_GPIO_OUTPUTS; i++){
    gpio_outputs[i].run_for_ms = runfor;
    gpio_outputs[i].desired_state = 0;
    GPIO_WriteBit(gpio_outputs[i].gpio_port, gpio_outputs[i].gpio_pin, gpio_outputs[i].desired_state);
  }

  TM_USART_Puts(USART2, buf);
}

void reservoir_level_ctrl()
{
  static uint8_t last_min_state = 1;
  static uint32_t drain_time = 0;
  static uint32_t stirring_time = 0;
  char buf[MAX_STR_LEN];
  memset(buf, 0, sizeof(buf));

  global_state.reservoir_min = (GPIO_ReadInputDataBit(irqs[SWITCH_RES_MIN].gpio_port, irqs[SWITCH_RES_MIN].gpio_pin));
  global_state.reservoir_max = (!GPIO_ReadInputDataBit(irqs[SWITCH_RES_MAX].gpio_port, irqs[SWITCH_RES_MAX].gpio_pin));
  global_state.reservoir_alarm = (!GPIO_ReadInputDataBit(irqs[SWITCH_RES_ALARM].gpio_port, irqs[SWITCH_RES_ALARM].gpio_pin));
  global_state.water_tank_empty = (GPIO_ReadInputDataBit(irqs[SWITCH_WATER_EMPTY].gpio_port, irqs[SWITCH_WATER_EMPTY].gpio_pin));
  global_state.sewage_tank_empty = (GPIO_ReadInputDataBit(irqs[SWITCH_SEWAGE_MIN].gpio_port, irqs[SWITCH_SEWAGE_MIN].gpio_pin));
  global_state.sewage_tank_full = (!GPIO_ReadInputDataBit(irqs[SWITCH_SEWAGE_MAX].gpio_port, irqs[SWITCH_SEWAGE_MAX].gpio_pin));

  if ((global_state.reservoir_min) && ((global_state.reservoir_max) || (global_state.reservoir_alarm))){
    global_state.reservoir_state = LEVEL_ERROR;
  }

  switch(global_state.reservoir_state){

    case NORMAL_IDLE:
      if (global_state.drain_cycle_active){
        if (!global_state.sewage_tank_empty){
          sprintf(buf, "{\"error\": \"Sewage Tank not empty\", \"time\": \"%s\"}\r\n", global_state.datestring);
          global_state.drain_cycle_active = 0;
        } else if (global_state.water_tank_empty){
          sprintf(buf, "{\"error\": \"Water Tank empty\", \"time\": \"%s\"}\r\n", global_state.datestring);
          global_state.drain_cycle_active = 0;
        } else {
          gpio_outputs[GPIO_OUTPUT_DRAIN_PUMP].run_for_ms = 5000;
          gpio_outputs[GPIO_OUTPUT_FILL_PUMP].run_for_ms = 0;
          global_state.stirring_nutrients = 1;
          global_state.reservoir_state = DRAIN_CYCLE_DRAINING;
          sprintf(buf, "{\"event\": \"Reservoir Draining started\", \"time\": \"%s\"}\r\n", global_state.datestring);
          break;
        }
      } 

      if ((misc_settings.fill_to_alarm_level) && (!global_state.reservoir_alarm)){
        global_state.reservoir_state = NORMAL_FILLING;
      }

      if ((global_state.reservoir_min) && (!global_state.reservoir_max) && (!global_state.reservoir_alarm)){
        global_state.reservoir_state = NORMAL_MIN;
      }

      if ((!global_state.adding_nutrients) && (global_state.nutrients_done))
        global_state.nutrients_done = 0;

      gpio_outputs[GPIO_OUTPUT_DRAIN_PUMP].run_for_ms = 0;
      gpio_outputs[GPIO_OUTPUT_FILL_PUMP].run_for_ms = 0;
      break;

    case NORMAL_MIN:
      global_state.stirring_nutrients = 1;
      if (!stirring_time) stirring_time = TM_Time;

      if ((global_state.reservoir_min) && (!global_state.reservoir_max) && (!global_state.reservoir_alarm)){
        if (TM_Time >= (stirring_time + misc_settings.res_settling_time_s * 1000)) {
          global_state.stirring_nutrients = 0;
          if (!global_state.nutrients_done)
            global_state.adding_nutrients = 1;
          global_state.reservoir_state = NORMAL_FILLING;
          sprintf(buf, "{\"event\": \"Reservoir topping up started\", \"time\": \"%s\"}\r\n", global_state.datestring);
        } else {
          global_state.reservoir_state = NORMAL_IDLE;
        }
      }
      break;

    case NORMAL_FILLING:
      if (((global_state.reservoir_max) && (!global_state.reservoir_alarm) && (misc_settings.fill_to_alarm_level))
      || ((!global_state.reservoir_max) && (!global_state.reservoir_alarm))){
        gpio_outputs[GPIO_OUTPUT_FILL_PUMP].run_for_ms = 5000;
      } else {
        global_state.reservoir_state = NORMAL_MAX;
      }
      break;

    case DRAIN_CYCLE_FILLING:

      if ((!global_state.reservoir_min) && (!global_state.reservoir_max) && (!global_state.reservoir_alarm)){
        if (last_min_state) {
          global_state.stirring_nutrients = 0;
          if (!global_state.nutrients_done)
            global_state.adding_nutrients = 1;
          
        }
      }

      if (((global_state.reservoir_max) && (!global_state.reservoir_alarm) && (misc_settings.fill_to_alarm_level))
      || ((!global_state.reservoir_max) && (!global_state.reservoir_alarm))){
        gpio_outputs[GPIO_OUTPUT_FILL_PUMP].run_for_ms = 5000;
      } else {
        global_state.reservoir_state = DRAIN_CYCLE_FULL;
      }
      break;

    case NORMAL_MAX:
      if (global_state.nutrients_done){
        global_state.nutrients_done = 0;
        global_state.reservoir_state = NORMAL_IDLE;
      } else {
        global_state.reservoir_state = NORMAL_NUTRIENTS;
      }
      sprintf(buf, "{\"event\": \"Reservoir topping up completed\", \"time\": \"%s\"}\r\n", global_state.datestring);
      gpio_outputs[GPIO_OUTPUT_FILL_PUMP].run_for_ms = 0;
      break;

    case DRAIN_CYCLE_FULL:
      global_state.drain_cycle_active = 0;
      if (global_state.nutrients_done){
        global_state.nutrients_done = 0;
        global_state.reservoir_state = NORMAL_IDLE;
      } else {
        global_state.reservoir_state = DRAIN_CYCLE_NUTRIENTS;
      }
      sprintf(buf, "{\"event\": \"Reservoir Filling complete\", \"time\": \"%s\"}\r\n", global_state.datestring);
      gpio_outputs[GPIO_OUTPUT_FILL_PUMP].run_for_ms = 0;
      break;

    case DRAIN_CYCLE_DRAINING:
      if (!drain_time) drain_time = TM_Time;

      if (TM_Time < (drain_time + misc_settings.flow_sensor_lag))
        break;

      gpio_outputs[GPIO_OUTPUT_DRAIN_PUMP].run_for_ms = 5000;
      TM_PWMIN_Get(&PWMIN1_Data);
      if (PWMIN1_Data.Frequency > 0) {
        print_pwmin(PWMIN_RES_DRAIN, PWMIN1_Data.Frequency);
      } else {
        global_state.reservoir_state = DRAIN_CYCLE_EMPTY;
        gpio_outputs[GPIO_OUTPUT_DRAIN_PUMP].run_for_ms = 0;
        sprintf(buf, "{\"event\": \"Reservoir Draining complete\", \"time\": \"%s\"}\r\n", global_state.datestring);
      }
      drain_time = TM_Time;
      break;

    case DRAIN_CYCLE_EMPTY:
      global_state.reservoir_state = DRAIN_CYCLE_FILLING;
      sprintf(buf, "{\"event\": \"Reservoir Filling started\", \"time\": \"%s\"}\r\n", global_state.datestring);
      gpio_outputs[GPIO_OUTPUT_FILL_PUMP].run_for_ms = 5000;
      break;

    case MANUAL_DRAIN:
      gpio_outputs[GPIO_OUTPUT_DRAIN_PUMP].run_for_ms = 1000;
      break;

    case MANUAL_FILL:
      gpio_outputs[GPIO_OUTPUT_FILL_PUMP].run_for_ms = 1000;
      break;

    case LEVEL_ERROR:
      global_state.drain_cycle_active = 0;
      global_state.reservoir_state = EMERGENCY_STOP;
      emergency_stop(0);
      sprintf(buf, "{\"error\": \"Impossible Level Switch Reading\", \"time\": \"%s\"}\r\n", global_state.datestring);
      break;

    case EMERGENCY_STOP:
      break;

  }

  last_min_state = global_state.reservoir_min;
  TM_USART_Puts(USART2, buf);

}

void nutrient_pump_ctrl(void){
  char buf[MAX_STR_LEN];
  memset(buf, 0, sizeof(buf));

  if (global_state.stirring_nutrients){
    if (gpio_outputs[GPIO_OUTPUT_STIRRER_MOTORS].desired_state == 0){
      gpio_outputs[GPIO_OUTPUT_STIRRER_MOTORS].desired_state = 1;
      gpio_outputs[GPIO_OUTPUT_STIRRER_MOTORS].run_for_ms = 0xffffffff;
      sprintf(buf, "{\"event\": \"%s turned on\", \"time\": \"%s\"}\r\n", gpio_output_names[GPIO_OUTPUT_STIRRER_MOTORS], global_state.datestring);
      TM_USART_Puts(USART2, buf);
    }

  }else if (global_state.adding_nutrients){
    if (gpio_outputs[GPIO_OUTPUT_STIRRER_MOTORS].desired_state == 1){
      gpio_outputs[GPIO_OUTPUT_STIRRER_MOTORS].desired_state = 0;
      sprintf(buf, "{\"event\": \"%s turned off\", \"time\": \"%s\"}\r\n", gpio_output_names[GPIO_OUTPUT_STIRRER_MOTORS], global_state.datestring);
      TM_USART_Puts(USART2, buf);
    }

    static uint32_t next_time = 0;
    if (!next_time) next_time = TM_Time + misc_settings.nutrient_pause_ms;

    if (TM_Time >= next_time) {
      static uint8_t i = 0;

      if (i < NUM_NUTRIENT_PUMPS) {
        float liters_added = 0;

        if ((global_state.reservoir_state == DRAIN_CYCLE_NUTRIENTS) || (global_state.reservoir_state == DRAIN_CYCLE_FILLING)){
          if (misc_settings.fill_to_alarm_level)
            liters_added = misc_settings.res_liters_alarm;
          else
            liters_added = misc_settings.res_liters_max;
        }else if ((global_state.reservoir_state == NORMAL_NUTRIENTS) || (global_state.reservoir_state == NORMAL_FILLING)){
          if (misc_settings.fill_to_alarm_level)
            liters_added = misc_settings.res_liters_alarm - misc_settings.res_liters_min;
          else
            liters_added = misc_settings.res_liters_max - misc_settings.res_liters_min;
        }

        float dosage_ml = (nutrient_pumps[i].ml_per_10l * misc_settings.nutrient_factor) / 10.0f * liters_added;
        uint8_t gpio_out = nutrient_pumps[i].gpio_output;

        gpio_outputs[gpio_out].run_for_ms = dosage_ml * nutrient_pumps[i].ms_per_ml;

        sprintf(buf, "{\"event\": \"%s turned on for %d ms\", \"time\": \"%s\"}\r\n", gpio_output_names[gpio_out], gpio_outputs[gpio_out].run_for_ms, global_state.datestring);
        TM_USART_Puts(USART2, buf);

        next_time = TM_Time + (dosage_ml * nutrient_pumps[i].ms_per_ml) + misc_settings.nutrient_pause_ms;
        i++;

      } else {
        if ((global_state.reservoir_state == DRAIN_CYCLE_NUTRIENTS)
        || (global_state.reservoir_state == NORMAL_NUTRIENTS))
          global_state.reservoir_state = NORMAL_IDLE;

        global_state.adding_nutrients = 0;
        global_state.nutrients_done = 1;
        i = 0;
      }

    }
  }
}

void sewage_pump_ctrl(void){
  char buf[MAX_STR_LEN];
  memset(buf, 0, sizeof(buf));

  global_state.sewage_tank_empty = GPIO_ReadInputDataBit(irqs[SWITCH_SEWAGE_MIN].gpio_port, irqs[SWITCH_SEWAGE_MIN].gpio_pin);
  global_state.sewage_tank_full = (!GPIO_ReadInputDataBit(irqs[SWITCH_SEWAGE_MAX].gpio_port, irqs[SWITCH_SEWAGE_MAX].gpio_pin));

  if (!global_state.sewage_tank_empty && !global_state.sewage_pump_blocked){
    static uint32_t last_time = 0;
    if (!last_time) last_time = TM_Time;

    if ((TM_Time >= (last_time + (misc_settings.sewage_pump_pause_s * 1000) + (misc_settings.sewage_pump_run_s * 1000))) || ((TM_Time > last_time) && (global_state.sewage_tank_full))) {
      gpio_outputs[GPIO_OUTPUT_SEWAGE_PUMP].run_for_ms = misc_settings.sewage_pump_run_s * 1000;
      sprintf(buf, "{\"event\": \"%s turned on for %d ms\", \"time\": \"%s\"}\r\n", gpio_output_names[GPIO_OUTPUT_SEWAGE_PUMP], gpio_outputs[GPIO_OUTPUT_SEWAGE_PUMP].run_for_ms, global_state.datestring);
      TM_USART_Puts(USART2, buf);
      last_time = TM_Time;
    }
  }
}

void ph_ctrl(void){
  char buf[MAX_STR_LEN];
  memset(buf, 0, sizeof(buf));

  float liters_in_res = 0;
  float ph_delta = 0;
  float ml_to_add = 0;
  uint32_t ms_to_run = 0;
  static uint32_t next_time = 0;

  if (global_state.reservoir_state == NORMAL_IDLE){
    if (!next_time) next_time = TM_Time + (misc_settings.res_settling_time_s * 1000);
    if (TM_Time >= next_time) {
      if (adc_ph.value >= ph_setpoints.max_ph) {
        ph_delta = adc_ph.value - ph_setpoints.min_ph;

        if (global_state.reservoir_alarm)
          liters_in_res = misc_settings.res_liters_alarm;
        else if (global_state.reservoir_min)
          liters_in_res = misc_settings.res_liters_max;
        else if (global_state.reservoir_min)
          liters_in_res = misc_settings.res_liters_min;
        else
          liters_in_res = misc_settings.res_liters_max - ((misc_settings.res_liters_max - misc_settings.res_liters_min) / 2);

        ml_to_add = ph_delta * (ph_setpoints.ml_per_ph_per_10l / 10.0f * liters_in_res);
        ms_to_run = ml_to_add * ph_setpoints.ms_per_ml;
        global_state.adjusting_ph = 1;
        gpio_outputs[GPIO_OUTPUT_PHDOWN_PUMP].run_for_ms = ms_to_run;

        sprintf(buf, "{\"info\": \"ph_down\", \"amount_ml\": %1.2f, \"liters_in_res\": %1.2f, \"ph_delta\": %1.2f.\", \"ts\": %d, \"time\": \"%s\"}\r\n", ml_to_add, liters_in_res, ph_delta, TM_Time, global_state.datestring);
        TM_USART_Puts(USART2, buf);

        sprintf(buf, "{\"event\": \"%s turned on for %d ms\", \"time\": \"%s\"}\r\n", gpio_output_names[GPIO_OUTPUT_PHDOWN_PUMP], gpio_outputs[GPIO_OUTPUT_PHDOWN_PUMP].run_for_ms, global_state.datestring);
        TM_USART_Puts(USART2, buf);
      } else {
        gpio_outputs[GPIO_OUTPUT_PHDOWN_PUMP].run_for_ms = 0;
        global_state.adjusting_ph = 0;
        global_state.reservoir_state = NORMAL_IDLE;
      }
      next_time = TM_Time + (misc_settings.res_settling_time_s * 1000) + ms_to_run;
    }
  }

  if  ((gpio_outputs[GPIO_OUTPUT_PHDOWN_PUMP].run_for_ms == 0) && (global_state.adjusting_ph)){
    sprintf(buf, "{\"event\": \"%s turned off\", \"time\": \"%s\"}\r\n", gpio_output_names[GPIO_OUTPUT_PHDOWN_PUMP], global_state.datestring);
    TM_USART_Puts(USART2, buf);
    global_state.adjusting_ph = 0;
    global_state.reservoir_state = NORMAL_IDLE;
  }
}

void gpio_ctrl(void){
  uint8_t i;
  static uint32_t last_time = 0;
  if (last_time == 0) last_time = TM_Time;

  // Schedule outputs to run for a specific time
  if ((TM_Time - last_time) >= 1) {
    for (i=0; i<NUM_GPIO_OUTPUTS; i++) {
      uint32_t run_for_ms = gpio_outputs[i].run_for_ms;
      if (run_for_ms != 0xffffffff) {
        if (run_for_ms > 0) {
          gpio_outputs[i].desired_state = 1;
          gpio_outputs[i].run_for_ms -= (TM_Time - last_time);
          if (gpio_outputs[i].run_for_ms > run_for_ms)
            gpio_outputs[i].run_for_ms = 0;
        } else {
          gpio_outputs[i].desired_state = 0;
        }
      }
    }
    last_time = TM_Time;
  }


  // Drive the outputs
  for (i=0; i<NUM_GPIO_OUTPUTS; i++) {

    // Sanity check if the pumps are safe to run
    // Break on safe conditions, otherwise fall through and disable the output
    switch(i){
      case GPIO_OUTPUT_FILL_PUMP:
        if ((!global_state.reservoir_alarm) && (misc_settings.fill_to_alarm_level))
          break;
        else if (!global_state.reservoir_max)
          break;

      case GPIO_OUTPUT_DRAIN_PUMP:
        if (!global_state.sewage_tank_full)
          break;

      case GPIO_OUTPUT_DEHUMI_PUMP:
        if (!global_state.sewage_tank_full)
          break;

      case GPIO_OUTPUT_SEWAGE_PUMP:
        if ((!global_state.sewage_pump_blocked) && (!global_state.sewage_tank_empty))
          break;

      case GPIO_OUTPUT_COOLANT_PUMP:
        break;

      case GPIO_OUTPUT_PHDOWN_PUMP:
        break;

      case GPIO_OUTPUT_NUTRIENT1_PUMP:
        break;

      case GPIO_OUTPUT_NUTRIENT2_PUMP:
        break;

      case GPIO_OUTPUT_NUTRIENT3_PUMP:
        break;

      case GPIO_OUTPUT_DEEP_RED_LEDS:
        break;

      case GPIO_OUTPUT_STIRRER_MOTORS:
        break;

      default:
        gpio_outputs[i].desired_state = 0;
    }

    // finally set the actual GPIO if we are not in emergency stop mode
    if (global_state.reservoir_state != EMERGENCY_STOP)
      GPIO_WriteBit(gpio_outputs[i].gpio_port, gpio_outputs[i].gpio_pin, gpio_outputs[i].desired_state);
    else
      GPIO_WriteBit(gpio_outputs[i].gpio_port, gpio_outputs[i].gpio_pin, 0);
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
    char buf[MAX_STR_LEN];
    memset(buf, 0, sizeof(buf));
    sprintf(buf, "{\"event\": \"Exhaust turned %s\", \"time\": \"%s\"}\r\n", desired_state ? "on" : "off", global_state.datestring);
    TM_USART_Puts(USART2, buf);
  }
}

void res_temp_ctrl(void){

  uint8_t current_state = gpio_outputs[GPIO_OUTPUT_COOLANT_PUMP].desired_state;
  uint8_t desired_state = current_state;

  if (ds18b20_sensors[TEMP_RES].value > coolant_setpoints.max_temp)
    desired_state = 100;

  if (ds18b20_sensors[TEMP_RES].value < coolant_setpoints.min_temp)
    desired_state = 0;

  if (desired_state != current_state) {
    gpio_outputs[GPIO_OUTPUT_COOLANT_PUMP].desired_state = desired_state;
    gpio_outputs[GPIO_OUTPUT_COOLANT_PUMP].run_for_ms = 0xffffffff;
    char buf[MAX_STR_LEN];
    memset(buf, 0, sizeof(buf));
    sprintf(buf, "{\"event\": \"Coolant pump turned %s\", \"time\": \"%s\"}\r\n", desired_state ? "on" : "off", global_state.datestring);
    TM_USART_Puts(USART2, buf);
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
    char buf[MAX_STR_LEN];
    memset(buf, 0, sizeof(buf));
    sprintf(buf, "{\"event\": \"Main Light turned %s\", \"time\": \"%s\"}\r\n", desired_state ? "on" : "off", global_state.datestring);
    TM_USART_Puts(USART2, buf);
  }
}

void print_env(void) {
  char buf[MAX_STR_LEN];
  memset(buf, 0, sizeof(buf));

  TM_USART_Puts(USART2, "{\"name\":\"Sensors\",\"content\":[\r\n");
  sprintf(buf, "\t{\"name\":\"%s\",\"ds_name\":\"%s\", \"value\":%.2f, \"unit\":\"%s\"},\r\n\t{\"name\":\"%s\",\"ds_name\":\"%s\", \"value\":%.2f, \"unit\":\"%s\"}, \r\n\t{\"name\":\"%s\",\"ds_name\":\"%s\", \"value\":%.2f, \"unit\":\"%s\"},\r\n",
    bme280_temp.name,
    sensor_names[TEMP_MAIN][1],
    bme280_temp.value,
    unit_names[bme280_temp.unit],
    bme280_humi.name,
    sensor_names[HUMI_MAIN][1],
    bme280_humi.value,
    unit_names[bme280_humi.unit],
    bme280_press.name,
    sensor_names[PRES_MAIN][1],
    bme280_press.value,
    unit_names[bme280_press.unit]
  );
  TM_USART_Puts(USART2, buf);

  uint8_t i;
  for (i=0; i< DS18B20_NUM_DEVICES; i++) {
    sprintf(buf, "\t{\"name\":\"%s\",\"ds_name\":\"%s\",\"value\":%.2f, \"unit\":\"%s\"},\r\n",
      ds18b20_sensors[i].name,
      sensor_names[i][1],
      ds18b20_sensors[i].value,
      unit_names[DEG_C]
    );
    TM_USART_Puts(USART2, buf);
  }

  sprintf(buf, "\t{\"name\":\"%s\",\"ds_name\": \"%s\", \"value\":%1.2f},\r\n", sensor_names[ADC_PH][0], sensor_names[ADC_PH][1], adc_ph.value);
  TM_USART_Puts(USART2, buf);

  sprintf(buf, "\t{\"name\":\"%s\",\"ds_name\": \"%s\", \"value\":%1.2f}\r\n", sensor_names[ADC_EC][0], sensor_names[ADC_EC][1], adc_ec.value);
  TM_USART_Puts(USART2, buf);

  TM_USART_Puts(USART2, "]}\r\n");
}

void print_light(void) {
  char buf[MAX_STR_LEN];
  memset(buf, 0, sizeof(buf));

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
  memset(buf, 0, sizeof(buf));

  coolant_setpoints.state = gpio_outputs[GPIO_OUTPUT_COOLANT_PUMP].desired_state;
  sprintf(buf, "{\"name\":\"Coolant Control\",\"content\":{\r\n\t\"max_temp\":%.2f,\r\n\t\"min_temp\":%.2f,\r\n\t\"state\":\"%s\"\r\n}}\r\n",
    coolant_setpoints.max_temp,
    coolant_setpoints.min_temp,
    coolant_setpoints.state ? "On" : "Off"
  );
  TM_USART_Puts(USART2, buf);

}

void print_exhaust(void) {
  char buf[MAX_STR_LEN];
  memset(buf, 0, sizeof(buf));

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
  memset(buf, 0, sizeof(buf));

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
  memset(buf, 0, sizeof(buf));
  uint8_t i;

  TM_USART_Puts(USART2, "{\"name\":\"Errors\",\"content\":[\r\n");
  
  for (i=0; i<DS18B20_NUM_DEVICES; i++) {
    
    sprintf((char *) buf, "\t{\"name\":\"DS18B20 %s\", \"count\":%d},\r\n", ds18b20_sensors[i].name, ds18b20_sensors[i].error_count);
    TM_USART_Puts(USART2, buf);
  }

  sprintf((char *) buf, "\t{\"name\":\"I2C Resets\", \"count\":\"%d\"},\r\n", global_state.i2c_restarts);
  TM_USART_Puts(USART2, buf);
  sprintf((char *) buf, "\t{\"name\":\"Restarted by Watchdog\", \"value\":\"%s\"}\r\n", watchdog_barked ? "yes" : "no");
  TM_USART_Puts(USART2, buf);
  TM_USART_Puts(USART2, "]}\r\n");

}

void reset_errors(void) {
  uint8_t i;
  for (i=0; i<DS18B20_NUM_DEVICES; i++) {
    ds18b20_sensors[i].error_count = 0;
  }
}

void print_gpio_outputs(void){
  char buf[MAX_STR_LEN];
  memset(buf, 0, sizeof(buf));
  uint8_t i;

  TM_USART_Puts(USART2, "{\"name\":\"GPIO Output Channels\",\"content\":[\r\n");
  
  for (i=0; i<NUM_GPIO_OUTPUTS; i++) {

    uint8_t pinstate = GPIO_ReadInputDataBit(gpio_outputs[i].gpio_port, gpio_outputs[i].gpio_pin);
    sprintf(buf, "\t{\"name\":\"%s\", \"state\":%d, \"run for ms\":%d}", gpio_output_names[i], pinstate, gpio_outputs[i].run_for_ms);
    TM_USART_Puts(USART2, buf);
    if (i == NUM_GPIO_OUTPUTS -1)
      TM_USART_Puts(USART2, "\r\n");
    else
      TM_USART_Puts(USART2, ",\r\n");
  }
  TM_USART_Puts(USART2, "]}\r\n");

}


void print_nutrients(void){
  char buf[MAX_STR_LEN];
  memset(buf, 0, sizeof(buf));
  uint8_t i;

  TM_USART_Puts(USART2, "{\"name\":\"Nutrient Pump Settings\",\"content\":[\r\n");
  
  for (i=0; i<NUM_NUTRIENT_PUMPS; i++) {

    
    sprintf(buf, "\t{\"name\":\"%s\", \"ms_per_ml\":%d,\"ml_per_10l\":%1.2f}", nutrient_pumps[i].name, nutrient_pumps[i].ms_per_ml, nutrient_pumps[i].ml_per_10l);
    TM_USART_Puts(USART2, buf);
    if (i == NUM_GPIO_OUTPUTS -1)
      TM_USART_Puts(USART2, "\r\n");
    else
      TM_USART_Puts(USART2, ",\r\n");
  }
  TM_USART_Puts(USART2, "]}\r\n");

}


void print_capsense(void){
  char buf[MAX_STR_LEN];
  memset(buf, 0, sizeof(buf));
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
  memset(buf, 0, sizeof(buf));
  sprintf(buf, "{\"name\": \"PWM Input\", \"id\":%d, \"frequency\":%7.2f}\r\n", id, freq);
  TM_USART_Puts(USART2, buf);

}


void print_irqs(void){
  char buf[MAX_STR_LEN];
  memset(buf, 0, sizeof(buf));
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
  memset(buf, 0, sizeof(buf));
  sprintf(buf, "{\"name\": \"%s\", \"value\":%1.2f}\r\n", sensor_names[ADC_PH][0], adc_ph.value);
  TM_USART_Puts(USART2, buf);
  sprintf(buf, "{\"name\": \"pH Adjustment Settings\", \"min_ph\":%1.2f, \"max_ph\":%1.2f, \"ms_per_ml\":%d, \"ml_per_ph_per_10l\":%1.2f}\r\n", ph_setpoints.min_ph, ph_setpoints.max_ph, ph_setpoints.ms_per_ml, ph_setpoints.ml_per_ph_per_10l);
  TM_USART_Puts(USART2, buf);
}


void print_ec(void){
  char buf[MAX_STR_LEN];
  memset(buf, 0, sizeof(buf));
  sprintf(buf, "{\"name\": \"%s\", \"value\":%1.4f}\r\n", sensor_names[ADC_EC][0], adc_ec.value);
  TM_USART_Puts(USART2, buf);
}


void print_settings(void){
  char buf[MAX_STR_LEN];
  memset(buf, 0, sizeof(buf));
  uint8_t i;

  sprintf(buf, "{\"name\":\"Misc. Settings\",\"content\":{\r\n\t\
\"res_liters_min\":%1.2f,\r\n\t\
\"res_liters_max\":%1.2f,\r\n\t\
\"res_liters_alarm\":%1.2f,\r\n\t\
\"nutrient_factor\":%1.2f,\r\n\t\
\"nutrient_pause_ms\":%1.2f,\r\n\t\
\"ec_k\":%1.4f,\r\n\t\
\"ec_temp_coef\":%1.4f,\r\n\t\
\"ec_r1_ohms\":%d,\r\n\t\
\"ec_ra_ohms\":%d,\r\n\t\
\"ec_read_interval_s\":%d,\r\n\t\
\"ph4_ph\":%1.4f,\r\n\t\
\"ph7_ph\":%1.4f,\r\n\t\
\"ph4_v\":%1.4f,\r\n\t\
\"ph7_v\":%1.4f,\r\n\t\
\"vcc_v\":%1.4f,\r\n\t\
\"res_settling_time\":%d,\r\n\t\
\"sewage_pump_pause_s\":%d,\r\n\t\
\"sewage_pump_run_s\":%d,\r\n\t\
\"fill_to_alarm_level\":%d}\r\n\
}\r\n",
    misc_settings.res_liters_min,
    misc_settings.res_liters_max,
    misc_settings.res_liters_alarm,
    misc_settings.nutrient_factor,
    misc_settings.nutrient_pause_ms,
    misc_settings.ec_k,
    misc_settings.ec_temp_coef,
    misc_settings.ec_r1_ohms,
    misc_settings.ec_ra_ohms,
    misc_settings.ec_read_interval_s,
    misc_settings.ph4_ph,
    misc_settings.ph7_ph,
    misc_settings.ph4_v,
    misc_settings.ph7_v,
    misc_settings.vcc_v,
    misc_settings.res_settling_time_s,
    misc_settings.sewage_pump_pause_s,
    misc_settings.sewage_pump_run_s,
    misc_settings.fill_to_alarm_level
  );
  TM_USART_Puts(USART2, buf);
}



void print_state(void){
  char buf[MAX_STR_LEN];
  memset(buf, 0, sizeof(buf));
  uint8_t i;

  sprintf(buf, "{\"name\":\"Global State\",\"content\":{\r\n\t\
\"sewage_pump_blocked\":%d,\r\n\t\
\"sewage_tank_full\":%d,\r\n\t\
\"sewage_tank_empty\":%d,\r\n\t\
\"drain_cycle_active\":%d,\r\n\t\
\"adjusting_ph\":%d,\r\n\t\
\"stirring_nutrients\":%d,\r\n\t\
\"adding_nutrients\":%d,\r\n\t\
\"nutrients_done\":%d,\r\n\t\
\"reservoir_alarm\":%d,\r\n\t\
\"reservoir_max\":%d,\r\n\t\
\"reservoir_min\":%d,\r\n\t\
\"water_tank_empty\":%d,\r\n\t\
\"reservoir_state\":\"%s\"}\r\n\
}\r\n",
    global_state.sewage_pump_blocked,
    global_state.sewage_tank_full,
    global_state.sewage_tank_empty,
    global_state.drain_cycle_active,
    global_state.adjusting_ph,
    global_state.stirring_nutrients,
    global_state.adding_nutrients,
    global_state.nutrients_done,
    global_state.reservoir_alarm,
    global_state.reservoir_max,
    global_state.reservoir_min,
    global_state.water_tank_empty,
    res_state_names[global_state.reservoir_state]
  );
  TM_USART_Puts(USART2, buf);
}

void delay_ms(__IO uint32_t ms)
{
  uint32_t ncount = 6736;
  while (ms--){
    while(ncount--);
  }
}
