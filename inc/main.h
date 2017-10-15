#ifndef _MAIN_H_
#define _MAIN_H_


#define MAX_SENSOR_NAME_LENGTH  64
#define MAX_OUTPUT_NAME_LENGTH  64
#define NUM_SENSORS 7
#define NUM_NUTRIENT_PUMPS 3
#define NUM_RES_STATES 14
#define NUM_UNITS 8
#define MAX_UNIT_NAME_LENGTH 9
#define MAX_STR_LEN 2048
#define V_IN 3.3F
#define FLASH_SIZE 2048

#define ERROR_SEWAGE_TANK_FULL -1

void print_env(void);
void print_errors(void);
void print_light(void);
void print_exhaust(void);
void print_relays(void);
void print_gpio_outputs(void);
void print_coolant(void);
void print_capsense(void);
void print_pwmin(uint8_t id, float freq);
void print_irqs(void);
void print_ec(void);
void print_ph(void);
void print_settings(void);
void print_state(void);
void print_nutrients(void);

void delay_ms(__IO uint32_t ms);
void set_defaults(void);
void emergency_stop(uint8_t release);
void gpio_check(void);
void reservoir_level_ctrl(void);
void reset_errors(void);
void ph_ctrl(void);
void gpio_ctrl(void);
void res_temp_ctrl(void);
void sewage_pump_ctrl(void);
void nutrient_pump_ctrl(void);
void exhaust_ctrl(void);
void light_scheduler(void);

extern output_relay_struct_t relays[NUM_RELAYS];
extern gpio_output_struct_t gpio_outputs[NUM_GPIO_OUTPUTS];
extern irq_switch_struct_t irqs[NUM_IRQ_PINS];
extern char irq_input_names[NUM_IRQ_PINS][2][MAX_SENSOR_NAME_LENGTH + 1];
extern char res_state_names[NUM_RES_STATES][2][MAX_SENSOR_NAME_LENGTH + 1];
extern char sensor_names[NUM_SENSORS][2][MAX_SENSOR_NAME_LENGTH + 1];
extern char gpio_output_names[NUM_GPIO_OUTPUTS][2][MAX_OUTPUT_NAME_LENGTH + 1];
extern char unit_names[NUM_UNITS][MAX_UNIT_NAME_LENGTH + 1];

extern struct bme280_t bme280_1;

typedef enum resservoir_state {
  DRAIN_CYCLE_DRAINING,
  DRAIN_CYCLE_EMPTY,
  DRAIN_CYCLE_FILLING,
  DRAIN_CYCLE_FULL,
  DRAIN_CYCLE_NUTRIENTS,
  NORMAL_MIN,
  NORMAL_FILLING,
  NORMAL_MAX,
  NORMAL_NUTRIENTS,
  NORMAL_IDLE,
  MANUAL_DRAIN,
  MANUAL_FILL,
  LEVEL_ERROR,
  EMERGENCY_STOP,
} res_states_t;


// list onewire sensors first!
typedef enum names {
  TEMP_STORAGE,
  TEMP_RES,
  TEMP_MAIN,
  HUMI_MAIN,
  PRES_MAIN,
  ADC_PH,
  ADC_EC,
} sensor_names_enum_t;

typedef enum units {
  DEG_C,
  MBAR,
  PERCENT,
  LITER,
  PH,
  MS_CM,
  VOLT,
  AMP
} unit_t;

typedef struct sensor {
  char * name;
  unit_t unit;
  float value;
  uint16_t error_count;
} sensor_t;


extern sensor_t bme280_temp;
extern sensor_t bme280_humi;
extern sensor_t bme280_press;
extern sensor_t adc_ph;
extern sensor_t adc_ec;


typedef struct light_timer {
  uint8_t on_hour;
  uint8_t on_minutes;
  uint8_t off_hour;
  uint8_t off_minutes;
  uint8_t state;
  output_relay_struct_t * gpio_output;
  uint8_t pad[3];
}light_timer_struct_t ;

light_timer_struct_t light_timer;

typedef struct coolant_setpoints {
  float max_temp;
  float min_temp;
  uint8_t state;
  gpio_output_struct_t * gpio_output;
  uint8_t pad[3];
}coolant_setpoints_struct_t ;

coolant_setpoints_struct_t coolant_setpoints;

typedef struct exhaust_setpoints {
  float max_temp;
  float max_humi;
  float min_temp;
  float min_humi;
  uint8_t state;
  output_relay_struct_t * gpio_output;
  uint8_t pad[3];
}exhaust_setpoints_struct_t ;

exhaust_setpoints_struct_t exhaust_setpoints;

typedef struct ph_setpoints {
  float min_ph;
  float max_ph;
  float ml_per_ph_per_10l;
  float ml_in_res;
  uint32_t ms_per_ml;
}ph_setpoints_struct_t;

ph_setpoints_struct_t ph_setpoints;

typedef struct nutrient_pump {
  uint32_t ms_per_ml;
  float ml_per_10l;
  float ml_in_res;
  uint8_t gpio_output;
  char* name;
} nutrient_pump_struct_t;

nutrient_pump_struct_t nutrient_pumps[NUM_NUTRIENT_PUMPS];


typedef struct misc_settings {
  float ec_k;
  float ec_temp_coef;

  uint16_t ec_r1_ohms;
  uint16_t ec_ra_ohms;
  uint32_t ec_read_interval_s;

  float ph4_ph;
  float ph7_ph;
  float ph4_v;
  float ph7_v;
  float vcc_v;

  float nutrient_factor;
  float res_liters_min;
  float res_liters_max;
  float res_liters_alarm;
  uint32_t res_settling_time_s;
  uint32_t sewage_pump_pause_s;
  uint32_t sewage_pump_run_s;
  uint32_t nutrient_pause_s;
  uint32_t nutrient_stirring_s;

  uint8_t fill_to_alarm_level   : 1;
  uint8_t i2c_break_enabled     : 1;
  uint8_t                       : 6;
  uint8_t pad[3];

  uint32_t i2c_timeout;
  uint32_t i2c_max_restarts;
  uint32_t i2c_max_reading_age_s;
  uint32_t flow_sensor_lag;
}misc_settings_struct_t;

misc_settings_struct_t misc_settings;


typedef struct global_state {
  uint8_t sewage_pump_blocked   : 1;
  uint8_t sewage_tank_full      : 1;
  uint8_t sewage_tank_empty     : 1;
  uint8_t drain_cycle_active    : 1;
  uint8_t adjusting_ph          : 1;
  uint8_t adding_nutrients      : 1;
  uint8_t stirring_nutrients    : 1;
  uint8_t nutrients_done        : 1;

  uint8_t reservoir_alarm       : 1;
  uint8_t reservoir_max         : 1;
  uint8_t reservoir_min         : 1;
  uint8_t water_tank_empty      : 1;
  uint8_t                       : 4;

  uint8_t active_dosing_pump_gpio;

  uint32_t system_uptime;
  uint32_t i2c_errors;
  uint32_t i2c_restarts;
  uint32_t i2c_last_good_reading;
  uint32_t i2c_max_reading_age_s;
  res_states_t reservoir_state;
  char* datestring;
}global_state_struct_t;

extern volatile global_state_struct_t global_state;

#endif
