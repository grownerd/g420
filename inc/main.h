#ifndef _MAIN_H_
#define _MAIN_H_


#define MAX_SENSOR_NAME_LENGTH  64
#define MAX_OUTPUT_NAME_LENGTH  64
#define NUM_SENSORS 7
#define NUM_UNITS 8
#define MAX_UNIT_NAME_LENGTH 9
#define MAX_STR_LEN 1024
#define V_IN 3.3F

#define ERROR_SEWAGE_TANK_FULL -1

void print_env(void);
void print_errors(void);
void print_light(void);
void print_exhaust(void);
void print_relays(void);
void print_pwm(void);
void print_coolant(void);
void print_capsense(void);
void print_pwmin(uint8_t id, float freq);
void print_irqs(void);
void print_ec(void);
void print_ph(void);
void print_settings(void);

void emergency_stop(uint8_t release);
void reservoir_drain_cycle_ctrl();
void reset_errors(void);
void ph_ctrl(void);
void pwm_ctrl(void);
void res_temp_ctrl(void);
void sewage_pump_ctrl(void);
void nutrient_pump_ctrl(void);
void exhaust_ctrl(void);
void light_scheduler(void);
void pwm_scheduler(void);

extern output_relay_struct_t relays[NUM_RELAYS];
extern output_pwm_struct_t pwms[NUM_PWM_OUTPUTS];
extern irq_switch_struct_t irqs[NUM_IRQ_PINS];
extern char irq_input_names[NUM_IRQ_PINS][MAX_SENSOR_NAME_LENGTH + 1];
extern char sensor_names[NUM_SENSORS][MAX_SENSOR_NAME_LENGTH + 1];
extern char pwm_output_names[NUM_PWM_OUTPUTS][MAX_OUTPUT_NAME_LENGTH + 1];
extern char unit_names[NUM_UNITS][MAX_UNIT_NAME_LENGTH + 1];


typedef enum resservoir_state {
  DRAIN_CYCLE_DRAINING,
  DRAIN_CYCLE_EMPTY,
  DRAIN_CYCLE_FILLING,
  DRAIN_CYCLE_FULL,
  DRAIN_CYCLE_NUTRIENTS,
  DRAIN_CYCLE_PHDOWN,
  NORMAL_MAX,
  NORMAL_FILLING,
  NORMAL_MIN,
  NORMAL_IDLE,
  NORMAL_PHDOWN,
} res_states_t;


// list onewire sensors first!
typedef enum names {
  TEMP_RES,
  TEMP_STORAGE,
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
  S_CM,
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
  output_pwm_struct_t * pwm_output;
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
  uint32_t ms_per_ml;
  uint32_t ml_per_ph;
}ph_setpoints_struct_t;

ph_setpoints_struct_t ph_setpoints;

typedef struct nutrient_pump {
  uint32_t ms_per_ml;
  float dosage_ml;
  uint8_t pwm_output;
  char* name;
} nutrient_pump_struct_t;

nutrient_pump_struct_t nutrient_pumps[3];

typedef struct misc_settings {
  float ec_k;
  float ec_temp_coef;
  uint16_t ec_r1_ohms;
  uint16_t ec_ra_ohms;
  uint16_t ph_cal401_mv;
  uint16_t ph_cal686_mv;
  uint32_t res_settling_time_s;
  uint32_t sewage_pump_pause_s;
  uint32_t sewage_pump_run_s;
  uint8_t fill_to_alarm_level   : 1;
  uint8_t                       : 7;
  uint8_t pad[3];
}misc_settings_struct_t;

misc_settings_struct_t misc_settings;

typedef struct global_state {
  uint8_t sewage_pump_blocked   : 1;
  uint8_t sewage_tank_empty     : 1;
  uint8_t drain_cycle_active    : 1;
  uint8_t adjusting_ph          : 1;
  uint8_t adding_nutrients      : 1;
  uint8_t                       : 3;
  res_states_t reservoir_state;
  char* datestring;
}global_state_struct_t;

extern volatile global_state_struct_t global_state;

#endif
