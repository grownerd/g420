
#ifndef GPIO_H
#define GPIO_H

/**
 * CONSTANTS
 */

#define NUM_IRQ_PINS 16
#define NUM_RELAYS 3
#define NUM_PWM_OUTPUTS 11

#define RELAY_LIGHT 0
#define RELAY_EXHAUST 1
#define RELAY_AUX 2

#define PWM_FILL_PUMP 0
#define PWM_DRAIN_PUMP 1
#define PWM_COOLANT_PUMP 2
#define PWM_SEWAGE_PUMP 3
#define PWM_DEHUMI_PUMP 4
#define PWM_PHDOWN_PUMP 5
#define PWM_NUTRIENT1_PUMP 6
#define PWM_NUTRIENT2_PUMP 7
#define PWM_NUTRIENT3_PUMP 8
#define PWM_DEEP_RED_LEDS 9
#define PWM_STIRRER_MOTORS 10

#define SWITCH_BLUE_BUTTON 0
#define SWITCH_NC_1 1
#define SWITCH_RES_MAX 2
#define SWITCH_NC_3 3
#define SWITCH_RES_MIN 4
#define SWITCH_RES_ALARM 5
#define SWITCH_UNUSED_6 6
#define SWITCH_DEHUMI_MAX 7
#define SWITCH_DEHUMI_MIN 8
#define SWITCH_NC_9 9
#define SWITCH_SEWAGE_MAX 10
#define SWITCH_SEWAGE_MIN 11
#define SWITCH_UNUSED_12 12
#define SWITCH_UNUSED_13 13
#define SWITCH_NC_14 14
#define SWITCH_WATER_EMPTY 15

#define PWMIN_RES_DRAIN 0
#define PWMIN_DEHUMI_DRAIN 1

#define OFF 0
#define ON 1
#define TOGGLE 2

#define SWITCH_TYPE_NO 0
#define SWITCH_TYPE_NC 1

#define EC_GPIO_PORT GPIOD
#define EC_GPIO_PIN GPIO_Pin_8

/**
 * PROTOTYPES
 */

typedef struct output_relay {
  uint16_t gpio_pin;
  GPIO_TypeDef *gpio_port;
}output_relay_struct_t ;

typedef struct output_pwm {
  TM_PWM_TIM_t * tim_data;
  TM_PWM_Channel_t pwm_channel;
  uint8_t duty_percent;
  uint32_t run_for_ms;
}output_pwm_struct_t ;


#if 1
typedef struct irq_switch{
  GPIO_TypeDef* gpio_port;
  uint16_t gpio_pin;
  uint8_t type            : 1; 
  uint8_t idle_state      : 1; 
  uint8_t last_state      : 1; 
  uint8_t current_state   : 1; 
  uint8_t                 : 4; 
} irq_switch_struct_t ;
#else
extern GPIO_TypeDef* irq_ports[NUM_IRQ_PINS];
#endif

extern TM_PWM_TIM_t TIM1_Data, TIM3_Data;
extern TM_PWMIN_t PWMIN1_Data, PWMIN2_Data;

void exti_init(void);
void gpio_init(void);
void pwm_init(void);
void pwmin_init(void);
void switch_relay(output_relay_struct_t * relay, uint8_t action);
void reservoir_level_irq_handler(uint16_t GPIO_Pin);
void sewage_level_irq_handler(uint16_t GPIO_Pin);
void dehumidifier_level_irq_handler(uint16_t GPIO_Pin);

#endif


