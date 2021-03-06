
#ifndef GPIO_H
#define GPIO_H

/**
 * CONSTANTS
 */

#define NUM_IRQ_PINS 16
#define NUM_RELAYS 3
#define NUM_GPIO_OUTPUTS 11

#define RELAY_LIGHT 0
#define RELAY_EXHAUST 1
#define RELAY_AUX 2

#define GPIO_OUTPUT_FILL_PUMP 0
#define GPIO_OUTPUT_DRAIN_PUMP 1
#define GPIO_OUTPUT_COOLANT_PUMP 2
#define GPIO_OUTPUT_SEWAGE_PUMP 3
#define GPIO_OUTPUT_DEHUMI_PUMP 4
#define GPIO_OUTPUT_PHDOWN_PUMP 5
#define GPIO_OUTPUT_NUTRIENT1_PUMP 6
#define GPIO_OUTPUT_NUTRIENT2_PUMP 7
#define GPIO_OUTPUT_NUTRIENT3_PUMP 8
#define GPIO_OUTPUT_DEEP_RED_LEDS 9
#define GPIO_OUTPUT_STIRRER_MOTORS 10

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

typedef struct gpio_output {
  uint16_t gpio_pin;
  GPIO_TypeDef *gpio_port;
  uint8_t desired_state;
  uint32_t run_for_ms;
}gpio_output_struct_t ;


typedef struct irq_switch{
  GPIO_TypeDef* gpio_port;
  uint16_t gpio_pin;
  uint8_t type            : 1; 
  uint8_t idle_state      : 1; 
  uint8_t last_state      : 1; 
  uint8_t current_state   : 1; 
  uint8_t                 : 4; 
} irq_switch_struct_t ;

extern TM_PWMIN_t PWMIN1_Data, PWMIN2_Data;

void exti_init(void);
void gpio_init(void);
void pwmin_init(void);
void switch_relay(output_relay_struct_t * relay, uint8_t action);

void dosing_pump_timer_interrupt_init (void);
void dosing_pump_timer_init(uint32_t run_for_ms);
void dosing_pump_timer_start (void);
void dosing_pump_timer_stop (void);
void dosing_pump_timer_interrupt_enable (void);
void dosing_pump_timer_interrupt_disable (void);

#endif


