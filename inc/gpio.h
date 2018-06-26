
#ifndef GPIO_H
#define GPIO_H

/**
 * CONSTANTS
 */

#define NUM_GPIO_INPUTS 16
#define NUM_GPIO_OUTPUTS 2

// don't use these!
#define GPIO_INPUT_BLUE_BUTTON 0
#define GPIO_INPUT_NC_1 1
#define GPIO_INPUT_NC_3 3
#define GPIO_INPUT_NC_9 9
#define GPIO_INPUT_NC_14 14

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

#define OFF 0
#define ON 1
#define TOGGLE 2


// PROTOTYPES

typedef struct gpio_output {
  uint16_t gpio_pin;
  GPIO_TypeDef *gpio_port;
  uint8_t desired_state;
  uint32_t run_for_ms;
} gpio_output_struct_t ;

extern gpio_output_struct_t gpio_outputs[NUM_GPIO_OUTPUTS];

void gpio_init(void);

#endif


