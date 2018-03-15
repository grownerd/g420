
#ifndef GPIO_H
#define GPIO_H

/**
 * CONSTANTS
 */

#define NUM_GPIO_INPUTS 16
#define NUM_GPIO_OUTPUTS 1

// don't use these!
#define GPIO_INPUT_BLUE_BUTTON 0
#define GPIO_INPUT_NC_1 1
#define GPIO_INPUT_NC_3 3
#define GPIO_INPUT_NC_9 9
#define GPIO_INPUT_NC_14 14

#define GPIO_OUTPUT_FEED_PUMP 0

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


