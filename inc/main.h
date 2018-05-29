#ifndef _MAIN_H_
#define _MAIN_H_


#define DATESTR_LEN 42
#define MAX_STR_LEN 2048
#define V_IN 3.3F
#define FLASH_SIZE 2048
#define MAX_TIMER_EVENTS 100


extern char datestring[DATESTR_LEN];

typedef struct timer_event {
  uint32_t on_time;
  uint32_t off_time;
  uint32_t desired_state;
  uint32_t gpio_num;
} timer_event_struct_t;

timer_event_struct_t timer_events[MAX_TIMER_EVENTS];


// Function prototypes
void init_timer_events(void);
void gpio_scheduler(void);
void set_defaults(void);
uint8_t slot_is_empty(uint32_t slot);

#endif
