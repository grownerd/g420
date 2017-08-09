
#ifndef RTC_H
#define RTC_H

/**
 * CONSTANTS
 */

/**
 * PROTOTYPES
 */

void update_datestring(void);
void print_time(void);
void set_time(char * timestring);
void set_alarm(TM_RTC_Alarm_t alarm, uint8_t hours, uint8_t minutes);

extern TM_RTC_Time_t Time;

#endif
