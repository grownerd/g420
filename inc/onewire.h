/**
 * @filename rtc_i2c.h
 * @description ONEWIRE support library for stm32f4 board
 * @author Nicholas Shrake, <shraken@gmail.com>
 */

#ifndef RTC_ONEWIRE_H
#define RTC_ONEWIRE_H

/**
 * CONSTANTS
 */

/* How many sensors we are expecting on 1wire bus? */
#define DS18B20_NUM_DEVICES    2

/**
 * PROTOTYPES
 */

void onewire_init(void);
void ds18b20_read_temp(void );


#endif
