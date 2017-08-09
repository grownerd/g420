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

#define DS18B20_NUM_DEVICES    2
extern sensor_t ds18b20_sensors[DS18B20_NUM_DEVICES];

/**
 * PROTOTYPES

uint8_t ds18b20_devices[DS18B20_NUM_DEVICES][8] = {
  {0x28, 0xff, 0xd8, 0x1a, 0x81, 0x16, 0x3, 0x66},
  {0x28, 0xff, 0x4b, 0x8d, 0x84, 0x16, 0x5, 0x10}
};

 */


void onewire_init(void);
void ds18b20_read_temp();


#endif
