/*
 * This file is designed to support Pulse functions in Espruino,
 * a JavaScript interpreter for Microcontrollers designed by Gordon Williams
 *
 * Copyright (C) 2016 by Juergen Marsch 
 *
 * This Source Code Form is subject to the terms of the Mozilla Publici
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * ----------------------------------------------------------------------------
 * This file is designed to be parsed during the build process
 *
 * Contains ESP32 board specific functions.
 * ----------------------------------------------------------------------------
 */
#include "jsutils.h"
 
#include "jshardwareRMT.h"
#include "driver/rmt.h"

#include <stdio.h>

#define RMTPinEmpty 111

rmt_item32_t items[1];

int RMTInitChannel(Pin pin, bool pulsePolarity){
  rmt_config_t config;
  int i = getFreeRMT(pin);
  if(i >= 0)
	{
    config.rmt_mode = RMT_MODE_TX;
    config.channel = i;
    config.gpio_num = pin;
    config.mem_block_num = 1;
	
	/* ESP-IDF how carrier_duty_percent is used ->
			if (carrier_en)
				{
				uint32_t duty_div, duty_h, duty_l;
				duty_div = rmt_source_clk_hz / carrier_freq_hz;
				duty_h = duty_div * carrier_duty_percent / 100;
				duty_l = duty_div - duty_h;
				rmt_ll_set_carrier_on_level(dev, channel, carrier_level);
				rmt_ll_set_tx_carrier_high_low_ticks(dev, channel, duty_h, duty_l);
				. . .
	*/
	
	config.tx_config.loop_en = 0;
    config.tx_config.carrier_en = 0;				// in theory we DO NOT use carrier
    config.tx_config.idle_output_en = 1;
    config.tx_config.carrier_freq_hz = 10000;
	config.tx_config.carrier_duty_percent = 50;
    config.tx_config.carrier_level = 1;
    config.clk_div = 80;
	
	// when idle how do we leave the PIN high or low
	if (pulsePolarity) config.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH; 
    else config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
	
    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
    return i;
  }
  else return -1;
} 

void setPulseLow(int duration){
  items[0].duration0 = duration;
  items[0].level0 = 0;
  items[0].duration1 = 10;
  items[0].level1 = 1;
}

void setPulseHigh(int duration){
  items[0].duration0 = duration;
  items[0].level0 = 1;
  items[0].duration1 = 10;
  items[0].level1 = 0;
}

//pin to be pulsed. value to be pulsed into the pin. duration in milliseconds to hold the pin.
void sendPulse(Pin pin, bool pulsePolarity, int duration){
  
  int i;
  
  // get if exist the RMT fot this PIN (future check if it is for Neopixel or sendPulse)
  i = getRMTIndex(pin);
  
  // if not ask for a new RMT channel and configure it
  if(i < 0) i = RMTInitChannel(pin,pulsePolarity);
  
  if(i >= 0)
	{
    if(pulsePolarity) setPulseLow(duration);else setPulseHigh(duration);
	rmt_set_pin(i, RMT_MODE_TX, pin); //set pin to rmt, in case that it was reset to GPIO(see jshPinSetValue)
    rmt_write_items(i,items,1,1);
	}
  else printf("all RMT channels in use\n");
  
  return;
}
	