/* 
This file is part of Espruino, a JavaScript interpreter for Microcontrollers
 * adapted from source written by Chris Osborn <fozztexx@fozztexx.com>
 * http://insentricity.com
 *
 * Copyright (C) 2013 Gordon Williams <gw@pur3.co.uk>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * ----------------------------------------------------------------------------
 * ESP32 specific exposed components for neopixel.
 * ----------------------------------------------------------------------------
 */
 
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <soc/rmt_struct.h>
#include <soc/dport_reg.h>
#include <driver/gpio.h>
#include <soc/gpio_sig_map.h>
#include <esp_intr.h>
#include <driver/rmt.h>

#include "esp32_neopixel.h"
#include "jshardwareRMT.h"

#define MAX_PULSES	32
#define DIVIDER		4 /* Above 4, timings start to deviate*/
#define DURATION	12.5 /* minimum time of a single RMT duration */

#define PULSE_T0H	(  350 / (DURATION * DIVIDER));
#define PULSE_T1H	(  900 / (DURATION * DIVIDER));
#define PULSE_T0L	(  900 / (DURATION * DIVIDER));
#define PULSE_T1L	(  350 / (DURATION * DIVIDER));
#define PULSE_TRS	(50000 / (DURATION * DIVIDER));

typedef union {
  struct {
    uint32_t duration0:15;
    uint32_t level0:1;
    uint32_t duration1:15;
    uint32_t level1:1;
  };
  uint32_t val;
} rmtPulsePair;

static uint8_t *neopixel_buffer = NULL;
static unsigned int neopixel_pos, neopixel_len, neopixel_half;
static xSemaphoreHandle neopixel_sem = NULL;
static intr_handle_t rmt_intr_handle = NULL;
static rmtPulsePair neopixel_bits[2];

static uint8_t intRMTchannel[8]={0,1,2,3,4,5,6,7};

void neopixel_initRMTChannel(int rmtChannel){
  RMT.apb_conf.fifo_mask = 1;  //enable memory access, instead of FIFO mode.
  RMT.apb_conf.mem_tx_wrap_en = 1; //wrap around when hitting end of buffer
  RMT.conf_ch[rmtChannel].conf0.div_cnt = DIVIDER;
  RMT.conf_ch[rmtChannel].conf0.mem_size = 1;
  RMT.conf_ch[rmtChannel].conf0.carrier_en = 0;
  RMT.conf_ch[rmtChannel].conf0.carrier_out_lv = 1;
  RMT.conf_ch[rmtChannel].conf0.mem_pd = 0;

  RMT.conf_ch[rmtChannel].conf1.rx_en = 0;
  RMT.conf_ch[rmtChannel].conf1.mem_owner = 0;
  RMT.conf_ch[rmtChannel].conf1.tx_conti_mode = 0;    //loop back mode.
  RMT.conf_ch[rmtChannel].conf1.ref_always_on = 1;    // use apb clock: 80M
  RMT.conf_ch[rmtChannel].conf1.idle_out_en = 1;
  RMT.conf_ch[rmtChannel].conf1.idle_out_lv = 0;

  return;
}

void neopixel_copy(int rmtc){
  unsigned int i, j, offset, len, bit;
  offset = neopixel_half * MAX_PULSES;
  neopixel_half = !neopixel_half;
  len = neopixel_len - neopixel_pos;
  if (len > (MAX_PULSES / 8)) len = (MAX_PULSES / 8);
  if (!len) {
    for (i = 0; i < MAX_PULSES; i++)
      RMTMEM.chan[rmtc].data32[i + offset].val = 0;
    return;
  }
  for (i = 0; i < len; i++) {
    bit = neopixel_buffer[i + neopixel_pos];
    for (j = 0; j < 8; j++, bit <<= 1) {
      RMTMEM.chan[rmtc].data32[j + i * 8 + offset].val = neopixel_bits[(bit >> 7) & 0x01].val;
    }
    if (i + neopixel_pos == neopixel_len - 1)
      RMTMEM.chan[rmtc].data32[7 + i * 8 + offset].duration1 = PULSE_TRS;
  }
  for (i *= 8; i < MAX_PULSES; i++) RMTMEM.chan[rmtc].data32[i + offset].val = 0;
  neopixel_pos += len;
  return;
}

void neopixel_handleInterrupt(void *arg){
  
  // receive the RMT channel
  uint8_t i=*((uint8_t *)arg);
  
  portBASE_TYPE taskAwoken = 0;
  
  //if (RMT.int_st.ch0_tx_thr_event)
  if ( RMT.int_st.val & (0x01000000 << i) )
	{
    neopixel_copy(i);
	
	RMT.int_clr.val=RMT.int_clr.val || (0x01000000 << i);				// RMT.int_clr.ch0_tx_thr_event = 1;
	}
  //else if (RMT.int_st.ch0_tx_end && neopixel_sem)
  else if ( (RMT.int_st.val & (0x00000001 << (i*3))) && neopixel_sem )
	{
    xSemaphoreGiveFromISR(neopixel_sem, &taskAwoken);
    
	RMT.int_clr.val=RMT.int_clr.val || (0x00000001 << (i*3));			// RMT.int_clr.ch0_tx_end = 1;
	}
	
  return;
}

int neopixel_init(int gpioNum){
  
  int i;
  
  // get if exist the RMT fot this PIN (future check if it is for Neopixel or sendPulse)
  i = getRMTIndex(gpioNum);
  
  // if not ask for a new RMT channel and configure it
  if(i < 0)
	{
	i = getFreeRMT(gpioNum);
	}
  if(i < 0)
	{
	printf("all RMT channels in use\n");
	return i;
	}
  
	// configure RMT driver
  DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_RMT_CLK_EN);
  DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_RMT_RST);
  
  // configure RMT channel & GPIO pin
  rmt_set_pin((rmt_channel_t)i, RMT_MODE_TX, (gpio_num_t)gpioNum);

  neopixel_initRMTChannel(i);

  RMT.tx_lim_ch[i].limit = MAX_PULSES;
  
  // config bits depending of channel
  RMT.int_ena.val=RMT.int_ena.val || (0x01000000 << i);					// RMT.int_ena.ch?_tx_thr_event = 1;
  RMT.int_ena.val=RMT.int_ena.val || (0x00000001 << (i*3));				// RMT.int_ena.ch?_tx_end = 1;
  
  neopixel_bits[0].level0 = 1;
  neopixel_bits[0].level1 = 0;
  neopixel_bits[0].duration0 = PULSE_T0H;
  neopixel_bits[0].duration1 = PULSE_T0L;
  neopixel_bits[1].level0 = 1;
  neopixel_bits[1].level1 = 0;
  neopixel_bits[1].duration0 = PULSE_T1H;
  neopixel_bits[1].duration1 = PULSE_T1L;

  // add argument to pass to the interrupt handler to know wich channel use
  esp_intr_alloc(ETS_RMT_INTR_SOURCE, 0, neopixel_handleInterrupt, &(intRMTchannel[i]) , &rmt_intr_handle);
  
  return i;
}

bool esp32_neopixelWrite(Pin pin,unsigned char *rgbData, size_t rgbSize){
  
  int i;		// RMT channel
  
  jsWarn('API RMT debug : start neopixel write\n');				// RIC DEBUG
  i=neopixel_init(pin);
  if (i >=0)
	{
	jsWarn('API RMT debug : RMT channel %d\n',i);					// RIC DEBUG
	neopixel_buffer = rgbData;
	neopixel_len = rgbSize;
	neopixel_pos = 0;
	neopixel_half = 0;
	neopixel_copy(i);
	if (neopixel_pos < neopixel_len) neopixel_copy(i);
	jsWarn('API RMT debug : Create sempahore for Neopixel\n');	// RIC DEBUG
	neopixel_sem = xSemaphoreCreateBinary();
	RMT.conf_ch[i].conf1.mem_rd_rst = 1;
	RMT.conf_ch[i].conf1.tx_start = 1;
	jsWarn('API RMT debug : Waiting sempahore\n');				// RIC DEBUG
	xSemaphoreTake(neopixel_sem, portMAX_DELAY);
	jsWarn('API RMT debug : sempahore free\n');					// RIC DEBUG
	vSemaphoreDelete(neopixel_sem);
	neopixel_sem = NULL;
	return true;
	}

  return false;
}
