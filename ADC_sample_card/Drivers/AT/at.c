/**
  ******************************************************************************
  * @file    at.c
  * @author  
  * @brief   at command API
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "at.h"
#include "command.h"
#include "ad7768.h"
#include "flash.h"
//#include "utilities.h"
//#include "lora.h"
//#include "LoRaMacTest.h"
//#include "radio.h"
//#include "vcom.h"
//#include "tiny_sscanf.h"
//#include "version.h"
//#include "hw_msp.h"
//#include "test_rf.h"

#define PROGRAM_VER    "0.0.4"

/**
 * @brief Max size of the data that can be received
 */
#define MAX_RECEIVED_DATA 255

/* Private macro -------------------------------------------------------------*/
/**
 * @brief Macro to return when an error occurs
 */
#define CHECK_STATUS(status) do {                    \
    ATEerror_t at_status = translate_status(status); \
    if (at_status != AT_OK) { return at_status; }    \
  } while (0)

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Exported functions ------------------------------------------------------- */


ATEerror_t at_return_ok(const char *param)
{
  return AT_OK;
}

ATEerror_t at_return_error(const char *param)
{
  return AT_ERROR;
}

ATEerror_t at_reset(const char *param)
{
  NVIC_SystemReset();
  /* return AT_OK; */
}


ATEerror_t at_version_get(const char *param)
{
  AT_PRINTF(PROGRAM_VER);
//  AT_PRINTF("MAC_VERSION= %02X.%02X.%02X.%02X\r\n", (uint8_t)(__LORA_MAC_VERSION >> 24), (uint8_t)(__LORA_MAC_VERSION >> 16), (uint8_t)(__LORA_MAC_VERSION >> 8), (uint8_t)__LORA_MAC_VERSION);
  return AT_OK;
}


ATEerror_t at_start(const char *param)
{
	start_stop  = 1;
	converting = 1;
	ad7768_start();
	return AT_OK;
}

ATEerror_t at_stop(const char *param)
{
	start_stop  = 1;
	converting = 0;
	ad7768_stop();
	return AT_OK;
}


int coupling = 0;

ATEerror_t at_coupling_set(const char *param)
{
	uint8_t ch, coupling;
	sscanf(param, "%hhu,%hhu", &ch, &coupling);
	
	if((ch > 4) || (!ch))
	{
		return AT_PARAM_ERROR;
	}
	
	ad7768_coupling_set(ch, coupling);
//	ad7768_stop();
	return AT_OK;
}


ATEerror_t at_coupling_get(const char *param)
{
//	ad7768_stop();
	sprintf((char *)cdc_tx, "ch1:%hhu, ch2:%hhu, ch3:%hhu, ch4:%hhu", \
		ad7768_device.coupling_mode[0], ad7768_device.coupling_mode[1], ad7768_device.coupling_mode[2], ad7768_device.coupling_mode[3]);
	return AT_OK;
}


ATEerror_t at_ch_state_set(const char *param)
{
	uint8_t ch, state;
	
	sscanf(param, "%hhu,%hhu", &ch, &state);
	if((ch > 4) || (!ch))
	{
		return AT_PARAM_ERROR;
	}
	
	ad7768_set_ch_state(&ad7768_device, (ad7768_ch)(ch-1), (ad7768_ch_state)state);
	
	return AT_OK;
}

ATEerror_t at_ch_state_get(const char *param)
{
	sprintf((char *)cdc_tx, "ch1:%hhu, ch2:%hhu, ch3:%hhu, ch4:%hhu", \
	ad7768_device.ch_state[0], ad7768_device.ch_state[1], ad7768_device.ch_state[2], ad7768_device.ch_state[3]);
	
	return AT_OK;
}


ATEerror_t at_IEPE_set(const char *param)
{
	uint8_t ch, state;
	
	sscanf(param, "%hhu,%hhu", &ch, &state);
	if((ch > 4) || (!ch))
	{
		return AT_PARAM_ERROR;
	}
	
	ad7768_IEPE_set(ch, (ad7768_ch_state)state);
	
	return AT_OK;
}

ATEerror_t at_IEPE_get(const char *param)
{
	sprintf((char *)cdc_tx, "ch1:%hhu, ch2:%hhu, ch3:%hhu, ch4:%hhu", \
	ad7768_device.IEPE[0], ad7768_device.IEPE[1], ad7768_device.IEPE[2], ad7768_device.IEPE[3]);
	
	return AT_OK;
}


ATEerror_t at_GAIN_set(const char *param)
{
	uint8_t ch;
	uint32_t gain;
	
	sscanf(param, "%hhu,%u", &ch, &gain);
	
	if((!ch) || (ch > 4))
	{
		return AT_PARAM_ERROR;
	}
	
	if(gain > 0xffffff)
	{
		return AT_PARAM_ERROR;
	}
	
	ad7768_gain_set(ch, gain);
	
	return AT_OK;
}

ATEerror_t at_GAIN_get(const char *param)
{
	sprintf((char *)cdc_tx, "ch1:%u, ch2:%u, ch3:%u, ch4:%u", \
	ad7768_device.gain[0], ad7768_device.gain[1], ad7768_device.gain[2], ad7768_device.gain[3]);
	
	return AT_OK;
}


ATEerror_t at_TYPE_set(const char *param)
{
	sscanf(param, "%hhu", &ad7768_device.dev_type);
	return AT_OK;
}

ATEerror_t at_TYPE_get(const char *param)
{
	sprintf((char *)cdc_tx, "%hhu", ad7768_device.dev_type);
	return AT_OK;
}

ATEerror_t at_DATA_set(const char *param)
{
	uint16_t size;
	sscanf(param, "%hu", &size);
	
	if(size && (size <= 256))
	{
		ad7768_device.data_block_size = size;
		return AT_OK;
	}
	else
	{
		return AT_PARAM_ERROR;
	}
//	sscanf(param, "%hu", &ad7768_device.data_block_size);
}

ATEerror_t at_DATA_get(const char *param)
{
	sprintf((char *)cdc_tx, "%hu", ad7768_device.data_block_size);
	return AT_OK;
}


ATEerror_t at_RATE_set(const char *param)
{
	uint16_t rate;
	sscanf(param, "%hu", &rate);
	if((rate==32) || (rate==64) || (rate==128) || (rate==256))
	{
		ad7768_rate_set(rate);
		return AT_OK;
	}
	else
	{
		return AT_ERROR;
	}
}

ATEerror_t at_RATE_get(const char *param)
{
	sprintf((char *)cdc_tx, "%hu", ad7768_device.rate);
	return AT_OK;
}


ATEerror_t at_RST(const char *param)
{
	ad7768_reset();
	return AT_OK;
}


ATEerror_t at_TRIG_set(const char *param)
{
	sscanf(param, "%hhu", &ad7768_device.trig);
	digital_trig_set();
	return AT_OK;
}

ATEerror_t at_TRIG_get(const char *param)
{
	sprintf((char *)cdc_tx, "%hhu", ad7768_device.trig);
	return AT_OK;
}

ATEerror_t at_save(const char *param)
{
	flash_write((uint32_t *)&ad7768_device, sizeof(ad7768_dev)/4);
	return AT_OK;
}
