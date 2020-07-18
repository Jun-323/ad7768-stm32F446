/**
  ******************************************************************************
  * @file    at.h
  * @author  
  * @brief   Header for driver at.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AT_H__
#define __AT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* Exported types ------------------------------------------------------------*/
/*
 * AT Command Id errors. Note that they are in sync with ATError_description static array
 * in command.c
 */
typedef enum eATEerror
{
  AT_OK = 0,
  AT_ERROR,
  AT_PARAM_ERROR,
  AT_BUSY_ERROR,
  AT_TEST_PARAM_OVERFLOW,
  AT_NO_NET_JOINED,
  AT_RX_ERROR,
  AT_NO_CLASS_B_ENABLE,
  AT_MAX,
} ATEerror_t;

/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* AT printf */
#define AT_PRINTF(a)     strcat((char*)cdc_tx, (char*)a)

/* AT Command strings. Commands start with AT */
#define AT_RESET      "Z"
#define AT_VER        "+VER"
#define AT_START      "+START"
#define AT_STOP       "+STOP"
#define AT_COUPL      "+COUPL"
#define AT_CHSTATE    "+STATE"
#define AT_IEPE       "+IEPE"
#define AT_GAIN       "+GAIN"
#define AT_TYPE       "+TYPE"
#define AT_DATA       "+DATA"
#define AT_RATE       "+RATE"
#define AT_RST        "+RST"
#define AT_TRIG				"+TRIG"
#define AT_SAVE       "+SAVE"
/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Return AT_OK in all cases
 * @param  Param string of the AT command - unused
 * @retval AT_OK
 */
ATEerror_t at_return_ok(const char *param);

/**
 * @brief  Return AT_ERROR in all cases
 * @param  Param string of the AT command - unused
 * @retval AT_ERROR
 */
ATEerror_t at_return_error(const char *param);

/**
 * @brief  Trig a reset of the MCU
 * @param  Param string of the AT command - unused
 * @retval AT_OK
 */
ATEerror_t at_reset(const char *param);

/**
 * @brief  Print the version of the AT_Slave FW
 * @param  String parameter
 * @retval AT_OK
 */
ATEerror_t at_version_get(const char *param);


ATEerror_t at_start(const char *param);

ATEerror_t at_stop(const char *param);

ATEerror_t at_coupling_set(const char *param);

ATEerror_t at_coupling_get(const char *param);

ATEerror_t at_ch_state_set(const char *param);

ATEerror_t at_ch_state_get(const char *param);

ATEerror_t at_IEPE_set(const char *param);

ATEerror_t at_IEPE_get(const char *param);

ATEerror_t at_GAIN_set(const char *param);

ATEerror_t at_GAIN_get(const char *param);

ATEerror_t at_TYPE_set(const char *param);

ATEerror_t at_TYPE_get(const char *param);

ATEerror_t at_DATA_set(const char *param);

ATEerror_t at_DATA_get(const char *param);

ATEerror_t at_RATE_set(const char *param);

ATEerror_t at_RATE_get(const char *param);

ATEerror_t at_RST(const char *param);

ATEerror_t at_TRIG_set(const char *param);

ATEerror_t at_TRIG_get(const char *param);

ATEerror_t at_save(const char *param);

#ifdef __cplusplus
}
#endif

#endif /* __AT_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
