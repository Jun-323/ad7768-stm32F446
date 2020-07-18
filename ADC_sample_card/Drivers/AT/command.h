/**
  ******************************************************************************
  * @file    command.h
  * @author  
  * @brief   Header for driver command.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMAND_H__
#define __COMMAND_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "at.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/

/* Character added when a RX error has been detected */
#define AT_ERROR_RX_CHAR 0x01

extern uint8_t cdc_tx[1024];
extern uint8_t start_stop;
extern uint8_t converting;
/* Exported functions ------------------------------------------------------- */

/**
 * @brief Initializes command module
 *
 * @param [IN] None
 * @retval None
 */
void CMD_Init(void);

/**
 * @brief Process the command
 *
 * @param [IN] None
 * @retval None
 */
void CMD_Process(uint8_t *command, uint16_t i);

/**
 * @brief  Print a string corresponding to an ATEerror_t
 * @param  The AT error code
 * @retval None
 */
void com_error(ATEerror_t error_type);

#ifdef __cplusplus
}
#endif

#endif /* __COMMAND_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
