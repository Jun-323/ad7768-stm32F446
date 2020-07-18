/**
  ******************************************************************************
  * @file    command.c
  * @author  
  * @brief   main command driver dedicated to command AT
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include "at.h"
//#include "hw.h"
#include "command.h"
#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "flash.h"
#include  "ad7768.h"
/* comment the following to have help message */
/* #define NO_HELP */
/* #define NO_KEY_ADDR_EUI */

uint8_t cdc_tx[1024] = {0};

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief  Structure defining an AT Command
 */
struct ATCommand_s
{
  const char *string;                       /*< command string, after the "AT" */
  const int size_string;                    /*< size of the command string, not including the final \0 */
  ATEerror_t (*get)(const char *param);     /*< =? after the string to get the current value*/
  ATEerror_t (*set)(const char *param);     /*< = (but not =?\0) after the string to set a value */
  ATEerror_t (*run)(const char *param);     /*< \0 after the string - run the command */
#if !defined(NO_HELP)
  const char *help_string;                  /*< to be printed when ? after the string */
#endif
};

/* Private define ------------------------------------------------------------*/
#define CMD_SIZE 270
#define CIRC_BUFF_SIZE 8
#define HELP_DISPLAY_FLUSH_DELAY 100

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/**
 * @brief  Array corresponding to the description of each possible AT Error
 */
static const char *const ATError_description[] =
{
  "\r\nOK\r\n",                     /* AT_OK */
  "\r\nAT_ERROR\r\n",               /* AT_ERROR */
  "\r\nAT_PARAM_ERROR\r\n",         /* AT_PARAM_ERROR */
  "\r\nAT_BUSY_ERROR\r\n",          /* AT_BUSY_ERROR */
  "\r\nAT_TEST_PARAM_OVERFLOW\r\n", /* AT_TEST_PARAM_OVERFLOW */
  "\r\nAT_NO_NETWORK_JOINED\r\n",   /* AT_NO_NET_JOINED */
  "\r\nAT_RX_ERROR\r\n",            /* AT_RX_ERROR */
  "\r\nAT_NO_CLASS_B_ENABLE\r\n",  /* AT_NO_CLASS_B_ENABLE */
  "\r\nerror unknown\r\n",          /* AT_MAX */
};

/**
 * @brief  Array of all supported AT Commands
 */
static const struct ATCommand_s ATCommand[] =
{
  {
    .string = AT_RESET,
    .size_string = sizeof(AT_RESET) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RESET ": Trig a reset of the MCU\r\n",
#endif
    .get = at_return_error,
    .set = at_return_error,
    .run = at_reset,
  },
	
	{
    .string = AT_VER,
    .size_string = sizeof(AT_VER) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_VER ": Get the version of the AT_Slave FW\r\n",
#endif
    .get = at_version_get,
    .set = at_return_error,
    .run = at_return_error,
  },
	
	{
    .string = AT_START,
    .size_string = sizeof(AT_START) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_START ": start ad7768 convert\r\n",
#endif
    .get = at_return_error,
    .set = at_return_error,
    .run = at_start,
  },
	
	{
    .string = AT_STOP,
    .size_string = sizeof(AT_STOP) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_STOP ": stop ad7768 convert\r\n",
#endif
    .get = at_return_error,
    .set = at_return_error,
    .run = at_stop,
  },
	
	{
    .string = AT_COUPL,
    .size_string = sizeof(AT_COUPL) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_COUPL ": get or set channel coupling mode(0:DC, 1:AC)\r\n",
#endif
    .get = at_coupling_get,
    .set = at_coupling_set,
    .run = at_return_error,
  },
	
	{
    .string = AT_CHSTATE,
    .size_string = sizeof(AT_CHSTATE) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_CHSTATE ": get or set channel state.(0:Enable, 1:Disable)\r\n",
#endif
    .get = at_ch_state_get,
    .set = at_ch_state_set,
    .run = at_return_error,
  },
	
	{
    .string = AT_IEPE,
    .size_string = sizeof(AT_IEPE) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_IEPE ": get or set IEPE.(0:Disable, 1:Enable)\r\n",
#endif
    .get = at_IEPE_get,
    .set = at_IEPE_set,
    .run = at_return_error,
  },
	
	{
    .string = AT_GAIN,
    .size_string = sizeof(AT_GAIN) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_GAIN ": get or set gain, 0-16777215(0xffffff), the default gain value is 5592405(0x555555)\r\n",
#endif
    .get = at_GAIN_get,
    .set = at_GAIN_set,
    .run = at_return_error,
  },
	
	{
    .string = AT_TYPE,
    .size_string = sizeof(AT_TYPE) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_TYPE ": get or set device type, master(0) or slave(1)\r\n",
#endif
    .get = at_TYPE_get,
    .set = at_TYPE_set,
    .run = at_return_error,
  },
	
	{
    .string = AT_DATA,
    .size_string = sizeof(AT_DATA) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_DATA ": get or set data block size, the max size is 256\r\n",
#endif
    .get = at_DATA_get,
    .set = at_DATA_set,
    .run = at_return_error,
  },
	
	{
    .string = AT_RATE,
    .size_string = sizeof(AT_RATE) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RATE ": get or set sample rate\r\n",
#endif
    .get = at_RATE_get,
    .set = at_RATE_set,
    .run = at_return_error,
  },
	
	{
    .string = AT_RST,
    .size_string = sizeof(AT_RST) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_RST ": set the device to factory setting\r\n",
#endif
    .get = at_return_error,
    .set = at_return_error,
    .run = at_RST,
  },
	
	{
    .string = AT_SAVE,
    .size_string = sizeof(AT_SAVE) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_SAVE ": save the parameters\r\n",
#endif
    .get = at_return_error,
    .set = at_return_error,
    .run = at_save,
  },
	
	{
    .string = AT_TRIG,
    .size_string = sizeof(AT_TRIG) - 1,
#ifndef NO_HELP
    .help_string = "AT"AT_TRIG ": set or get the digital channel tirgger.(0:falling edge, 1:rising edge)\r\n",
#endif
    .get = at_TRIG_get,
    .set = at_TRIG_set,
    .run = at_return_error,
  },
};


/* Private function prototypes -----------------------------------------------*/



/**
 * @brief  Parse a command and process it
 * @param  The command
 * @retval None
 */
static void parse_cmd(const char *cmd);

/* Exported functions ---------------------------------------------------------*/


void CMD_Process(uint8_t *command, uint16_t i)
{
  /* Process all commands */


#if 0 /* echo On    */
    PRINTF("%c", circBuffer[ridx]);
#endif
	
		if ((command[i-1] == '\r') || (command[i-1] == '\n'))
		{
			command[i-2] = '\0';
			parse_cmd((const char*)command);
		}
}

void com_error(ATEerror_t error_type)
{
  if (error_type > AT_MAX)
  {
    error_type = AT_MAX;
  }
  AT_PRINTF(ATError_description[error_type]);
}

/* Private functions ---------------------------------------------------------*/

uint8_t start_stop = 0;
uint8_t converting = 0;

static void parse_cmd(const char *cmd)
{
	memset(cdc_tx, 0, sizeof(cdc_tx));
	
  ATEerror_t status = AT_OK;
  const struct ATCommand_s *Current_ATCommand;
  int i;

  if ((cmd[0] != 'A') || (cmd[1] != 'T'))
  {
    status = AT_ERROR;
  }
  else if (cmd[2] == '\0')
  {
    /* status = AT_OK; */
  }
  else if (cmd[2] == '?')
  {
#ifdef NO_HELP
#else
    AT_PRINTF("AT+<CMD>?        : Help on <CMD>\r\n"
            "AT+<CMD>         : Run <CMD>\r\n"
            "AT+<CMD>=<value> : Set the value\r\n"
            "AT+<CMD>=?       : Get the value\r\n");
    for (i = 0; i < (sizeof(ATCommand) / sizeof(struct ATCommand_s)); i++)
    {
      AT_PRINTF(ATCommand[i].help_string);
    }
    /* Wait for the message queue to be flushed in order
       not to disturb following com_error() display */
//    osDelay(HELP_DISPLAY_FLUSH_DELAY);
#endif
  }
  else
  {
    /* point to the start of the command, excluding AT */
    status = AT_ERROR;
    cmd += 2;
    for (i = 0; i < (sizeof(ATCommand) / sizeof(struct ATCommand_s)); i++)
    {
      if (strncmp(cmd, ATCommand[i].string, ATCommand[i].size_string) == 0)
      {
        Current_ATCommand = &(ATCommand[i]);
        /* point to the string after the command to parse it */
        cmd += Current_ATCommand->size_string;

        /* parse after the command */
        switch (cmd[0])
        {
          case '\0':    /* nothing after the command */
            status = Current_ATCommand->run(cmd);
            break;
          case '=':
            if ((cmd[1] == '?') && (cmd[2] == '\0'))
            {
              status = Current_ATCommand->get(cmd + 1);
            }
            else
            {
              status = Current_ATCommand->set(cmd + 1);
//							flash_write((uint32_t *)&ad7768_device, sizeof(ad7768_dev)/4);
            }
            break;
          case '?':
#ifndef NO_HELP
            AT_PRINTF(Current_ATCommand->help_string);
#endif
            status = AT_OK;
            break;
          default:
            /* not recognized */
            break;
        }

        /* we end the loop as the command was found */
        break;
      }
    }
  }

  com_error(status);
	
	if(start_stop || converting)
	{
		start_stop = 0;
		return;
	}
	CDC_Transmit_HS(cdc_tx, strlen((const char*)cdc_tx));
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
