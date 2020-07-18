#include "CDCEL913.h"
#include "stm32f4xx_hal.h"

const uint8_t REG_GEN_INIT_VALUE[7] = {0x01, 0x00, 0xb4, 0x06, 0x02, 0x00, 0x40};
uint8_t read_buff[15] = {0};
const uint8_t REG_PLL_INIT_VALUE[15] = {
0x00, 0x00, 0x00, 0x00, 0x4d, 
0x02, 0x00, 0x00, 0xc3, 0xa5,
0xb3, 0xeb, 0xc3, 0xa5, 0xb3};

extern FMPI2C_HandleTypeDef hfmpi2c1;

void CDCEL913_Config(void)
{
	
//	HAL_FMPI2C_Mem_Write(&hfmpi2c1, 0xca, 0x00, FMPI2C_MEMADD_SIZE_8BIT, (uint8_t *)REG_GEN_INIT_VALUE, 7, 1000);
	HAL_FMPI2C_Mem_Read(&hfmpi2c1, 0xcb, 0x00, FMPI2C_MEMADD_SIZE_8BIT, read_buff, 7, 1000);
//	HAL_FMPI2C_Mem_Write(&hfmpi2c1, 0xca, 0x10, FMPI2C_MEMADD_SIZE_8BIT, (uint8_t *)REG_PLL_INIT_VALUE, 15, 1000);
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, 0);
}

