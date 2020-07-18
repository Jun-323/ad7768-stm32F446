#ifndef __SDRAM_H
#define __SDRAM_H

#include "main.h"

uint8_t sdram_test(void);

void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command);

#endif
