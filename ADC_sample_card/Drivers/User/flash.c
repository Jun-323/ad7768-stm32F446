#include "flash.h"


void flash_write(uint32_t *data, uint16_t size)
{
	uint32_t Address = ADDR_FLASH_SECTOR_7, SECTORError = 0;
	FLASH_EraseInitTypeDef EraseInitStruct;
	
  HAL_FLASH_Unlock();
	
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector        = FLASH_SECTOR_7;
  EraseInitStruct.NbSectors     = 1;
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		return;
	}
	
	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	 you have to make sure that these data are rewritten before they are accessed during code
	 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	 DCRST and ICRST bits in the FLASH_CR register. */
  __HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();

  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
	
	
	while (size--)
	{
	  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *data) == HAL_OK)
    {
      Address = Address + 4;
			data++;
    }
	}
	
	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
}

	
	
void flash_read(uint32_t *data, uint16_t size)
{
	uint32_t Address = ADDR_FLASH_SECTOR_7;
	
	while(size--)
	{
		*data = *(__IO uint32_t *)Address;

    Address = Address + 4;
		data++;
	}
}


