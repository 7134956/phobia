#ifndef _H_FLASH_
#define _H_FLASH_

#ifdef STM32F7
#define FLASH_SECTOR_MAX	2
#define FLASH_SECTOR_RELOC	0
#else
#define FLASH_SECTOR_MAX	5
#define FLASH_SECTOR_RELOC	3
#endif

extern const unsigned long	FLASH_map[];

void *FLASH_erase(void *flash);
void *FLASH_prog(void *flash, const void *s, int n);

void FLASH_selfupdate();

#endif /* _H_FLASH_ */

