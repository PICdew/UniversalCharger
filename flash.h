#ifndef FLASH_H
#define	FLASH_H

#include "GenericTypeDefs.h"
#include "Compiler.h"

void writeEEPROM(UINT8, UINT16);
UINT16 readEEPROM(UINT8);

#endif	/* FLASH_H */
