#ifndef FLASH_H
#define	FLASH_H

#include "GenericTypeDefs.h"
#include "Compiler.h"

void writeEEPROM(UINT16, UINT16);
UINT16 readEEPROM(UINT16);

#endif	/* FLASH_H */
