#ifndef FLASH_H
#define	FLASH_H

#include "GenericTypeDefs.h"
#include "Compiler.h"

void writeFlash(UINT16, UINT16);
UINT16 readFlash(UINT16);

#endif	/* FLASH_H */
