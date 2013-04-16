#ifndef LCD_H
#define	LCD_H

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"

void __inline lcdConfig(void);
void lcdOut(UINT8, const char *);
void lcdChar(UINT8, char);

#endif	/* LCD_H */
