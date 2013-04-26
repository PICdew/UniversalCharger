#include <flash.h>

#define BASE_FLASH 0x1F80

void writeFlash(UINT16 iAddr, UINT16 iData)
{
    iAddr += BASE_FLASH;

    GIE = 0;        //disable interupts incase they interfere
    //ERASE SECTION
    PMCON1 = 0;     //not configuration space
    PMADRL = iAddr&0xFF;
    PMADRH = (iAddr&0xFF00)>>8;
    FREE = 1;       //specify erase operation
    WREN = 1;       //allow write
    PMCON2 = 0x55;  //unlock program memory
    PMCON2 = 0xAA;  //unlock program memory
    WR = 1;         //begin write
    NOP();
    NOP();
    WREN = 0;       //disallow write
    //END OF ERASE SECTION

    //WRITE SECTION
    PMCON1 = 0;     //not configuration space
    PMADRL = iAddr&0xFF;
    PMADRH = (iAddr&0xFF00)>>8;
    FREE = 0;       //selecting write operation
    LWLO = 1;       //load write latches only
    WREN = 1;       //allow write
    PMDATL = iData&0xFF;
    PMDATH = (iData&0xFF00)>>8;
    LWLO = 0;       //write latches to flash
    PMCON2 = 0x55;  //unlock program memory
    PMCON2 = 0xAA;  //unlock program memory
    WR = 1;         //begin write
    NOP();
    NOP();
    WREN= 0;        //disallow write
    //END OF WRITE SECTION
    GIE = 1;        //enable interupts again
}

UINT16 readFlash(UINT16 iAddr)
{
    iAddr += BASE_FLASH;

    PMCON1 = 0;     //not configuration space
    PMADRL = iAddr&0xFF;
    PMADRH = (iAddr&0xFF00)>>8;
    RD = 1;         //initiate read operation
    NOP();
    NOP();
    return (PMDATH<<8) | PMDATL;  //joins bytes & returns the value stored
}
