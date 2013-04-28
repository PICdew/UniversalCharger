#include <flash.h>

#define BASE_FLASH 0x1F80

#define FLASH

void writeFlash(UINT16 iAddr, UINT16 iData)
{
    UINT16 iBase;
    UINT16 aiBackup[32];
    UINT8 iNdx;
    #if defined(FLASH)
    // Backup row 32 bytes
    iBase = (iAddr >> 5) * 0x20;
    for(iNdx=0;iNdx<32;iNdx++)
        aiBackup[iNdx] = readFlash(iBase+iNdx);
    aiBackup[iAddr-iBase] = iData;
    //iAddr += BASE_FLASH;
    iBase += BASE_FLASH;

    GIE = 0;        //disable interupts incase they interfere
    //ERASE SECTION
    PMCON1 = 0;     //not configuration space
    PMADRL = iBase&0xFF;
    PMADRH = iBase>>8;
    FREE = 1;       //specify erase operation
    WREN = 1;       //allow write
    PMCON2 = 0x55;  //unlock program memory
    PMCON2 = 0xAA;  //unlock program memory
    WR = 1;         //begin write
    NOP();
    NOP();
    WREN = 0;       //disallow write
    //END OF ERASE SECTION

    PMCON1 = 0;     //not configuration space
    WREN = 1;       //allow write
    FREE = 0;       //selecting write operation
    LWLO = 1;       //load write latches only
    for(iNdx=0;iNdx<32;iNdx++)
    {
        if(aiBackup[iNdx]==0x03FF)
            continue;
        //WRITE SECTION
        PMADRL = (iBase+iNdx)&0xFF;
        PMADRH = (iBase+iNdx)>>8;
        PMDATL = aiBackup[iNdx]&0xFF;
        PMDATH = aiBackup[iNdx]>>8;
        if(iNdx==31)
            LWLO = 0;       //write latches to flash
        PMCON2 = 0x55;  //unlock program memory
        PMCON2 = 0xAA;  //unlock program memory
        WR = 1;         //begin write
        NOP();
        NOP();
    }
    WREN= 0;        //disallow write
    //END OF WRITE SECTION
    GIE = 1;        //enable interupts again
    #endif
}

UINT16 readFlash(UINT16 iAddr)
{
    iAddr += BASE_FLASH;

    PMCON1 = 0;     //not configuration space
    PMADRL = iAddr&0xFF;
    PMADRH = iAddr>>8;
    RD = 1;         //initiate read operation
    NOP();
    NOP();
    return (PMDATH<<8) | PMDATL;  //joins bytes & returns the value stored
}
