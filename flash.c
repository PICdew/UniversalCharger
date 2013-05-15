#include <flash.h>

#define BASE_FLASH 0x1F80

#define REAL_FLASH

#warning "writeEEPROM should be check the write and return BOOL"
void writeEEPROM(UINT8 iAddr, UINT16 iData)
{
    #if defined(_PIC14E)
    UINT16 iBase;
    UINT16 aiBackup[32];
    UINT8 iNdx;
    #if defined(REAL_FLASH)
    // Backup row 32 bytes
    iBase = (iAddr >> 5) * 0x20;
    for(iNdx=0;iNdx<32;iNdx++)
        aiBackup[iNdx] = readEEPROM(iBase+iNdx);
    aiBackup[iAddr-iBase] = iData;
    iBase += BASE_FLASH;

    #warning "review disable/enable interrupt too many for a simple write"
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
    #else
    EEDATA = (UINT8)iData;
    EEADR = iAddr;
    // start write sequence as described in datasheet, page 91
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.WREN = 1; // enable writes to data EEPROM
    GIE = 0;  // disable interrupts
    EECON2 = 0x55;
    EECON2 = 0x0AA;
    EECON1bits.WR = 1;   // start writing
    while(EECON1bits.WR){}
    //if(EECON1bits.WRERR){
    EECON1bits.WREN = 0;
    GIE = 1;  // enable interrupts
    #endif
}

#warning "readEEPROM should be return UINT8"
UINT16 readEEPROM(UINT8 iAddr)
{
    #if defined(_PIC14E)
    UINT16 iRealAddr;
    iRealAddr = BASE_FLASH + iAddr;

    PMCON1 = 0;     //not configuration space
    PMADRL = iRealAddr&0xFF;
    PMADRH = iRealAddr>>8;
    RD = 1;         //initiate read operation
    NOP();
    NOP();
    return (PMDATH<<8) | PMDATL;  //joins bytes & returns the value stored
    #else
    EEADR = iAddr;
    EECON1bits.CFGS = 0;
    EECON1bits.EEPGD = 0;
    EECON1bits.RD = 1;
    return EEDATA;
    #endif
}
