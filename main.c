/* PIN MAPPING
 *
 * RA0 USB D+ / ICSPDAT
 * RA1 USB D- / ICSPCLK
 * RA3 SPARE da vedere perchè è VPP
 * RA4 UP Key IN e Pull-up
 * RA5 DOWN Key IN e Pull-up
 * RC0 RS LCD
 * RC1 SPARE da vedere forse per RW LCD(inutile) o FAN Cooler
 * RC2 AN6 (battery voltage multiplied by R6/(R5+R6)) (analog IN)
 * RC3 AN7 (current pickup) (analog IN)
 * RC4 OK ENTER Key IN (Pull-up esterno)
 * RC5 PWM1 CHARGE OUT
 * RC6 PWM2 DISCHARGE OUT
 * RC7 E  LCD
 * RB4 D4 LCD
 * RB5 D5 LCD
 * RB6 D6 LCD
 * RB7 D7 LCD
 *
 * BUZZER mancante rispetto all'originale
 *
 */

#include <configwords.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "./USB/usb.h"
#include "./USB/usb_function_cdc.h"

#include "HardwareProfile.h"

#include <lcd.h>
#include "flash.h"

#define IDLE        0
#define DISCHARGECC 1
#define CHARGECC    2
#define CHARGECV    3

#define NICD 0
#define NIMH 1
#define LIPO 2
#define SLA  3

#define PROFILE_SIZE 10

// Address in EEPROM for each profile
#define CHEMISTRY  0    //parameters position in a specific profile: Chemistry
#define CAPACITY   1    //capacity in mAh/100, 255 -> 25500mAh
#define CELLS      2    //Number of cells
#define CHARGE     3    //charge in capacity units 255 -> 25.5*capacity
#define DISCHARGE  4    //discharge in capacity units 255 -> 25.5*capacity
#define INHIBIT    5    //Inhibit in minutes for delta peak check: 255 -> 255 min
#define CUTOFF     6    //Cutoff in discharge
#define CHARGE_CON 7    //deltav in charge for NiCd
                        //deltav in charge for NiMh
                        //max voltage in charge for LiPo
                        //max voltage in charge for SLA
#define FINALCURR  8    //final current (% of initial) for LiPo
                        //final current (% of initial) for SLA
#define TIMEOUT    9    //Timeout for charge

// 120..128 eeprom spare
#define PROFILE       119     //Position for the selected profile in EEPROM
#define MAXCHARGE     118     //Maximum allowable charge current 255 -> 255A
#define MAXDISCHARGE  117     //Maximum allowable discharge current 255 -> 255A
#define R6_H          116     //
#define R6_L          115     //R5=0..65535 in Ohm
#define R5_H          114     //
#define R5_L          113     //R6=0..65535 in Ohm
#define CURR_H        112     //
#define CURR_L        111     //current pick up sensitivity 0..268435456 -> 268435456uV/A
#define MODE          110     //idle, charge or discharge mode, see below constants
// 0..109 profiles PROFILE_SIZE*11 profiles

#define MODEIDLE      0
#define MODECHARGE    1
#define MODEDISCHARGE 2

#define MSGMAIN1      "Task select:    "
#define MSGMAIN2      "Profile select  "
#define MSGMAIN3      "Batt. charge    "
#define MSGMAIN4      "Batt. discharge "
#define MSGMAIN7      "Profile change  "
#define MSGMAIN8      "PC management   "
#define MSGMAIN9      "Volt calibr.    "
#define MSGMAIN10     "Ampere calibr.  "
#define MSGCHANGE1    "Chemistry:      "
#define MSGCHANGE2    "Battery type    "
#define MSGCHANGE3    "NiCd"
#define MSGCHANGE4    "NiMh"
#define MSGCHANGE5    "LiPo"
#define MSGCHANGE6    "SLA "
#define MSGCHANGE7    "Capacity:       "
#define MSGCHANGE8    "mAh per cell    "
#define MSGCHANGE9    "Cells:          "
#define MSGCHANGE10   "Number of cells "
#define MSGCHANGE11   "Charge:         "
#define MSGCHANGE12   "mult. for capac."
#define MSGCHANGE13   "Discharge:      "
#define MSGCHANGE15   "Name:           "
#define MSGPCMAN1     "PC serial link.."
#define MSGDISCHARGE  "Dsch."
#define MSGCHARGE     "Chrg."
#define MSGEND        "End!"
#define MSGDISPLAY    "Pack#"
#define MSGINIT       "Initializing... "
#define MSGHELLO1     "Universal       "
#define MSGHELLO2     "Charger         "

#define REPEATPERIOD 16

#define KEYDOWN PORTAbits.RA5
#define KEYUP PORTAbits.RA4
#define KEYENTER PORTCbits.RC4

#define REPEATSTART 200 // 5ms * 200
#define KEYPRESSED  10  // 5ms * 10

char asBuffIn[32];
char asBuffOut[32];

volatile INT16 iDutyPwmCharge;
volatile INT16 iDutyPwmDischarge;

INT8 iMenuPos;
volatile UINT8 iKeys;
volatile UINT8 iMSec, iMin, iSec;
volatile BOOL bCVControl;
volatile UINT8 iRepeatKey;
volatile BOOL bRepeatKey;

volatile UINT8 iRepeatK1,iRepeatK2,iRepeatK3;

volatile UINT16 iFastV,iFastC;
volatile UINT32 iAccV, iAccC;

volatile UINT8 iSlowCnt;
volatile UINT16 iSlowV, iSlowC;

volatile UINT8 iAction;                          //task to be performed: charge, discharge...

volatile UINT16 iZerC;
volatile UINT32 iMAh;

volatile UINT16 iTargC, iTargV;

volatile BOOL bPwmErr;

UINT8 iCurrProfile;
volatile UINT8 iPosOut;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static __inline void __attribute__((always_inline)) initSystem(void);
static __inline void __attribute__((always_inline)) processData(char);
void USBDeviceTasks(void);
void USBCBSendResume(void);
//void __inline mainMenu(void);
void static setPwm(void);
void lcdProfile(UINT8);
void selProfile(void);
UINT16 atoadu(UINT16);
UINT16 adutoa(UINT16);

void interrupt interruptCode()
{
    if(TMR0IF)
    {
        UINT16 iTmp;
        UINT8 iNdx;

        //RC1 = 1;
        GIE = 0;
        TMR0 = 22; // 4,992 (original 4,992) ms interrupt
        TMR0IF = 0;

        if(++iMSec==200)
        {
            iMSec = 0;
            if(++iSec==60)
            {
                iSec = 0;
                iMin++;
            }
        }
        if(bCVControl)
            bCVControl = FALSE;
        else
            bCVControl = TRUE;

        if(++iRepeatKey==REPEATPERIOD)
        {
            iRepeatKey = 0;
            bRepeatKey = TRUE;
        }
        else
            bRepeatKey = FALSE;
        if(!KEYDOWN)
        {
            if(iRepeatK1!=REPEATSTART)
            {
                if(++iRepeatK1==KEYPRESSED)
                    iKeys |= 0x01;
                if(iRepeatK1==REPEATSTART)
                    if(bRepeatKey)
                        iKeys |= 0x01;
            }
        }
        else
            iRepeatK1 = 0;
        if(!KEYUP)
        {
            if(iRepeatK2!=REPEATSTART)
            {
                if(++iRepeatK2==KEYPRESSED)
                    iKeys |= 0x02;
                if(iRepeatK2==REPEATSTART)
                    if(bRepeatKey)
                        iKeys |= 0x02;
            }
        }
        else
            iRepeatK2 = 0;
        if(!KEYENTER)
        {
            if(iRepeatK3!=REPEATSTART)
            {
                if(++iRepeatK3==KEYPRESSED)
                    iKeys |= 0x04;
                if(iRepeatK3==REPEATSTART)
                    if(bRepeatKey)
                        iKeys |= 0x04;
            }
        }
        else
            iRepeatK3 = 0;

        iTmp = 0;
        ADCON0 = 0x19; // AN6
        __delay_us(5);
        for(iNdx=0;iNdx<63;iNdx++)
        {
            ADCON0bits.GO = 1;
            while(ADCON0bits.GO);
            iTmp += ADRESL;
            iTmp += (ADRESH << 8);
        }
        iFastV = iTmp;
        iAccV += iTmp;

        iTmp = 0;
        ADCON0 = 0x1D; // AN7
        __delay_us(5);
        for(iNdx=0;iNdx<63;iNdx++)
        {
            ADCON0bits.GO = 1;
            while(ADCON0bits.GO);
            iTmp += ADRESL;
            iTmp += (ADRESH << 8);
        }
        iFastC = iTmp;
        iAccC += iTmp;

        if(++iSlowCnt==0)
        {
            iSlowV = iAccV >> 8;
            iSlowC = iAccC >> 8;
            iAccV = 0;
            iAccC = 0;
        }

        switch(iAction)
        {
            case DISCHARGECC:
            {
                INT16 iTmpD = iFastC - iZerC;
                if(iTmpD<0)
                    iTmpD = 0;
                iMAh += iTmpD;
                if(iTmpD>iTargC)
                {
                    if(--iDutyPwmDischarge<0)
                        iDutyPwmDischarge = 0;
                }
                else
                {
                    if(++iDutyPwmDischarge>1023)
                    {
                        iDutyPwmDischarge = 1023;
                        bPwmErr = TRUE;
                    }
                }
                break;
            }
            case CHARGECC:
            {
                INT16 iTmpD = iZerC - iFastC;
                if(iTmpD>0)
                    iMAh += iTmpD;
                if(iTmpD>iTargC)
                {
                    if(--iDutyPwmDischarge<0)
                        iDutyPwmDischarge = 0;
                }
                else
                {
                    if(++iDutyPwmDischarge>1023)
                    {
                        iDutyPwmDischarge = 1023;
                        bPwmErr = TRUE;
                    }
                }
                break;
            }
            case CHARGECV:
            {
                INT16 iTmpD = iZerC - iFastC;
                if(iTmpD>0)
                    iMAh += iTmpD;
                if(iTmpD>iTargC)
                {
                    if(--iDutyPwmDischarge<0)
                        iDutyPwmDischarge = 0;
                }
                else
                {
                    if(bCVControl)
                    {
                        if(iFastV>iTargV)
                        {
                            if(--iDutyPwmDischarge<0)
                                iDutyPwmDischarge = 0;
                        }
                    }
                    if(++iDutyPwmDischarge>1023)
                    {
                        iDutyPwmDischarge = 1023;
                        bPwmErr = TRUE;
                    }
                }
                break;
            }
            default:
                iDutyPwmCharge = 0;
                iDutyPwmDischarge = 0;
                break;
        }
        setPwm();

        // Trasmissione seriale dei dati
        if(iAction!=0)
        {
            /*iPosOut = 1;
            switch(iSlowCnt&0xF)
            {
                case 0:
                    asBuffOut[0] = 0x55;
                    break;
                case 1:
                    asBuffOut[0] = 0x56;
                    break;
                case 2:
                    asBuffOut[0] = 0x57;
                    break;
                case 3:
                    asBuffOut[0] = 0x58;
                    //if(iAction==1)
                    break;
                default:
                    iPosOut = 0;
                    break;
            }*/
        }
        USBDeviceTasks();
        if(USBDeviceState >= CONFIGURED_STATE && USBSuspendControl!=1)
        {
            if(USBUSARTIsTxTrfReady() && iPosOut>0)
                putUSBUSART(asBuffOut, iPosOut);
            CDCTxService();
        }

        GIE = 1;
        //RC1 = 0;
    }
}

int main(void)
{
    UINT8 iTmp = 0;

    initSystem();

    GIE = 1;
    iKeys = 0;
    while(!iKeys && iTmp++<255)
        __delay_ms(10);
    if(iTmp==255)
    {
        UINT8 iPrevMode = readFlash(MODE);
        if(iPrevMode==MODECHARGE)
        {
            //charge();
        }
        else if(iPrevMode==MODEDISCHARGE)
        {
            //discharge();
        }
    }
    writeFlash(MODE,MODEIDLE);
    iMenuPos = 0;
    while(TRUE)
    {
        do {
            lcdOut(128,MSGMAIN1);
            switch(iMenuPos)
            {
                case 0:
                    lcdOut(192,MSGMAIN2);
                    break;
                case 1:
                    lcdOut(192,MSGMAIN3);
                    break;
                case 2:
                    lcdOut(192,MSGMAIN4);
                    break;
                case 3:
                    lcdOut(192,MSGMAIN7);
                    break;
                case 4:
                    lcdOut(192,MSGMAIN8);
                    break;
                case 5:
                    lcdOut(192,MSGMAIN9);
                    break;
                case 6:
                    lcdOut(192,MSGMAIN10);
                    break;
            }
            iKeys = 0;
            while(!iKeys);
            if(iKeys & 0x01) // Down Key
            {
                iMenuPos--;
                if(iMenuPos<0)
                    iMenuPos = 0;
            }
            else if(iKeys & 0x02) // Up key
            {
                iMenuPos++;
                if(iMenuPos>=7)
                    iMenuPos = 6;
            }
        }while(!(iKeys & 0x04)); // if menu key is pressed exit
        switch(iMenuPos)
        {
            case 0:
                selProfile();
                break;
            case 1:
                // charge
                break;
            case 2:
                // discharge
                break;
            case 3:
                // changeprofile
                break;
            case 4:
                // pcmanage
                break;
            case 5:
                // calibration 0
                break;
            case 6:
                // calibration 1
                break;
        }
    }
}

static __inline void __attribute__((always_inline)) initSystem(void)
{
    UINT8 iTmp;

    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x0C; // RC2 RC3 analog

    TRISA = 0b110000; // RA4 RA5 input
    TRISB = 0;
    TRISC = 0b00011100; // RC2 RC3 RC4 input

    LATA = 0;
    LATB = 0;
    LATC = 0;

    OPTION_REG = 0b01010111; // Weak Pull-up, prescaler 256 assigned to TIMER0
    WPUA = 0b00110000; // RA4 e RA5 pull-up enabled for button

    OSCCON = 0xFC;  //16MHz HFINTOSC with 3x PLL enabled (48MHz operation)
    ACTCON = 0x90;  //Enable active clock tuning with USB

    while(!OSCSTATbits.HFIOFR);
    while(!OSCSTATbits.PLLRDY);

    PR2 = 0xFF;
    PWM1DCH = 0;
    PWM2DCH = 0;

    ADCON1 = 0xF0; // FRC & VDD

    INTCON = 0b01100000;

    iAction = 0;

    lcdConfig();
    lcdClear();

    iCurrProfile = 0;
    if((iTmp = (readFlash(PROFILE)&0xFF))==0xFF)
    {
        UINT8 iNdx;

        lcdOut(128,MSGINIT);
        writeFlash(PROFILE,0);           //Default profile selected (0) -> 1st profile
        writeFlash(MAXCHARGE,5);         //Default max charge current (5) -> 5A
        writeFlash(MAXDISCHARGE,5);      //Default max discharge current (20) -> 20A
        writeFlash(R5_H,0xB7);           //default (183)
        writeFlash(R5_L,0x98);           //default (152) R5=R5h*256+R5l = 47000 Ohm
        writeFlash(R6_H,0x2E);           //default (46)
        writeFlash(R6_L,0xE0);           //default (224) R6=R6h*256+R6l = 12000 Ohm
        writeFlash(CURR_H,97);           //default (97)
        writeFlash(CURR_L,168);          //default (168) Curr=Currh*256+Currl= 25000 -> 25000uv/A
        writeFlash(MODE,MODEIDLE);       //default mode (0) -> idle
        for(iNdx=0;iNdx<11;iNdx++)
        {
            UINT8 iNdx2;
            iNdx2 = iNdx*PROFILE_SIZE;

            writeFlash(iNdx2+CHEMISTRY,0);          //Default NiCd (0) -> 0:NiCd, 1:NiMh, 2:LiPo, 3:SLA
            writeFlash(iNdx2+CAPACITY,10);          //Default cells capacity (10) -> 10*100=1000mAh
            writeFlash(iNdx2+CELLS,1);              //Deafult number of cells (6) -> 6
            writeFlash(iNdx2+CHARGE,1);             //default charge (1) -> 1000*1.0=1A
            writeFlash(iNdx2+DISCHARGE,1);          //default discharge (1) -> 1000*1.0=10A
            writeFlash(iNdx2+INHIBIT,5);            //default deltapeak check inhibition (5) -> 5 minutes
            writeFlash(iNdx2+CUTOFF,10);            //default NiCd cutoff (80) -> 80*10=800mV
                                                    //default NiMh cutoff (100) -> 100*10=1000mV
                                                    //default LiPo cutoff (125) -> 2500+125*4=3000mV
                                                    //default SLA cutoff (125) -> 1500+125*4=2000mV
            writeFlash(iNdx2+CHARGE_CON,10);        //default NiCd deltapeak (10) -> 10mV
                                                    //default NiMh deltapeak (5) -> 5mV
                                                    //default LiPo max voltage (175) -> 3500+175*4=4200mV
                                                    //default SLA max voltage (125) -> 2000+125*4=2500mV
            writeFlash(iNdx2+FINALCURR,5);          //default LiPo final current (5) -> 3000*5/100=150mA
                                                    //default SLA final current (5) -> 3000*5/100=150mA
            writeFlash(iNdx2+TIMEOUT,120);          //default cell max charge (120) -> 3000*120/100=3600mAh
        }
    }
    else
        iCurrProfile = iTmp * PROFILE_SIZE;

    USBDeviceInit();
    lcdOut(128,MSGHELLO1);
    lcdOut(192,MSGHELLO2);
}

void static setPwm(void)
{
    PWM1DCL = (iDutyPwmCharge & 0x03) << 6;
    PWM1DCH = (iDutyPwmCharge >> 2) & 0xFF;

    PWM2DCL = (iDutyPwmDischarge & 0x03) << 6;
    PWM2DCH = (iDutyPwmDischarge >> 2) & 0xFF;
}

void lcdProfile(UINT8 iSelProf)
{
    UINT8 iTmp;
    char aiConvTmp[6];

    lcdOut(128,MSGDISPLAY);
    iTmp = iSelProf + 1;

    sprintf(aiConvTmp,"%2d",iTmp);
    lcdChar(133,aiConvTmp[0]);
    lcdChar(134,aiConvTmp[1]);
    lcdChar(135,':');
    iTmp = readFlash(iCurrProfile+CHEMISTRY);

    switch(iTmp)
    {
        case NICD:
            lcdOut(136,MSGCHANGE3);
            break;
        case NIMH:
            lcdOut(136,MSGCHANGE4);
            break;
        case LIPO:
            lcdOut(136,MSGCHANGE5);
            break;
        case SLA:
            lcdOut(136,MSGCHANGE6);
            break;
    }
    lcdChar(141,'x');

    iTmp = readFlash(iCurrProfile+CELLS);
    sprintf(aiConvTmp,"%2d",iTmp);
    lcdChar(142,aiConvTmp[0]);
    lcdChar(143,aiConvTmp[1]);

    iTmp = readFlash(iCurrProfile+CAPACITY);
    sprintf(aiConvTmp,"%5d",iTmp*100);
    lcdChar(192,'K');
    lcdChar(193,aiConvTmp[0]);
    lcdChar(194,aiConvTmp[1]);
    lcdChar(195,aiConvTmp[2]);
    lcdChar(196,aiConvTmp[3]);
    lcdChar(197,aiConvTmp[4]);

    iTmp = readFlash(iCurrProfile+CHARGE);
    sprintf(aiConvTmp,"%3d",iTmp);
    lcdChar(198,'C');
    lcdChar(199,aiConvTmp[0]);
    lcdChar(200,aiConvTmp[1]);
    lcdChar(201,'.');
    lcdChar(202,aiConvTmp[2]);

    iTmp = readFlash(iCurrProfile+DISCHARGE);
    sprintf(aiConvTmp,"%3d",iTmp);
    lcdChar(203,'D');
    lcdChar(204,aiConvTmp[0]);
    lcdChar(205,aiConvTmp[1]);
    lcdChar(206,'.');
    lcdChar(207,aiConvTmp[2]);
}

void selProfile()
{
    INT8 iSelProf;
    iSelProf = readFlash(PROFILE);
    do {
        lcdProfile(iSelProf);
        iKeys = 0;
        while(!iKeys);
        if(iKeys & 0x01)
        {
            iSelProf--;
            if(iSelProf<0)
                iSelProf = 0;
            iCurrProfile = iSelProf * PROFILE_SIZE;
        }
        else if(iKeys & 0x02)
        {
            iSelProf++;
            if(iSelProf>=11)
                iSelProf = 10;
            iCurrProfile = iSelProf * PROFILE_SIZE;
        }
    }while(!(iKeys & 0x04));
    writeFlash(PROFILE,iSelProf);
}

//TODO capire costante 7629
UINT16 atoadu(UINT16 i)
{
    UINT32 iCurr = readFlash(CURR_H) << 14 | readFlash(CURR_L);
    return (i * iCurr) / 7629;
}
//TODO capire costante 7629
UINT16 adutoa(UINT16 iADU)
{
    UINT32 iCurr = readFlash(CURR_H) << 14 | readFlash(CURR_L);
    return (iADU * 7629) / iCurr;
}

//TODO capire costante 625
UINT16 mvtoadu(UINT16 iMv)
{
    UINT16 iTmp, iTmp2;
    UINT16 iR5 = readFlash(R5_H) << 8 | readFlash(R5_L);
    UINT16 iR6 = readFlash(R6_H) << 8 | readFlash(R6_L);
    iTmp = iR6 << 12;
    iTmp2 = iR5 + iR6;
    iTmp /= iTmp2;
    iTmp = (iMv * iTmp) << 1;
    return iTmp/625;
}

// TODO capire costante 53687
UINT16 adutomv(UINT16 iADU)
{
    UINT16 iTmp;
    UINT16 iR5 = readFlash(R5_H) << 8 | readFlash(R5_L);
    UINT16 iR6 = readFlash(R6_H) << 8 | readFlash(R6_L);
    iTmp = iR5 << 12;
    iTmp = (4096+(iTmp/iR6)) * iADU;
    return iTmp/53687;
}

// TODO capire costante 6944
UINT16 recallmah()
{
    UINT32 iCurr = readFlash(CURR_H) << 14 | readFlash(CURR_L);

    return ((iMAh >> 16) * 6944) / iCurr;
}
