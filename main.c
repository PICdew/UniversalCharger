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

#define IDLE 0
#define DISCHARGECC 1
#define CHARGECC 2
#define CHARGECV 3

#define NICD 0
#define NIMH 1
#define LIPO 2
#define SLA 3

#define MODEIDLE 0
#define MODECHARGE 1
#define MODEDISCHARGE 2

#define MSGMAIN1 "Task select:    "
#define MSGMAIN2 "Profile select  "
#define MSGMAIN3 "Batt. charge    "
#define MSGMAIN4 "Batt. discharge "
#define MSGMAIN7 "Profile change  "
#define MSGMAIN8 "PC management   "
#define MSGMAIN9 "Volt calibr.    "
#define MSGMAIN10 "Ampere calibr.  "
#define MSGCHANGE1 "Chemistry:      "
#define MSGCHANGE2 "Battery type    "
#define MSGCHANGE3 "NiCd"
#define MSGCHANGE4 "NiMh"
#define MSGCHANGE5 "LiPo"
#define MSGCHANGE6 "SLA "
#define MSGCHANGE7 "Capacity:       "
#define MSGCHANGE8 "mAh per cell    "
#define MSGCHANGE9 "Cells:          "
#define MSGCHANGE10 "Number of cells "
#define MSGCHANGE11 "Charge:         "
#define MSGCHANGE12 "mult. for capac."
#define MSGCHANGE13 "Discharge:      "
#define MSGCHANGE15 "Name:           "
#define MSGPCMAN1 "PC serial link.."
#define MSGDISCHARGE "Dsch."
#define MSGCHARGE "Chrg."
#define MSGEND "End!"
#define MSGDISPLAY "Pack#"
#define MSGINIT "Initializing... "

#define REPEATPERIOD 16

#define KEYDOWN PORTAbits.RA5
#define KEYUP PORTAbits.RA4
#define KEYENTER PORTCbits.RC4

#define REPEATSTART 200 // 5ms * 200
#define KEYPRESSED  10  // 5ms * 10

char asBuffIn[CDC_DATA_IN_EP_SIZE];
char asBuffOut[CDC_DATA_OUT_EP_SIZE];

volatile INT16 iDutyPwmCharge;
volatile INT16 iDutyPwmDischarge;

UINT8 iMenuPos;
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

/** P R I V A T E  P R O T O T Y P E S ***************************************/
static __inline void __attribute__((always_inline)) initSystem(void);
static __inline void __attribute__((always_inline)) processData(char);
void USBDeviceTasks(void);
void USBCBSendResume(void);
void __inline mainMenu(void);
void static setPwm(void);

void interrupt ISRCode()
{
    if(PIR1bits.TMR1IF)
    {
        UINT16 iTmp;
        UINT8 iNdx;

        TMR0 = 0xEA; // 4,991 (original 4,992) ms interrupt
        PIR1bits.TMR1IF = 0;

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
                    iKeys |= 0x03;
                if(iRepeatK3==REPEATSTART)
                    if(bRepeatKey)
                        iKeys |= 0x03;
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
        /*if(iAction<>0)
        {
            char sTrasm;
            switch(iSlowCnt&0xF)
            {
                case 0:
                    sTrasm = 0x55;
                    break;
                case 1:
                    sTrasm = 0x55;
                    break;
                case 2:
                    sTrasm = 0x55;
                    break;
                case 3:
                    sTrasm = 0x55;
                    if(iAction==1)

                    break;
            }
        }*/
    }
    #if defined(USB_INTERRUPT)
    USBDeviceTasks();
    #endif
}

int main(void)
{
    UINT8 iTmp = 0;

    initSystem();

    iKeys = 0;
    while(!iKeys && iTmp++<255)
        __delay_ms(10);
    if(iTmp==255)
    {
        UINT8 iPhase = 0; //read_eeprom();
        /*
         * read eeprom last action */
        if(iPhase==MODECHARGE)
        {}    //charge();
        else if(iPhase==MODEDISCHARGE)
        {}    //discharge();
    }
    //eepromWrite(,MODEIDLE);
    iMenuPos = 0;
    while(TRUE)
    {
        mainMenu();
        switch(iMenuPos)
        {
            case 0:
                // sel profile
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
        if(USBDeviceState < CONFIGURED_STATE || USBSuspendControl==1)
            continue;
        #if defined(USB_INTERRUPT)
        if(USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE))
            USBDeviceAttach();
        #endif

        #if defined(USB_POLLING)
        USBDeviceTasks();
        #endif
    }
}

static __inline void __attribute__((always_inline)) initSystem(void)
{
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

    OSCTUNE = 0;
    OSCCON = 0xFC;  //16MHz HFINTOSC with 3x PLL enabled (48MHz operation)
    ACTCON = 0x90;  //Enable active clock tuning with USB

    PR2 = 0xFF;
    PWM1DCH = 0;
    PWM2DCH = 0;

    ADCON1 = 0xF0; // FRC & VDD

    lcdConfig();

    // Controllare la flash dedicata ad EEPROM e se non è inizializzata
    // scrivere i valori di default
    /*
       j:=eeprom_read(uni_profile);
       if j=255 then                                                              //if the EEPROM is unprogrammed, program for defaults
         begin                                                                    //writing 255 to the profile selection address the default is reloaded
           lcd_df_out(128,@msg_init);
           default_values_uni;
           for k:=0 to 11 do default_values_pack;
         end
       else profile_point:=j*profilelen;                                          //else restore the current profile
    */

    /*
     * Display welcome message
    lcdOut(128,);
    lcdOut(192,); */

    USBDeviceInit();
}

void static setPwm(void)
{
    PWM1DCL = (iDutyPwmCharge & 0x03) << 6;
    PWM1DCH = (iDutyPwmCharge >> 2) & 0xFF;

    PWM2DCL = (iDutyPwmDischarge & 0x03) << 6;
    PWM2DCH = (iDutyPwmDischarge >> 2) & 0xFF;
}

void __inline mainMenu(void)
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
        if(iKeys & 0x01)
        {
            iMenuPos--;
            if(iMenuPos==0xff)
                iMenuPos = 0;
        }
        else if(iKeys & 0x02)
        {
            iMenuPos++;
            if(iMenuPos==7)
                iMenuPos = 6;
        }
    }while(!(iKeys & 0x03)); // if menu key is pressed exit
}

/** EOF main.c *************************************************/
