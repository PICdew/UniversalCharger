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

#include "./USB/usb.h"
#include "./USB/usb_function_cdc.h"

#include "HardwareProfile.h"
#include "flash.h"

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover Mode (Internal/External Switchover Mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection Bit (NO CPU system divide)
#pragma config USBLSCLK = 48MHz // USB Low SPeed Clock Selection bit (System clock expects 48 MHz, FS/LS USB CLKENs divide-by is set to 8.)
#pragma config PLLMULT = 3x     // PLL Multipler Selection Bit (3x Output Frequency Selected)
#pragma config PLLEN = ENABLED  // PLL Enable Bit (3x or 4x PLL Enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

/** I N C L U D E S **********************************************************/

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "USB/usb_device.h"
#include "USB/usb.h"

#include "HardwareProfile.h"

#include <lcd.h>

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
#define KEYPRESSED 10 // 5ms * 10

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

// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA* each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

// Note *: The "usb_20.pdf" specs indicate 500uA or 2.5mA, depending upon device classification. However,
// the USB-IF has officially issued an ECN (engineering change notice) changing this to 2.5mA for all 
// devices.  Make sure to re-download the latest specifications to get all of the newest ECNs.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
}

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *					
 *					This call back is invoked when a wakeup from USB suspend 
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and 
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific 
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckCDCRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    //Enable the CDC data endpoints
    CDCInitEP();
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function will only be able to wake up the host if
 *                  all of the below are true:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behavior, 
 *                  as a USB device that has not been armed to perform remote 
 *                  wakeup must not drive remote wakeup signalling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *                  
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are 
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex: 
 *                  anytime application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup. 
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in 
 *                  USBCBWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    //First verify that the host has armed us to perform remote wakeup.
    //It does this by sending a SET_FEATURE request to enable remote wakeup,
    //usually just before the host goes to standby mode (note: it will only
    //send this SET_FEATURE request if the configuration descriptor declares
    //the device as remote wakeup capable, AND, if the feature is enabled
    //on the host (ex: on Windows based hosts, in the device manager 
    //properties page for the USB device, power management tab, the 
    //"Allow this device to bring the computer out of standby." checkbox 
    //should be checked).
    if(USBGetRemoteWakeupStatus() == TRUE) 
    {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if(USBIsBusSuspended() == TRUE)
        {
            USBMaskInterrupts();
            
            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0; 
            USBBusIsSuspended = FALSE;  //So we don't execute this code again, 
                                        //until a new suspend condition is detected.

            //Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
            //device must continuously see 5ms+ of idle on the bus, before it sends
            //remote wakeup signalling.  One way to be certain that this parameter
            //gets met, is to add a 2ms+ blocking delay here (2ms plus at 
            //least 3ms from bus idle to USBIsBusSuspended() == TRUE, yeilds
            //5ms+ total delay since start of idle).
            delay_count = 3600U;
            do {
                delay_count--;
            }while(delay_count);
            
            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signaling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do {
                delay_count--;
            }while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}


/*******************************************************************
 * Function:        void USBCBEP0DataReceived(void)
 *
 * PreCondition:    ENABLE_EP0_DATA_RECEIVED_CALLBACK must be
 *                  defined already (in usb_config.h)
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called whenever a EP0 data
 *                  packet is received.  This gives the user (and
 *                  thus the various class examples a way to get
 *                  data that is received via the control endpoint.
 *                  This function needs to be used in conjunction
 *                  with the USBCBCheckOtherReq() function since 
 *                  the USBCBCheckOtherReq() function is the apps
 *                  method for getting the initial control transfer
 *                  before the data arrives.
 *
 * Note:            None
 *******************************************************************/
#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)
void USBCBEP0DataReceived(void)
{
}
#endif

/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        int event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           int event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            //Add application specific callback task or callback function here if desired.
            //The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
            //FEATURE (endpoint halt) request on an application endpoint which was 
            //previously armed (UOWN was = 1).  Here would be a good place to:
            //1.  Determine which endpoint the transaction that just got terminated was 
            //      on, by checking the handle value in the *pdata.
            //2.  Re-arm the endpoint if desired (typically would be the case for OUT 
            //      endpoints).
            break;
        default:
            break;
    }      
    return TRUE; 
}
/** EOF main.c *************************************************/
