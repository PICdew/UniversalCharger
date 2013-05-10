#include <lcd.h>

#define LCD_RS PORTCbits.RC0
#define LCD_E  PORTCbits.RC7

#define CMD 0
#define DATA 1

static void lcd4(UINT8,UINT8);
static void lcd8(UINT8);

static void lcd4(UINT8 iData,UINT8 iRS)
{
    LCD_RS = iRS; // 0 command, 1 data

    PORTB = iData & 0xF0;
    LCD_E = 1;
    LCD_E = 0;
    PORTB = (iData & 0x0F) << 4;
    LCD_E = 1;
    LCD_E = 0;
    __delay_ms(1);
}

static void lcd8(UINT8 iData)
{
    LCD_RS = 0; // command

    PORTB = iData & 0xF0;
    LCD_E = 1;
    LCD_E = 0;
    __delay_ms(1);
}

void lcdClear(void)
{
    lcd4(1,CMD);
    __delay_ms(2);
}

void __inline lcdConfig(void)
{
    UINT8 iNdx;

    __delay_ms(1);
    lcd8(0x30);
    __delay_ms(10);
    lcd8(0x30);
    __delay_ms(10);
    lcd8(0x30);
    __delay_ms(10);
    lcd8(0x20);
    __delay_ms(10);
    lcd4(0x28,CMD);
    lcd4(0x0C,CMD);
    lcd4(6,CMD);
    lcdClear();
    for(iNdx=0;iNdx<25;iNdx++)
        __delay_ms(10);
    lcd4(0x80,CMD);
    lcd4(32,DATA);
}

void lcdOut(UINT8 iCmd, const char *sMsg)
{
    UINT8 iNdx;

    lcd4(iCmd,CMD);
    for(iNdx=0;sMsg[iNdx];iNdx++)
        lcd4(sMsg[iNdx],DATA);
}

void lcdChar(UINT8 iCode,char iChar)
{
    if(!iChar)
        iChar = ' ';
    lcd4(iCode,CMD);
    lcd4(iChar,DATA);
}
