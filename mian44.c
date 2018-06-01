#include <stdio.h>
#include <stdlib.h>
#include <xc.h>

// CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled and controlled by software (SBOREN is enabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#define lcd_clear()lcd_command(1)
#define lcd_origin()lcd_command(2)
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 40000000
#endif
#define E_pulse_with 50
#define LCD_E PORTDbits.RD3
#define LCD_RS PORTDbits.RD2
#define LCD_Data4 PORTD
void lcd_clk(void)
{
    LCD_E = 1;
    __delay_us (E_pulse_with);
    LCD_E = 0;
    __delay_us(E_pulse_with);
}
void lcd_command(unsigned char outbyte)
{
    LCD_RS = 0;
    LCD_Data4 = (LCD_Data4&0x0f)|(outbyte&0xf0);
    lcd_clk();
    LCD_Data4 = (LCD_Data4&0x0f)|((outbyte<<4)&0xf0);
    lcd_clk();
    __delay_ms(1);
}
void lcd_init()
{
    TRISD &= 0x03;
    LCD_Data4 &= 0b00001111;
    LCD_E = 0;
    LCD_RS = 0;
    __delay_ms(1);
    
    LCD_Data4=(LCD_Data4&0x0f)|0x30;
    lcd_clk();
    __delay_ms(1);
    
    LCD_Data4=(LCD_Data4&0x0f)|0x30;
    lcd_clk();
    __delay_ms(1);
    
    LCD_Data4=(LCD_Data4&0x0f)|0x30;
    lcd_clk();
    __delay_ms(1);
   
    LCD_Data4=(LCD_Data4&0x0f)|0x20;
    lcd_clk();
    __delay_ms(1);
    lcd_command(0x28);
    lcd_command(0x01);
    lcd_command(0x06);
    lcd_command(0x0C);
    lcd_command(0x02);
    lcd_command(0x01);
}
void lcd_putc  (char outbyte)
{
    LCD_RS = 1;
    LCD_Data4 = (LCD_Data4&0x0f)|(outbyte&0xf0);
    lcd_clk();
    LCD_Data4 = (LCD_Data4&0x0f)|((outbyte<<4)&0xf0);
    lcd_clk();
}
// Вывод строки на экран
void lcd_puts(unsigned char line, const char *p)
{
    lcd_origin();
    lcd_command(line);
    while(*p)
    {
        lcd_putc(*p);
        p++;  
    }
}
// Вывод на экран значений скважности
void inttolcd(unsigned char posi, long value)
{
    char buff[16];
    itoa(buff,value,10);
    lcd_puts(posi,buff);
}
// Инициализация АЦП преобразователя
void Adc_init()
{
    TRISA|=0b00101111;
    TRISE|=0b00000111;
    ADCON1bits.PCFG=0b0111;
    ADCON1bits.VCFG=0b00;
    ADCON2bits.ACQT=0b111;
    ADCON2bits.ADCS=0b110;
    ADCON2bits.ADFM=0;
    ADCON0bits.ADON=1;
}
// Чтение с АЦП преобразователя
int read_Adc(unsigned char k) //Чтение данных с k-го датчика
{   
    ADCON0bits.CHS=k;
    ADCON0bits.GO_DONE=1;
    while(ADCON0bits.GO_DONE==1);
    return(ADRESH<<2)+(ADRESL>>6); 
}
// Инициализация управления двигателями
void motor_init()
{
    TRISDbits.RD0=0;
    TRISDbits.RD1=0;
    TRISBbits.RB1=0;
    TRISBbits.RB2=0;
    TRISCbits.TRISC1=0;
    TRISCbits.TRISC2=0;
    CCP1CONbits.CCP1M=0b1100;
    CCP1CONbits.P1M=0b00;
    CCP2CONbits.CCP2M=0b1111;
    CCPR1L=0;
    CCPR2L=0;
    PR2=124;
    T2CONbits.T2CKPS=0b00;
    T2CONbits.TMR2ON=1;
}
// Управление двигателем А
void motor_a_change_Speed(signed char speed)
{
    if ( speed > 0 )
    {
        CCPR1L=speed;
        PORTDbits.RD0=0;
        PORTDbits.RD1=1;
    }
    else if (speed < 0)
    {
        CCPR1L=-speed;
        PORTDbits.RD0=1;
        PORTDbits.RD1=0;
    }
    else
    {
        CCPR1L=0;
        PORTDbits.RD0=0;
        PORTDbits.RD1=0;
    }
}
// Управление двигателем Б
void motor_b_change_Speed(signed char speed)
{
    if ( speed > 0 )
    {
        CCPR2L=speed;
        PORTBbits.RB1=0;
        PORTBbits.RB2=1;
    }
    else if (speed < 0)
    {
        CCPR2L=-speed;
        PORTBbits.RB1=1;
        PORTBbits.RB2=0;
    }
    else
    {
        CCPR2L=0;
        PORTBbits.RB1=0;
        PORTBbits.RB2=0;
    }
}

void inttolcd2(unsigned char posi, float value) 
{ 
char* buff; 
int status; 
buff = ftoa(value,&status); 
lcd_puts(posi,buff); 
}

void main(void) 
{   
    int potenc;
    Adc_init();
    motor_init();
    lcd_init();
    lcd_puts(128, "CKBA}1{HOCTb =");
    while(1)
    {
        potenc=read_Adc(7)/4.125-124;
        inttolcd2(192, potenc);
        motor_a_change_Speed(potenc);
        motor_b_change_Speed(potenc);
        if(read_Adc(3)>=300)
        {
        motor_a_change_Speed(-potenc);
        motor_b_change_Speed(potenc); 
        }
    }
}

