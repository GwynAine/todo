/*
 * File:   main4.c
 * Author: user
 *
 * Created on 30 мая 2018 г., 17:02
 */


#include <xc.h>
#include <stdlib.h>

// PIC18F4525 Configuration Bit Settings

// 'C' source line config statements

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

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define lcd_clear() lcd_command(1)
#define lcd_origin() lcd_command(2)
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 40000000 // определение тактовой частоты  40 МГц  для функции __delay_ms
#endif
#define E_pulse_with 50 //длительность тактовых импульсов LCD в мкс
#define LCD_E PORTDbits.RD3
#define LCD_RS PORTDbits.RD2
#define LCD_Data4 LATD // инициализация портов D4-D7 LCD


void Adc_init()
    {
    TRISA|=0b00101111; //перевод выводов RA0, RA1, RA2, RA3, RA5 в режим входов
    TRISE|=0b00000111; //перевод выводов RE0, RE1, RE2 в режим входов
    ADCON1bits.PCFG=0b0111; // конфигурация аналого-цифровых портов
    ADCON1bits.VCFG=0b00; // опорное напряжение Vss Vdd
    ADCON2bits.ACQT=0b111;// время преобразования 20 Tad
    ADCON2bits.ADCS=0b110;// частота преобразования Fosc/64
    ADCON2bits.ADFM=0;//левое смещение
    ADCON0bits.ADON=1;   // модуль АЦП включен
     }

int read_Adc(char channel)
    {
    ADCON0bits.CHS=channel; // выбор аналогового канала
    ADCON0bits.GO_DONE=1; // запуск преобразования
    while(ADCON0bits.GO_DONE==1);     
    return (ADRESH<<2)+(ADRESL>>6);//запись результата преобразования
    }


void lcd_clk(void) /*генерация импульса на вход EN*/
{
  LCD_E = 1;
  __delay_us(E_pulse_with);
  LCD_E = 0;
  __delay_us(E_pulse_with);
}
void lcd_command(unsigned char outbyte) /*отправить команду (4-битный ре-жим работы) */
{
  LCD_RS=0; //режим передачи команды
  LCD_Data4=(LCD_Data4&0x0f)|(outbyte&0xf0); // отправка старших четырех бит
  lcd_clk();
  LCD_Data4=(LCD_Data4&0x0f)|((outbyte<<4)&0xf0); // отправка младших че-тырех бит
  lcd_clk();
  __delay_ms(1);
}
void lcd_putc(char outbyte) /* отправить данные (4-битная операция) */
{
  LCD_RS=1; //режим передачи данных
  LCD_Data4=(LCD_Data4&0x0f)|(outbyte&0xf0);
  lcd_clk();
  LCD_Data4=(LCD_Data4&0x0f)|((outbyte<<4)&0xf0);
  lcd_clk();
}
void lcd_puts(unsigned char line,const char *p) // *вывод строки на экран*
{
	lcd_origin();         // переход к нулевому адресу LCD
	lcd_command(line);			// установить адрес LCD 00H
	while(*p)                  // проверить, равен ли указатель 0
	{
	 lcd_putc(*p);             // отправить данные на LCD
	 p++;                      // увеличить адрес на 1
	}
    lcd_putc(' ');
}
void inttolcd(unsigned char posi, long value) //вывод на экран значений пере-менных
{
	char buff[16];
	itoa(buff,value,10);
	lcd_puts(posi,buff);
}

void lcd_init() // инициализация LCD-дисплея
{
  TRISD &= 0x03;// перевод выводов RD4-RD7в режим выходов
  LCD_Data4 &= 0b00001111;//установка нулей на линии передачи данных
  LCD_E=0;
  LCD_RS=0;
  __delay_ms(1);
/*инициализация дисплея*/
  LCD_Data4=(LCD_Data4&0x0f)|0x30;
  lcd_clk();
 __delay_ms(1);
  LCD_Data4=(LCD_Data4&0x0f)|0x30;
 lcd_clk();
__delay_ms(1);
  LCD_Data4=(LCD_Data4&0x0f)|0x30;
  lcd_clk();
  __delay_ms(1);
/*---------------------------------*/
  LCD_Data4=(LCD_Data4&0x0f)|0x20;	// переключить на 4-битный режим пе-ре-дачи
  lcd_clk();
  __delay_ms(1);
  lcd_command(0x28);//установить N=1, F=0 (две строки, размер символа 5*8 точек
  lcd_command(0x01);	// очистить всё
  lcd_command(0x06);	// автоматически передвинуть курсор после байта
  lcd_command(0x0C);	// дисплей включён, курсора нет, не моргает
  lcd_command(0x02);	// начальная позиция
  lcd_command(0x01);	// очистить всё снова
}

void motor_init()
{
TRISDbits.RD0=0;
TRISDbits.RD1=0;
TRISBbits.RB1=0;
TRISBbits.RB2=0;
TRISCbits.TRISC1=0;
TRISCbits.TRISC2=0;
CCP1CONbits.CCP1M=0b1100; //задаём режим работы модуля CCP1 (ШИМ)
CCP1CONbits.P1M=0b00; //задействован только один вывод P1A
CCP2CONbits.CCP2M=0b1111; // задаём режим работы модуля CCP2(ШИМ)
CCPR1L=0; // задаём нулевую скважность
CCPR2L=0; // задаём нулевую скважность
PR2=124;// задаём период ШИМ
T2CONbits.T2CKPS=0b00; //задаём предделитель модуля Timer2 равным 1
T2CONbits.TMR2ON=1;// включение модуля Timer2
}

void straight (void)
{
    CCPR1L=124;
    CCPR2L=124;
    PORTDbits.RD0=1;
    PORTDbits.RD1=0;   
    
    PORTBbits.RB2=0;    
    PORTBbits.RB1=1; 
}

void left (void)
{
    CCPR1L=124;
    CCPR2L=124;
    PORTDbits.RD0=0;
    PORTDbits.RD1=1;   
    
    PORTBbits.RB2=0;    
    PORTBbits.RB1=1; 
}

void right (void)
{
    CCPR1L=124;
    CCPR2L=124;
    PORTDbits.RD0=1;
    PORTDbits.RD1=0;   
    
    PORTBbits.RB2=1;    
    PORTBbits.RB1=0; 
}

void main(void)
{
   motor_init();
   Adc_init();
   lcd_init();
  
   while(1)
    {
        inttolcd(0x80,3);
        inttolcd(0x83,read_Adc(0)); //180 473 963
        inttolcd(0x87,read_Adc(1));
        inttolcd(0xc0,read_Adc(2));
       if(read_Adc(0)<700)
           left();
       else if
         (read_Adc(1)<700)
           right();
       else
           straight();
    }
}
