/*
 * File:   main3.c
 * Author: user
 *
 * Created on 4 мая 2018 г., 18:15
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

int t1me=0;
unsigned char pr1nt=1;
unsigned char stp=1;

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

void interrupt HIisr (void)
{
    if (INTCONbits.INT0IF)
    {               
        __delay_ms(30);// задержка для исключения влияния «дребезга» контак-тов
        if (PORTBbits.RB0==0)
            {              
            T0CONbits.TMR0ON=!T0CONbits.TMR0ON;
            pr1nt=!pr1nt;  
            if(pr1nt)
                inttolcd(0xc0, t1me--);
           
            }        
    INTCONbits.INT0IF=0;
    }
}

void interrupt low_priority  LIisr (void)
{
    if (INTCONbits.TMR0IF)
        {        
        TMR0H=255;
        TMR0L=216;       
        ++t1me;
        INTCONbits.TMR0IF=0;
        }
}
    

void main(void) {
    TRISBbits.RB0=1;//настройка RB0 на вход
    RCONbits.IPEN=1; // разрешение двухуровневых прерываний
    INTCON2bits.INTEDG0=0; //прерывание по ниспадающему фронту сигнала на входе INT0
    INTCONbits.INT0IF=0; //обнуление флага прерывания от внешнего источни-ка
    INTCONbits.GIEH=1; //разрешение высокоуровневых прерываний
    INTCONbits.INT0IE=1;//разрешение прерывания от внешнего источника
    T0CONbits.T08BIT=0;//настройка таймера №0 на 16-битный режим работы   
    
    T0CONbits.PSA=0;//разрешение использовать предделитель
    T0CONbits.T0PS=0b111;//предделитель равен 256
    T0CONbits.T0CS=0;//выбор внутреннего источника тактовых импульсов
    TMR0H=103;//запись старшего байта начального значения
    TMR0L=106;//запись младшего байта начального значения
    T0CONbits.TMR0ON=0;
    INTCON2bits.TMR0IP=0;//присвоение прерыванию низкого приоритета
    INTCONbits.TMR0IF=0;//обнуление флага прерывания по переполнению таймера 0
    INTCONbits.GIEL=1; //разрешение низкоуровневых прерываний
    INTCONbits.TMR0IE=1;//разрешение прерывания по переполнению таймера 0
    TMR0H=255;
    TMR0L=216;
    
    lcd_init();
    while(1)        
        inttolcd(0x80, t1me);        
}   
