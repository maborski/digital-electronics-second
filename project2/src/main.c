/***********************************************************************
 * 
 * Stopwatch by Timer/Counter2 on the Liquid Crystal Display (LCD)
 *
 * ATmega328P (Arduino Uno), 16 MHz, PlatformIO
 *
 * Copyright (c) 2017 Tomas Fryza
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 * 
 * Components:
 *   16x2 character LCD with parallel interface
 *     VSS  - GND (Power supply ground)
 *     VDD  - +5V (Positive power supply)
 *     Vo   - (Contrast)
 *     RS   - PB0 (Register Select: High for Data transfer, Low for Instruction transfer)
 *     RW   - GND (Read/Write signal: High for Read mode, Low for Write mode)
 *     E    - PB1 (Read/Write Enable: High for Read, falling edge writes data to LCD)
 *     D3:0 - NC (Data bits 3..0, Not Connected)
 *     D4   - PD4 (Data bit 4)
 *     D5   - PD5 (Data bit 5)
 *     D6   - PD6 (Data bit 3)
 *     D7   - PD7 (Data bit 2)
 *     A+K  - Back-light enabled/disabled by PB2
 * 
 **********************************************************************/
/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <lcd.h>            // Peter Fleury's LCD library
#include <stdlib.h>         // C library. Needed for number conversions

/* Defines ----------------------------------------------------------*/ 
#define VRX PC0     //PC0 is pin where joystick x axis is connected
#define VRY PC1     //PC1 is pin where joystick y axis is connected
#define M1 PB1      //PB1 is pin where servo motor 1 is connected
#define M2 PB2      //PB2 is pin where servo motor 2 is connected

#define M_default 1500
#define M_left 600
#define M_right 2400
#define M_step 1500


uint16_t value = 0;
uint32_t M1_pos = M_default;
uint32_t M2_pos = M_default;

/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Update stopwatch value on LCD screen when 8-bit 
 *           Timer/Counter2 overflows.
 * Returns:  none
 **********************************************************************/
int main(void)
{   

    lcd_init(LCD_DISP_ON);
    lcd_gotoxy(0, 0);

    GPIO_mode_output(&DDRB, M1);
    GPIO_mode_output(&DDRB, M2);
    
    // Configure Analog-to-Digital Convertion unit
    // Select ADC voltage reference to "AVcc with external capacitor at AREF pin"

    ADMUX |= (1<<REFS0);
    ADMUX &= ~(1<<REFS1);

    // Enable ADC module
    ADCSRA |= (1<<ADEN);
    // Enable conversion complete interrupt
    ADCSRA |= (1<<ADIE);
    // Set clock prescaler to 128
    ADCSRA |= (1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0);

    TCCR1A &= ~((1 << COM1A0) | (1 << COM1B0));
    TCCR1A |= (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
    TCCR1B |= (1 << WGM13);

    ICR1 = 20000;
    OCR1A = M1_pos;
    OCR1B = M2_pos;
    
    TCCR1B |= (1 << CS11) | (1 << CS10);
    //PCICR |= (1<<PCIE0);
    //PCMSK0 |= (1<< PCINT0);
    


    // Configure 16-bit Timer/Counter1 to start ADC conversion
    // Set prescaler to 16 ms and enable overflow interrupt    
    TIM0_overflow_16ms();
    TIM0_overflow_interrupt_enable();
    
    sei();

    // Infinite loop
    while (1)
    {
        /* Empty loop. All subsequent operations are performed exclusively 
         * inside interrupt service routines ISRs */
    }

    // Will never reach this
    return 0;
}


/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Use single conversion mode and start conversion every 100 ms.
 **********************************************************************/
ISR(TIMER0_OVF_vect)
{
    static int8_t noofoverflow = 0;
    noofoverflow++;
    if(noofoverflow > 50)
    {
        noofoverflow = 0;
        ADCSRA |= (1 << ADSC);
    }

    /*if(value == 0)
    {
        ADMUX &= ~((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3)); 
    }
    else if(value == 1)
    {
        ADMUX &= ~((1<<MUX1) | (1<<MUX2) | (1<<MUX3)); ADMUX |= (1<<MUX0);
    }*/
}

ISR(ADC_vect)
{
    static uint16_t xValue;
    static uint16_t yValue;
    char string[4];  // String for converted numbers by itoa()

    // Read converted value
    // Note that, register pair ADCH and ADCL can be read as a 16-bit value ADC
    if (value == 0)
    {
        xValue = ADC;
        value = 1;
    }
    else if (value == 1)
    {
        
        yValue = ADC;
        value = 0;
    }

    lcd_gotoxy(0,1);
    lcd_puts("x");
    itoa(xValue, string, 10);
    lcd_gotoxy(1, 1);
    lcd_puts("    ");
    lcd_gotoxy(1, 1);
    lcd_puts(string);

    lcd_gotoxy(6,1);
    lcd_puts("y");
    itoa(yValue, string, 10);
    lcd_gotoxy(7, 1);
    lcd_puts("    ");
    lcd_gotoxy(7, 1);
    lcd_puts(string);

    if (xValue < 300 && M1_pos >= M_left)
    {
        
        M1_pos -= M_step;
        M2_pos -= M_step;
        ICR1 -= 500;
        lcd_gotoxy(11, 1);
        lcd_puts("     ");
        lcd_gotoxy(11,1);
        lcd_puts("LEFT ");
    }
    else if (xValue > 800 && M1_pos <= M_right)
    {
        M1_pos += M_step;
        M2_pos += M_step;
        ICR1 +=500;
        lcd_gotoxy(11, 1);
        lcd_puts("     ");
        lcd_gotoxy(11,1);
        lcd_puts("RIGHT");
    }
    else if (yValue < 100)
    {

        lcd_gotoxy(11, 1);
        lcd_puts("     ");
        lcd_gotoxy(11,1);
        lcd_puts("DOWN ");
    }
    else if (yValue > 900)
    {

        lcd_gotoxy(11, 1);
        lcd_puts("     ");
        lcd_gotoxy(11,1);
        lcd_puts("UP   ");
    }
    else
    {
        lcd_gotoxy(11, 1);
        lcd_puts("NONE ");
    }
    OCR1A = M1_pos;
    OCR1B = M2_pos;

}