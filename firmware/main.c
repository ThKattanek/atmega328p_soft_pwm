/* Name: main.c
 * Projekt: multi_led_controller
 * Author: Thorsten Kattanek
 * Erstellt am: 06.03.2019
 * Copyright: Thorsten Kattanek
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 *
 */

// ** PIN CONFIG AtMega8 **
//
// Clk (Clock)         PB5
// Data Out            PB3
// Latch               PB0
// OE (Output Enable)  PB1


// serial output over spi
#define LED_PORT    PORTB
#define LED_DDR     DDRB

#define LED_CLK     PB5     //(SPI SCK)
#define LED_DOUT    PB3     //(SPI MOSI)
#define LED_LATCH   PB0
#define LED_OE      PB1
#define SPI_SS      PB2     //(SPI SS)


// CONFIGURATION
#define F_CPU   8000000UL
#define MODULE_COUNT 1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void init_soft_pwm(void);
void set_pwm_value(uint8_t modul_nr, uint8_t led_nr, uint8_t value);
void init_send_byte(void);
void send_byte(uint8_t value);

volatile uint8_t pwm_channel_value[8*MODULE_COUNT];
volatile uint8_t pwm_channel_out[MODULE_COUNT];

// 8-Bit PWM Counter
volatile uint8_t pwm_counter;

int main(void)
{
    init_send_byte();
    send_byte(0);
    init_soft_pwm();
    sei();

    uint32_t counter1 = 0;
    uint8_t counter2 = 0;

    uint8_t led0 = 0;
    uint8_t led_nr = 0;

    uint8_t loop_mode = 0;

    while( 1 )
    {        
        switch(loop_mode)
        {
        case 0:
            if(counter1 == 300)
            {
                counter1 = 0;
                set_pwm_value(0,led_nr,led0);
                if(led0 == 0xff)
                {
                    if(led_nr == 7)
                    {
                        loop_mode = 1;
                    }
                    else
                    {
                        led0 = 0x00;
                        led_nr++;
                    }
                }
                else
                    led0++;
            }
            else
                counter1++;
            break;
        case 1:
            if(counter1 == 300)
            {
                counter1 = 0;
                set_pwm_value(0,led_nr,led0);
                if(led0 == 0x00)
                {
                    if(led_nr == 0)
                    {
                        loop_mode = 2;
                    }
                    else
                    {
                        led0 = 0xff;
                        led_nr--;
                    }
                }
                else
                    led0--;

            }
            else
                counter1++;
            break;
        case 2:
            if(counter1 == 300)
            {
                counter1 = 0;
                set_pwm_value(0,led_nr,led0);
                if(led0 == 0xff)
                {
                        loop_mode = 3;
                }
                else
                    led0++;
            }
            else
                counter1++;
            break;
        case 3:
            if(counter1 == 300)
            {
                counter1 = 0;
                set_pwm_value(0,led_nr,led0);
                if(led0 == 0x00)
                {
                        loop_mode = 2;
                        led_nr++;
                        if(led_nr == 8)
                        {
                            loop_mode = 4;
                            led_nr = 0;
                        }
                }
                else
                    led0--;

            }
            else
                counter1++;
            break;

        case 4:
            set_pwm_value(0,0,8);
            set_pwm_value(0,1,32);
            set_pwm_value(0,2,64);
            set_pwm_value(0,3,96);
            set_pwm_value(0,4,128);
            set_pwm_value(0,5,160);
            set_pwm_value(0,6,192);
            set_pwm_value(0,7,225);
            loop_mode = 5;
            break;
        case 5:
            if(counter1 == 100000)
            {
                counter1 = 0;
                loop_mode = 6;
            }
            else
                counter1++;
            break;
        case 6:
            set_pwm_value(0,7,8);
            set_pwm_value(0,6,32);
            set_pwm_value(0,5,64);
            set_pwm_value(0,4,96);
            set_pwm_value(0,3,128);
            set_pwm_value(0,2,160);
            set_pwm_value(0,1,192);
            set_pwm_value(0,0,225);
            loop_mode = 7;
            break;
        case 7:
            if(counter1 == 100000)
            {
                counter1 = 0;

                if(counter2 == 4)
                {
                    loop_mode = 0;
                    set_pwm_value(0,0,0);
                    set_pwm_value(0,1,0);
                    set_pwm_value(0,2,0);
                    set_pwm_value(0,3,0);
                    set_pwm_value(0,4,0);
                    set_pwm_value(0,5,0);
                    set_pwm_value(0,6,0);
                    set_pwm_value(0,7,0);
                }
                else {
                    counter2++;
                   loop_mode = 4;
                }

            }
            else
                counter1++;
            break;

        default:
            break;
        }
    }
}

void init_soft_pwm(void)
{
    uint8_t i = 0;
    pwm_counter = 0;

    for(i=0; i<(8*MODULE_COUNT); i++)
    {
        pwm_channel_value[i] = 0;
    }

    for(i=0; i<MODULE_COUNT; i++)
    {
        pwm_channel_out[i] = 0;
    }

    // Timer2 --> wird alle 40us aufgerufen (40us*256PWM ^ ca. 100MHz)
    TCCR0B= 1<<CS00;	// Start Timer 0 with prescaler 1
    TIMSK0= (1<<TOIE0);  // Enable Timer 0 overflow interrupt
}

void set_pwm_value(uint8_t modul_nr, uint8_t led_nr, uint8_t value)
{
    pwm_channel_value[(modul_nr << 3) + led_nr] = value;
}

void init_send_byte(void)
{
    //
    LED_DDR |= (1<<LED_LATCH) | (1<<LED_OE) | (1<<LED_CLK) | (1<<LED_DOUT) | (1<<SPI_SS);
    LED_PORT = 0x00;

    // configure spi
    SPCR |= (1<<MSTR);                  // Set as Master

    SPCR &= ~((1<<SPR0)|(1<<SPR1));     // divided clock by 2
    SPSR |= 1<<SPI2X;

    SPCR |= (1<<SPE);                   // enable SPI
}

void send_byte(uint8_t value)
{
    SPDR = value;

    _delay_us(2);

    LED_PORT ^= 1<<LED_LATCH;
    LED_PORT ^= 1<<LED_LATCH;
}

ISR (TIMER0_OVF_vect)
{
    uint8_t modul_counter;

    if(pwm_counter == 0)
        for(modul_counter=0; modul_counter<MODULE_COUNT; modul_counter++)
            pwm_channel_out[modul_counter] = 0xff;

    uint8_t *value = &pwm_channel_value[0];
    for(modul_counter=0; modul_counter<MODULE_COUNT; modul_counter++)
    {
        if(pwm_counter > *value++)
            pwm_channel_out[modul_counter] &= ~(1<<0);

        if(pwm_counter > *value++)
            pwm_channel_out[modul_counter] &= ~(1<<1);

        if(pwm_counter > *value++)
            pwm_channel_out[modul_counter] &= ~(1<<2);

        if(pwm_counter > *value++)
            pwm_channel_out[modul_counter] &= ~(1<<3);

        if(pwm_counter > *value++)
            pwm_channel_out[modul_counter] &= ~(1<<4);

        if(pwm_counter > *value++)
            pwm_channel_out[modul_counter] &= ~(1<<5);

        if(pwm_counter > *value++)
            pwm_channel_out[modul_counter] &= ~(1<<6);

        if(pwm_counter > *value++)
            pwm_channel_out[modul_counter] &= ~(1<<7);

        send_byte(pwm_channel_out[modul_counter]);
    }
    pwm_counter++;
}
