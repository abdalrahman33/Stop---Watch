#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

unsigned char count_flag = 0x00;
unsigned char seconds = 0;
unsigned char minutes = 0;
unsigned char hours = 0;

unsigned char segments[6] = {0x20, 0x10, 0x08, 0x04, 0x02, 0x01};

void Timer1_init_CTC()
{
    TCCR1A = (1<<FOC1A); /* non PWM mode */
    TCCR1B = (1<<WGM12) | (1<<CS10) | (1<<CS12); /* N = 1024, CTC */
    TCNT1 = 0;
    OCR1A = 15625; /* NUM OF COUNTS TO REACH A SECOND */
    TIMSK |= (1<<OCIE1A); /* OUTPUT COMPARE INTERRUPT ENABLE */
}

void INT0_init()
{
    DDRD &= ~(1<<PD2); /* Configure INT0/PD2 as input pin */
    MCUCR |= (1<<ISC01); /* The falling edge of INT0 */
    GICR |= (1<<INT0);
}

void INT1_init()
{
    DDRD &= ~(1<<PD3);
    MCUCR = (1<<ISC11) | (1<<ISC10);
    GICR |= (1<<INT1);
}

void INT2_init()
{
    DDRB &= ~(1<<PB2); /* Configure INT2/PB2 as input pin */
    MCUCSR |= (1<<ISC2); /* The falling edge of INT2 */
    GICR |= (1<<INT2);
}

void Display_Segments()
{
    PORTA &= ~(0x3F);
    PORTA |= segments[0];
    PORTC = (PORTC & 0xF0) | (seconds % 10);
    _delay_ms(2); // delay to display

    PORTA &= ~(0x3F);
    PORTA |= segments[1];
    PORTC = (PORTC & 0xF0) | (seconds / 10);
    _delay_ms(2);

    PORTA &= ~(0x3F);
    PORTA |= segments[2];
    PORTC = (PORTC & 0xF0) | (minutes % 10);
    _delay_ms(2);

    PORTA &= ~(0x3F);
    PORTA |= segments[3];
    PORTC = (PORTC & 0xF0) | (minutes / 10);
    _delay_ms(2);

    PORTA &= ~(0x3F);
    PORTA |= segments[4];
    PORTC = (PORTC & 0xF0) | (hours % 10);
    _delay_ms(2);

    PORTA &= ~(0x3F);
    PORTA |= segments[5];
    PORTC = (PORTC & 0xF0) | (hours / 10);
    _delay_ms(2);
}


void toggle_counter()
{
	static unsigned char button_pressed = 0x00;

	if((PINB & (1<<PB7)) && !(button_pressed) )
	{
		Display_Segments();// to make the 6 7segments not flash
		_delay_ms(30);

		if(PINB & (1<<PB7))
		{
			count_flag ^= 0x01;
			button_pressed = 0x01;
		}

		if(count_flag & 0x01)
		{
			PORTD &= ~(1<<PD4);
			PORTD |= (1<<PD5);

		}else
		{
			PORTD |= (1<<PD4);
			PORTD &= ~(1<<PD5);

		}
	}
	else if(!(PINB & (1<<PB7)))
	{
		Display_Segments();// to make the 6 7segments not flash
		button_pressed = 0x00;
	}

}

void sec_increments()
{
	static unsigned char button_pressed = 0x00;

	if( (PINB & (1<<PB6)) )
	{
		_delay_ms(30);

		if( (PINB & (1<<PB6)) && !(button_pressed))
		{

			if(seconds == 59)

			{
				seconds = 0;

			}
			else
			{

				seconds++;
			}
		}

		button_pressed = 0x01;

	}
	else
	{
		button_pressed = 0x00;
	}

	Display_Segments();// to make the 6 7segments not flash



}

void sec_decrements()
{
	static unsigned char button_pressed = 0x00;

	if( (PINB & (1<<PB5)) )
	{
		_delay_ms(30);

		if( (PINB & (1<<PB5)) && !(button_pressed))
		{

			if(seconds == 0)

			{
				seconds = 59;

			}
			else
			{

				seconds--;
			}
		}

		button_pressed = 0x01;

	}
	else
	{
		button_pressed = 0x00;
	}

	Display_Segments();// to make the 6 7segments not flash


}

void minutes_decrements()
{
	static unsigned char button_pressed = 0x00;

	if( (PINB & (1<<PB3)) )
	{
		_delay_ms(30);

		if( (PINB & (1<<PB3)) && !(button_pressed))
		{

			if(minutes == 0)

			{
				minutes = 59;

			}
			else
			{

				minutes--;
			}
		}

		button_pressed = 0x01;

	}
	else
	{
		button_pressed = 0x00;
	}

	Display_Segments();// to make the 6 7segments not flash


}

void minutes_increments()
{
	static unsigned char button_pressed = 0x00;

	if( (PINB & (1<<PB4)) )
	{
		_delay_ms(30);

		if( (PINB & (1<<PB4)) && !(button_pressed))
		{

			if(minutes == 59)

			{
				minutes = 0;

			}
			else
			{

				minutes++;
			}
		}

		button_pressed = 0x01;

	}
	else
	{
		button_pressed = 0x00;
	}

	Display_Segments();// to make the 6 7segments not flash


}

void hours_decrements()
{
	static unsigned char button_pressed = 0x00;

	if( (PINB & (1<<PB0)) )
	{
		_delay_ms(30);

		if( (PINB & (1<<PB0)) && !(button_pressed))
		{

			if(hours == 0)

			{
				hours = 24;

			}
			else
			{

				hours--;
			}
		}

		button_pressed = 0x01;

	}
	else
	{
		button_pressed = 0x00;
	}

	Display_Segments();// to make the 6 7segments not flash


}

void hours_increments()
{
	static unsigned char button_pressed = 0x00;

	if( (PINB & (1<<PB1)) )
	{
		_delay_ms(30);

		if( (PINB & (1<<PB1)) && !(button_pressed))
		{

			if(hours == 24)

			{
				hours = 0;

			}
			else
			{

				hours++;
			}
		}

		button_pressed = 0x01;

	}
	else
	{
		button_pressed = 0x00;
	}

	Display_Segments();// to make the 6 7segments not flash


}

ISR(TIMER1_COMPA_vect) // update timer
{
    if (!(count_flag & 0x01))
    {
        // COUNT UP MODE
        seconds++;
        if (seconds == 60)
        {
            seconds = 0;
            minutes++;
        }

        if (minutes == 60)
        {
            minutes = 0;
            hours++;
        }

        if (hours == 24)
        {
            hours = 0;
        }
        PORTD &= ~(1 << PD0);
    }
    else
    {

        // COUNT DOWN MODE
        if (seconds == 0)
        {
            if (minutes == 0)
            {
                if (hours == 0)
                {
                    // buzzer code
                	PORTD |= (1<<PD0);
                }
                else
                {
                    hours--;
                    minutes = 59;
                    seconds = 59;
                }
            }
            else
            {
                minutes--;
                seconds = 59;
            }
        }
        else
        {
            seconds--;
        }
    }
}

ISR(INT0_vect)
{
    /* RESET THE TIMER */
    seconds = 0;
    minutes = 0;
    hours = 0;
}

ISR(INT1_vect)
{
    /* PAUSE THE TIMER */
    TCCR1A = 0;
    TCCR1B = 0;
}

ISR(INT2_vect)
{
    /* RESUME THE TIMER */
    Timer1_init_CTC(); // Reinitialize Timer1 to start counting again
}

int main()
{
    SREG |= (1<<7); // GLOBAL INTERRUPT ENABLE (I-BIT)
    DDRA |= (0x3F);
    PORTA |= (0x3F);
    DDRC |= (0x0F);
    PORTC &= ~(0x0F);
    DDRD |= (0x31); // SET PD0, PD4, PD5 (OUTPUT PINS)
    DDRB &= ~(0xFF); // INPUT PINS
    PORTD &= ~(0x30); // TURNING COUNT LEDS OFF (POSITIVE LOGIC)
    PORTD |= (1<<PD4); // count up LED on (default)

    Timer1_init_CTC();
    INT0_init();
    INT1_init();
    INT2_init();


    while (1)
    {
    	sec_increments();
    	sec_decrements();
    	minutes_increments();
    	minutes_decrements();
    	hours_increments();
    	hours_decrements();
    	toggle_counter();
        Display_Segments(); // Update display with new values
    }
}
