/*
 * HB9GKW RF-AMP-CONTROLLER
 */

#define F_CPU 8000000UL

#include <stdbool.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "config.h"
#include "i2cmaster.h"
#include "mcp23017.h"
#include "i2clcd.h"

// Function prototypes
void gpio_setup(void);
void ADC_init(void);
void display_init(void);
uint16_t ADC_read(uint8_t channel);
void sequence_on(void);
void sequence_off(void);
void clean(unsigned char *var);
void read_display_button(uint8_t *, uint8_t *);
void print_temp(void);
void print_vdd(void);
void print_idd(void);

int main(void) {
	// GPIO Setup
	gpio_setup();
	// I2C init
	i2c_init();
	// LCD Display init
	display_init();
	// MCP23017 init
	mcp23017_init();
	// ADC init
	ADC_init();
	// Display mode init
	uint8_t dm = 0, disp = 0;
	// Enable global interrupts if no ILK is pending
	if (PIND & (1 << ILK)) sei();
	else PORTB |= (1 << FAULT);

	// Main loop
	for (;;) {
	lcd_command(LCD_CLEAR);
	_delay_ms(10);
	// Set display units [°C]...[V]...[°C]
	lcd_putcharlc(1, 4, 0); lcd_putcharlc(1, 10, 86); lcd_putcharlc(1, 16, 0);

	// Standbye mode
	while ( (PINB & (1 << OPR)) && !(PINB & (1 << FAULT)) ) {
		print_temp();
		print_vdd();
		read_display_button(&dm, &disp);
		_delay_ms(100);
	}

	// OPR mode
	// Switch on VDD
	if ( !(PINB & (1 << FAULT)) && !(PINB & (1 << OPR)) ) PORTD |= (1 << VDD_EN);
	while ( !(PINB & (1 << OPR)) && !(PINB & (1 << FAULT)) ) {
		// display Temp / VDD / IDD
		print_temp();
		print_vdd();
		print_idd();
	}
	// Leave OPR mode
	sequence_off();
	PORTD &= ~(1 << VDD_EN);

	// Fault mode
	if (PINB & (1 << FAULT)) {
		lcd_command(LCD_CLEAR);
		_delay_ms(10);
		lcd_putcharlc(1, 4, 0); lcd_putcharlc(1, 10, 86); lcd_putcharlc(1, 16, 0);
		// Read ILK register and set FAULT register
		uint8_t err = mcp23017_readbyte(MCP23017_INTCAPA);
		mcp23017_writebyte(MCP23017_OLATB, ~err);
		lcd_printlc_P(2, 1, string_flash13);
		switch (err) {
			case ILK_HSWR1:
			lcd_printlc_P(2, 7, string_flash8);
			lcd_printlc_P(2, 12, string_flash9);
			break;
			case ILK_HSWR2:
			lcd_printlc_P(2, 7, string_flash8);
			lcd_printlc_P(2, 12, string_flash10);
			break;
			case ILK_HSWR3:
			lcd_printlc_P(2, 7, string_flash8);
			lcd_printlc_P(2, 12, string_flash11);
			break;
			case ILK_HSWR4:
			lcd_printlc_P(2, 7, string_flash8);
			lcd_printlc_P(2, 12, string_flash12);
			break;
			case ILK_RF_OL:
			lcd_printlc_P(2, 7, string_flash3);
			break;
			case ILK_IDD_OL:
			lcd_printlc_P(2, 7, string_flash4);
			break;
			case ILK_TEMP1:
			lcd_printlc_P(2, 7, string_flash6);
			break;
			case ILK_TEMP2:
			lcd_printlc_P(2, 7, string_flash7);
			break;
		}
	}
	while (PINB & (1 << FAULT)) {
		// Clear FAULT if Reset is pressed and no ILK is pending
		if ( (PIND & (1 << ILK)) && !(PINB & (1 << RESET_ILK)) ) {
			PORTB &= ~(1 << FAULT);
			sei();
		}
		print_temp();
		print_vdd();
		(void) mcp23017_readbyte(MCP23017_INTCAPA);
		_delay_ms(10);
	}
	}
	return 0;
}
// Interrupt Service Routine at falling edge of ILK
ISR (INT0_vect) {
	PORTD &= ~( (1 << GATE_ALC_EN) | (1 << VDD_EN) | (1 << RF_SW2_EN) );
	PORTB |= (1 << FAULT);
	cli();
}
// Interrupt Service Routine at any change of PTT
ISR (INT1_vect) {
	if ( !(PINB & (1 << FAULT)) && !(PINB & (1 << OPR)) && !(PIND & (1 << PTT)) ) sequence_on();
	else sequence_off();
}

void gpio_setup(void) {
	// Data Direction Register Port D: GATE_ALC_EN / VDD_EN / RF_SW1_EN / RF_SW2_EN -> output
	DDRD = (1 << GATE_ALC_EN) | (1 << VDD_EN) | (1 << RF_SW1_EN) | (1 << RF_SW2_EN);
	// Data Direction Register Port B: ON_AIR / FAULT / MOSI_FAN -> output
	DDRB = (1 << ON_AIR) | (1 << FAULT) | (1 << MOSI_FAN);
	// Activate pull-up resistors Port D: ILK / PTT -> active low
	PORTD |= ( (1 << ILK) | (1 << PTT) );
	// Activate pull-up resistors Port B: OPR / RESET_ILK / DISPLAY -> active low
	PORTB |= ( (1 << OPR) | (1 << RESET_ILK) | (1 << DISPLAY) );
	// MCU Control Register: INT for falling edge on INT0 (ILK) and any change on INT1 (PTT)
	MCUCR |= (1 << ISC01) | (1 << ISC10);
	// General Interrupt Control Register; enable INT0 + INT1
	GICR |= (1 << INT0) | (1 << INT1);
}

void ADC_init(void) {
	// uncomment for AVcc reference
	// ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADPS2) | (0 << ADPS1) | (1 << ADPS0);
	ADCSRA |= (1 << ADEN);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC) ) {}
	(void) ADCW;
}

void display_init(void) {
	lcd_init();
	lcd_def_char(degC, 0);
	lcd_def_char(bar1, 1);
	lcd_def_char(bar2, 2);
	lcd_def_char(bar3, 3);
	lcd_def_char(bar4, 4);
	lcd_command(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGOFF);
	lcd_light(true);
	lcd_printlc_P(1, 1, string_flash1);
	lcd_printlc_P(2, 1, string_flash2);
	_delay_ms(1500);
}

uint16_t ADC_read(uint8_t channel) {
	ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC) ) {}
	return ADCW;
}

void sequence_on(void) {
	// Switch ON RF_SW1
	PORTD |= (1 << RF_SW1_EN);
	// wait until RF_SW1_ON
	while (!(PIND & (1 << RF_SW1_ON)) ) {}
	// Switch ON RF_SW2 if RF_SW1 is ON
	PORTD |= (1 << RF_SW2_EN);
	while (!(PIND & (1 << RF_SW2_ON)) ) {}
	// Switch ON GATE & ALC if RF_SW2 is ON
	PORTD |= (1 << GATE_ALC_EN);
	PORTB |= (1 << ON_AIR);
}

void sequence_off(void) {
	// Switch OFF GATE & ALC
	PORTD &= ~(1 << GATE_ALC_EN);
	// wait 500us
	_delay_us(500);
	// Switch OFF RF_SW2
	PORTD &= ~(1 << RF_SW2_EN);
	// Switch OFF RF_SW1 if RF_SW2 is OFF
	while ( PIND & (1 << RF_SW2_ON) ) {}
	PORTD &= ~(1 << RF_SW1_EN);
	PORTB &= ~(1 << ON_AIR);
}
// Clean buffer
void clean(unsigned char *var) {
	int i = 0;
	while(var[i] != '\0') {
		var[i] = '\0';
		i++;
	}
}
// Poll diplay select button
void read_display_button(uint8_t *dm, uint8_t *disp) {
	if ( !(PINB & (1 << DISPLAY)) && (*dm == 0) ) {
		*dm = 1; *disp =+ 1;
		if (*disp > 2) *disp = 0;
		}
	else if ( (PINB & (1 << PB2)) && (*dm == 1) ) *dm = 0;
}
// Read temperature and print to LCD
void print_temp(void) {
        int16_t adcval;
        unsigned char buffer[4] = {'\0'};               		// string buffer for LCD
        for (uint8_t i = 0; i < 2; i++) {
        	char cache_i[2];                                        // cache for integer calculation
                adcval = ADC_read(i);
		if (i == 1) adcval -= 10;				// tweak sensor 1 and 2
                if ( adcval > temp_hi && !(PINB & (1 << MOSI_FAN)) ) {
			PORTB |= (1 << MOSI_FAN);  			// switch on FAN
		}
                if ( adcval < temp_lo && (PINB & (1 << MOSI_FAN)) ) {
			PORTB &= ~(1 << MOSI_FAN);  			// switch off FAN
		}
                if ( adcval <= 267 ) {
			adcval = adcval - off_temp_l;        		// subtract offset
			if (adcval >= 0) buffer[0] = 43;        	// adds '+' as 1st char
			else buffer[0] = 45;				// adds '-' as 1st char
			adcval = abs(10 * adcval);			// change to abs value and multiply by 10
			if ( ((10 * (adcval % g_temp_l)) / g_temp_l) >= 5 ) {
			adcval = (adcval / g_temp_l) + 1;		// round up
			}
			else adcval = adcval / g_temp_l;
                }
                else {
			adcval = adcval + off_temp_h;        		// add offset
			buffer[0] = 43;        				// adds '+' as 1st char
			adcval = 10 * adcval;                        	// multiply by 10
			if ( ((10 * (adcval % g_temp_h)) / g_temp_h) >= 5 ) {
				adcval = (adcval / g_temp_h) + 1; 	// round up
			}
			else adcval = adcval / g_temp_h;
		}
                itoa(adcval, cache_i, 10);                      	// converte integer part to string
                if (adcval < 10) {
                        buffer[1] = 32;                                 // insert space as 2nd char
                        buffer[2] = cache_i[0];                 	// put cache_i as 3rd char
                }
                else {							// value > 10
                        buffer[1] = cache_i[0];                 	// put cache_i[0] as 2nd char
                        buffer[2] = cache_i[1];                 	// put cache_i[1] as 3rd char
                }
                buffer[3] = '\0';
                if (i == 0) lcd_printlc(1, 1, buffer);  		// print buffer left
                else lcd_printlc(1, 13, buffer);                	// print buffer right
                clean(buffer);
        }
}
// Read voltage and print to LCD
void print_vdd(void) {
	uint16_t adcval;
	unsigned char buffer[5] = {'\0'};
	char cache_i[2], cache_f[1];
	adcval = ( (ADC_read(2) - off_vdd) << 6);
	itoa((adcval / g_vdd), cache_i, 10);
	if (adcval / g_vdd < 10) {
		buffer[0] = 32;
		buffer[1] = cache_i[0];
	}
	else {
		buffer[0] = cache_i[0];
		buffer[1] = cache_i[1];
	}
	if ( ((10 * (adcval % g_vdd)) / g_vdd) >= 5 ) {
	itoa( (10 * (adcval % g_vdd) / g_vdd) + 1, cache_f, 10);
	}
	else itoa( (10 * (adcval % g_vdd) / g_vdd), cache_f, 10);
	buffer[2] = 46;
	buffer[3] = cache_f[0];
	buffer[4] = '\0';
	lcd_printlc(1, 6, buffer);
}
// Read current and print bargraph to LCD
void print_idd(void) {
	uint16_t adcval;
	unsigned char buffer[17] = {'\0'};
	uint8_t zero_flag = 0;
	adcval = ( (ADC_read(3) - off_idd) << 6);
	for (uint8_t i = 0; i <= 15; i++) {
		if (zero_flag == 1) buffer[i] = 254;
		else if ( adcval >= bar_f * (1 + i) ) buffer[i] = 255;
		else if ( adcval >= bar_4 + (bar_f * i) ) buffer[i] = 4;
		else if ( adcval >= bar_3 + (bar_f * i) ) buffer[i] = 3;
		else if ( adcval >= bar_2 + (bar_f * i) ) buffer[i] = 2;
		else if ( adcval >= bar_1 + (bar_f * i) ) buffer[i] = 1;
		else {
			buffer[i] = 254;
			zero_flag = 1;
		}
	}
	buffer[16] = '\0';
	lcd_printlc(2, 1, buffer);
}
