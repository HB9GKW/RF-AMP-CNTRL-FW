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
void read_temp(void);

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
	// Enable global interrupts
	sei();
	// Main loop
	for (;;) {
	// OPR mode
	lcd_command(LCD_CLEAR);
	while ( !(PINB & (1 << OPR)) && !(PINB & (1 << FAULT)) ) {
		// Switch on VDD
		if ( !(PORTD & (1 << VDD_EN)) ) PORTD |= (1 << VDD_EN);
		// read ADCs and update Display
		if ( (PINB & (1 << ON_AIR)) ) {
			// read & display temp 1 + 2, IDD, VDD
		}
	}
	sequence_off();
	PORTD &= ~(1 << VDD_EN);
	lcd_printlc_P(1, 1, string_flash6); lcd_printlc_P(2, 1, string_flash7);
	unsigned char test = 0x00;
	lcd_putcharlc(2, 9, test);
	// Standbye mode
	while ( (PINB & (1 << OPR)) && !(PINB & (1 << FAULT)) ) {
		read_temp();
		read_display_button(&dm, &disp);
		_delay_ms(100);
	}
	// Fault mode
	while (PINB & (1 << FAULT)) {
		// Read ILK register and set FAULT register
		uint8_t err = mcp23017_readbyte(MCP23017_INTCAPA);
		mcp23017_writebyte(MCP23017_OLATB, ~err);
		switch (err) {
			case ILK_HSWR1:
			case ILK_HSWR2:
			case ILK_HSWR3:
			case ILK_HSWR4:
			lcd_printlc_P(2, 5, string_flash8);
			break;
			case ILK_RF_OL:
			lcd_printlc_P(2, 5, string_flash3);
			break;
			case ILK_IDD_OL:
			lcd_printlc_P(2, 5, string_flash4);
			break;
			case ILK_TEMP1:
			lcd_printlc_P(2, 5, string_flash6);
			break;
			case ILK_TEMP2:
			lcd_printlc_P(2, 5, string_flash7);
			break;
		}
		// Clear FAULT if Reset is pressed and no ILK is pending
		if ( (PIND & (1 << ILK)) && !(PINB & (1 << RESET_ILK)) ) {
		PORTB &= ~(1 << FAULT);
		sei();
		}
		_delay_ms(100);
		(void) mcp23017_readbyte(MCP23017_INTCAPA);
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
	ADMUX = (1 << REFS0);
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
	_delay_ms(1000);
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
// clean buffer
void clean(unsigned char *var) {
	int i = 0;
	while(var[i] != '\0') {
		var[i] = '\0';
		i++;
	}
}
// poll diplay select button
void read_display_button(uint8_t *dm, uint8_t *disp) {
	if ( !(PINB & (1 << PB2)) && (*dm == 0) ) {
		*dm = 1; *disp++;
		if (*disp > 2) *disp = 0;
		}
	else if ( (PINB & (1 << PB2)) && (*dm == 1) ) *dm = 0;
}
// Read ADC and print to Display
void read_temp(void) {
	int16_t adcval;
	// Buffer for Display
	unsigned char buffer[6] = {'\0'};
	// cache for integer calculation
	char cache_i[3];
	// cache for floating number calculation
	char cache_f[1];
	//clean(buffer);
	adcval = ADC_read(1)-off1;		// subtract offset
	if (adcval>0) buffer[0] = 43; 		// adds '+' as 1st char
	else buffer[0]=45; 			// adds '-' as 1st char
	adcval=abs(adcval);			// change to abs value (modulo)
	itoa((adcval/g1), cache_i, 10); 	// converte integer part to string
	if ((adcval/g1) < 10) {
		buffer[1] = 32; 		// insert space as 2nd char
		buffer[2] = cache_i[0];		// put cache_i as 3rd char
	}
	else {					// value > 10
		buffer[1] = cache_i[0];		// put cache_i[0] as 2nd char
		buffer[2] = cache_i[1];		// put cache_i[1] as 3rd char
	}
	itoa((10*(adcval%g1)/g1), cache_f, 10); // extract decimal place
	buffer[3] = 46;				// put '.' as 4th char
	buffer[4] = cache_f[0];			// put cache_f[0] as 5th char
	buffer[5] = '\0';
	lcd_printlc(2, 4, buffer);		// print buffer string to lcd
}
