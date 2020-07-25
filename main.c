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
#include "i2cmaster.h"
#include "i2clcd.h"

// Signal mapping for port D
#define RF_SW1_ON PD0
#define RF_SW2_ON PD1
#define ILK PD2
#define PTT PD3
#define GATE_ALC_EN PD4
#define VDD_EN PD5
#define RF_SW1_EN PD6
#define RF_SW2_EN PD7

// Signal mapping for port B
#define OPR PB0
#define RESET_ILK PB1
#define DISPLAY PB2
#define MOSI_FAN PB3
#define MISO_RF_INHIBIT PB4
#define SCK_SOFT_ILK PB5
#define ON_AIR PB6
#define FAULT PB7

// Signal mapping for port C
#define TEMP1_MON PC0
#define TEMP2_MON PC1
#define VDD_MON PC2
#define IDD_MON PC3
#define FWD_MON ADC6
#define REF_MON ADC7

volatile uint8_t tx_req_flag = 0;
volatile uint8_t rx_req_flag = 0;
volatile uint8_t read_fault_req_flag = 0;

// Define Display Strings and Characters in PROGMEM
const char string_flash1[] PROGMEM = "RF-AMP-CNTRL V1";
const char string_flash2[] PROGMEM = "*** HB9GKW ***";
const char string_flash3[] PROGMEM = "OL";
const char string_flash4[] PROGMEM = "IDD";
const char string_flash5[] PROGMEM = "VDD";
const char string_flash6[] PROGMEM = "T1";
const char string_flash7[] PROGMEM = "T2";
const char string_flash8[] PROGMEM = "HSWR";
const char string_flash9[] PROGMEM = "FWD";
const char string_flash10[] PROGMEM = "REF";
const char string_flash11[] PROGMEM = "SWR";
const char bar1[] PROGMEM = {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F};

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
	while (ADCSRA & (1 << ADSC) ) {
	}
	(void) ADCW;
}

void display_init(void) {
	lcd_init();
	lcd_def_char(bar1, 3);
	lcd_command(LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKINGOFF);
	lcd_light(true);
	lcd_printlc_P(1, 1, string_flash1);
	lcd_printlc_P(2, 1, string_flash2);
	_delay_ms(1000);
	lcd_command(LCD_CLEAR);
}

uint16_t ADC_read( uint8_t channel ) {
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

void clean(char *var) {
	int i = 0;
	while(var[i] != '\0') {
		var[i] = '\0';
		i++;
	}
}

// Buffer for Display
unsigned char buffer[6] = {'\0'};
// cache for integer calculation
char cache_i[3];
// cache for floating number calculation
char cache_f[1];
// ADC value
int16_t adcval;
// Temp offset
const uint8_t off1 = 100;
// Temp gain
const uint8_t g1 = 10;
// Read ADC and print to Display
void read_temp(void) {
	//clean(buffer);
	adcval = ADC_read(1)-off1;		// subtract offset
	if (adcval>0) buffer[0]=43; 		// adds '+' as 1st char
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


int check_state(int dm, int disp) {
	if ( (PINB & (1 << PB2)) && (dm == 0) ) {
		dm = 1; disp++;
		if (disp > 2) disp = 0;
		}
	else dm = 0;
	return disp;
}

int main(void) {
	// GPIO Setup
	gpio_setup();
	// Enable global interrupts
	sei();
	// I2C init
	i2c_init();
	// LCD Display init
	display_init();
	// ADC init
	ADC_init();
	uint8_t dm = 0, disp = 0;
	// Main loop
	for (;;) {
	// OPR mode
	while ( !(PINB & (1 << OPR)) ) {
		// Switch on VDD if Fault is cleared
		if ( !(PORTD & (1 << VDD_EN)) && !(PINB & (1 << FAULT)) ) {
			PORTD |= (1 << VDD_EN);
		}
		if (rx_req_flag == 1) {
			sequence_off();
			rx_req_flag = 0;
		}
		if (tx_req_flag == 1) {
			sequence_on();
			tx_req_flag = 0;
		}
		// Clear FAULT if Reset is pressed and no ILK is pending
		if ( (PINB & (1 << FAULT)) && (PIND & (1 << ILK)) && !(PINB & (1 << RESET_ILK)) ) {
			PORTB &= ~(1 << FAULT);
		}
	}
	sequence_off();
	PORTD &= ~(1 << VDD_EN);
	lcd_printlc_P(1, 1, string_flash6); lcd_printlc_P(2, 1, string_flash7);
	// Standbye mode
	while (PINB & (1 << OPR)) {
		// Clear FAULT if Reset is pressed and no ILK is pending
		if ( (PINB & (1 << FAULT)) && (PIND & (1 << ILK)) && !(PINB & (1 << RESET_ILK)) ) {
			PORTB &= ~(1 << FAULT);
		}
	read_temp();
	check_state(dm, disp);
	_delay_ms(100);
	}
	}
	return 0;
}

ISR (INT0_vect) {
	// Interrupt Service Routine at falling edge of ILK
	PORTD &= ~( (1 << GATE_ALC_EN) | (1 << VDD_EN) | (1 << RF_SW2_EN) );
	PORTB |= (1 << FAULT);
	// Read ILK register and set FAULT register
	read_fault_req_flag = 1;
	rx_req_flag = 1;
	tx_req_flag = 0;
}

ISR (INT1_vect) {
	// Interrupt Service Routine at any change of PTT
	if ( !(PINB & (1 << FAULT)) && !(PINB & (1 << OPR)) && !(PIND & (1 << PTT)) ) {
		tx_req_flag = 1;
		rx_req_flag = 0;
	}
	else {
		rx_req_flag = 1;
		tx_req_flag = 0;
	}
}
