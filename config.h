/*
 * Amplifier config
 */

// Signal mapping for port D
#define RF_SW1_ON 		PD0
#define RF_SW2_ON 		PD1
#define ILK 			PD2
#define PTT 			PD3
#define GATE_ALC_EN 	PD4
#define VDD_EN 			PD5
#define RF_SW1_EN 		PD6
#define RF_SW2_EN 		PD7

// Signal mapping for port B
#define OPR 			PB0
#define RESET_ILK 		PB1
#define DISPLAY 		PB2
#define MOSI_FAN 		PB3
#define MISO_RF_INHIBIT PB4
#define SCK_SOFT_ILK 	PB5
#define ON_AIR 			PB6
#define FAULT 			PB7

// Signal mapping for port C
#define TEMP1_MON 		PC0
#define TEMP2_MON 		PC1
#define VDD_MON 		PC2
#define IDD_MON 		PC3
#define FWD_MON 		ADC6
#define REF_MON 		ADC7

// Signal mapping for ILK port
#define ILK_HSWR1		0b11111110
#define ILK_HSWR2		0b11111101
#define ILK_HSWR3		0b11111011
#define ILK_HSWR4		0b11110111
#define ILK_RF_OL		0b11101111
#define ILK_IDD_OL		0b11011111
#define ILK_TEMP1		0b10111111
#define ILK_TEMP2		0b01111111

// Signal settings
// Offset
#define off_temp		100
// Gain
#define g_temp 			10
#define g_vdd			17
// Bargraph grid
#define bar_f			4096	// bar_f = 2^16 / 16
#define bar_1			819		// bar_1 = bar_f * 1 / 5
#define bar_2			1638	// bar_2 = bar_f * 2 / 5
#define bar_3			2458	// bar_2 = bar_f * 3 / 5
#define bar_4			3277	// bar_2 = bar_f * 4 / 5

// Define Display Strings and Characters in PROGMEM
const char string_flash1[] PROGMEM = "RF-AMP-CNTRL_V1";
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
const char string_flash12[] PROGMEM = "FAULT";
const char degC[] PROGMEM = {0x18, 0x18, 0x07, 0x08, 0x08, 0x08, 0x08, 0x07};
const char bar1[] PROGMEM = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};
const char bar2[] PROGMEM = {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18};
const char bar3[] PROGMEM = {0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C};
const char bar4[] PROGMEM = {0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E};
