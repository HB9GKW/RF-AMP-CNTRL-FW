/*
 * Amplifier config
 */

// Signal mapping for port D
#define RF_SW1_ON       PD0
#define RF_SW2_ON       PD1
#define ILK             PD2
#define PTT             PD3
#define GATE_ALC_EN     PD4
#define VDD_EN          PD5
#define RF_SW1_EN       PD6
#define RF_SW2_EN       PD7

// Signal mapping for port B
#define OPR             PB0
#define RESET_ILK       PB1
#define DISPLAY         PB2
#define MOSI_FAN        PB3
#define MISO_RF_INHIBIT PB4
#define SCK_SOFT_ILK    PB5
#define ON_AIR          PB6
#define FAULT           PB7

// Signal mapping for port C
#define TEMP1_MON       PC0
#define TEMP2_MON       PC1
#define VDD_MON         PC2
#define IDD_MON         PC3
#define FWD_MON         ADC6
#define REF_MON         ADC7

// Signal mapping for ILK port
#define ILK_HSWR1       0b11111110
#define ILK_HSWR2       0b11111101
#define ILK_HSWR3       0b11111011
#define ILK_HSWR4       0b11110111
#define ILK_RF_OL       0b11101111
#define ILK_IDD_OL      0b11011111
#define ILK_TEMP1       0b10111111
#define ILK_TEMP2       0b01111111

// Signal settings
// Offset
#define OFF_TEMP_LO     77
#define OFF_TEMP_HI     15
#define OFF_VDD         0
#define OFF_IDD         0
// Gain
#define G_TEMP_LO       83
#define G_TEMP_HI       123
#define G_VDD           1017
#define G_IDD           98
// Fan threshold
#define TEMP_HI         420
#define TEMP_LO         300
// Bargraph grid
#define BAR_F           4014    // bar_f = 2^16 / 16 x g_idd/100
#define BAR_1           803     // bar_1 = bar_f * 1 / 5
#define BAR_2           1606    // bar_2 = bar_f * 2 / 5
#define BAR_3           2408    // bar_2 = bar_f * 3 / 5
#define BAR_4           3211    // bar_2 = bar_f * 4 / 5

// Define Display Strings and Characters in PROGMEM
const char string_flash1[] PROGMEM = "RF-AMP-CNTRL_V1";
const char string_flash2[] PROGMEM = "*** HB9GKW ***";
const char string_flash3[] PROGMEM = "OL";
const char string_flash4[] PROGMEM = "IDD";
const char string_flash5[] PROGMEM = "VDD";
const char string_flash6[] PROGMEM = "TEMP1";
const char string_flash7[] PROGMEM = "TEMP2";
const char string_flash8[] PROGMEM = "HSWR";
const char string_flash9[] PROGMEM = "AMP1";
const char string_flash10[] PROGMEM = "AMP2";
const char string_flash11[] PROGMEM = "SYM";
const char string_flash12[] PROGMEM = "ANT";
const char string_flash13[] PROGMEM = "FAULT";
const char degC[] PROGMEM = {0x18, 0x18, 0x07, 0x08, 0x08, 0x08, 0x08, 0x07};
const char bar1[] PROGMEM = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};
const char bar2[] PROGMEM = {0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18};
const char bar3[] PROGMEM = {0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C};
const char bar4[] PROGMEM = {0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E};
