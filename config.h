/*
 * Amplifier config
 */

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

// Signal settings
// Temp1 offset
#define off1 100
// Temp1 gain
#define g1 10

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
// const char bar1[] PROGMEM = {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F};

