/*
 * MCP23017
 */

#include <avr/io.h>
#include "i2cmaster.h"
#include "mcp23017.h"

/*
 * write a byte
 */
void mcp23017_writebyte(uint8_t reg, uint8_t data) {
	i2c_start_wait(I2C_WRITE + MCP23017_ADDR);
	i2c_write(reg);
	i2c_write(data);
	i2c_stop();
}

/*
 * write a word
 */
void mcp23017_writeword(uint8_t reg, uint16_t data) {
	i2c_start_wait(I2C_WRITE + MCP23017_ADDR);
	i2c_write(reg);
	i2c_write((uint8_t)data);
	i2c_write((uint8_t)(data>>8));
	i2c_stop();
}

/*
 * read a byte
 */
uint8_t mcp23017_readbyte(uint8_t reg) {
	i2c_start_wait(I2C_WRITE + MCP23017_ADDR);
	i2c_write(reg);
	i2c_rep_start(I2C_READ + MCP23017_ADDR);
	uint8_t data = i2c_readNak();
	i2c_stop();
	return data;
}

/*
 * write single pin on port A
 */
void mcp23017_writepinA(uint8_t pin, uint8_t state) {
	uint8_t data = mcp23017_readbyte(MCP23017_OLATA);
	data &= ~(1<<pin);
	if(state) data |= (1<<pin);
	mcp23017_writebyte(MCP23017_GPIOA, data);
}

/*
 * write single pin on port B
 */
void mcp23017_writepinB(uint8_t pin, uint8_t state) {
	uint8_t data = mcp23017_readbyte(MCP23017_OLATB);
	data &= ~(1<<pin);
	if(state) data |= (1<<pin);
	mcp23017_writebyte(MCP23017_GPIOB, data);
}

/*
 * read pin status on port A
 */
uint8_t mcp23017_readpinA(uint8_t pin) {
	uint8_t data = mcp23017_readbyte(MCP23017_GPIOA);
	return ((data>>pin) & 0b00000001);
}

/*
 * read pin status on port B
 */
uint8_t mcp23017_readpinB(uint8_t pin) {
	uint8_t data = mcp23017_readbyte(MCP23017_GPIOB);
	return ((data>>pin) & 0b00000001);
}

/*
 * init
 */
void mcp23017_init() {
	mcp23017_writebyte(MCP23017_IOCONA, 0x34);	
	mcp23017_writebyte(MCP23017_GPPUA, 0xFF);	
	mcp23017_writebyte(MCP23017_IODIRA, 0xFF);
	mcp23017_writebyte(MCP23017_IODIRB, 0x00);	
	mcp23017_writebyte(MCP23017_DEFVALA, 0xFF);
	mcp23017_writebyte(MCP23017_GPINTENA, 0xFF);
	mcp23017_writebyte(MCP23017_INTCONA, 0xFF);
}
