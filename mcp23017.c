/*
 * MCP23017
 */

#include "mcp32017.h"
#include "i2cmaster.h"

/*
 * write a byte
 */
void mcp23017_writebyte(uint8_t address, uint8_t reg, uint8_t data) {
	i2c_start_wait(I2C_WRITE + MCP23017_BASEADDRESS + address);
	i2c_write(reg);
	i2c_write(data);
	i2c_stop();
}

/*
 * write a word
 */
void mcp23017_writeword(uint8_t address, uint8_t reg, uint16_t data) {
	i2c_start_wait(I2C_WRITE + MCP23017_BASEADDRESS + address);
	i2c_write(reg);
	i2c_write((uint8_t)data);
	i2c_write((uint8_t)(data>>8));
	i2c_stop();
}

/*
 * read a byte
 */
uint8_t mcp23017_readbyte(uint8_t address, uint8_t reg) {
	i2c_start_wait(I2C_WRITE + MCP23017_BASEADDRESS + address);
	i2c_write(reg);
	i2c_rep_start(I2C_READ + MCP23017_BASEADDRESS + address);
	uint8_t data = i2c_readNak();
	i2c_stop();
	return data;
}

