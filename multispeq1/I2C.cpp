#include "I2C.h"
// See the .h file for documentation

const uint8_t I2C_READ_BIT = 1;
const uint8_t I2C_WRITE_BIT = 0; 

void external_i2c(uint8_t address, uint8_t * data_buffer, uint8_t * rw_array, int buffer_size) {
	// first, check if this is a read-only or mixed read-write
	bool readonly = true;
	for(int i = 0; i < buffer_size; i++){
		if(rw_array[i] == I2C_WRITE){
			readonly = false;
			break;
		}
	}
	uint8_t realAddress;
	if(readonly){
		realAddress = (address << 1) | I2C_READ;
	} else {
		realAddress = (address << 1) | I2C_WRITE;
	}
	// initialize I2C if it isn't ready
	SlowSoftI2CMaster i2c_bus = SlowSoftI2CMaster(EXTERNAL_I2C_DATA, EXTERNAL_I2C_CLOCK, true);
	if( digitalRead(EXTERNAL_I2C_CLOCK) == LOW || digitalRead(EXTERNAL_I2C_DATA) == LOW ){
		i2c_bus.i2c_init();
	}
	// let's get this gravy train moving'!
	i2c_bus.i2c_start(realAddress);
	for(int i = 0; i < buffer_size; i++){
		if(rw_array[i] == I2C_WRITE){
			i2c_bus.i2c_write(data_buffer[i]);
		} else {
			data_buffer[i] = i2c_bus.i2c_read(false);
		}
	}
	// done
	i2c_bus.i2c_stop();
}


