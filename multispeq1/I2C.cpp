#include "I2C.h"
// See the .h file for documentation

const uint8_t I2C_READ_BIT = 1;
const uint8_t I2C_WRITE_BIT = 0; 

void external_i2c(uint8_t address, uint8_t * data_buffer, bool read_mode /* true = read, false = write*/, int buffer_size) {
	uint8_t realAddress;
	if(read_mode){
		realAddress = (address << 1) | I2C_READ;
	} else {
		realAddress = (address << 1) | I2C_WRITE;
	}
	// initialize I2C if it isn't ready
	SlowSoftI2CMaster i2c_bus = SlowSoftI2CMaster(EXTERNAL_I2C_DATA, EXTERNAL_I2C_CLOCK, true);
  i2c_bus.setHigh(EXTERNAL_I2C_CLOCK);
  delayMicroseconds(DELAY);
  i2c_bus.setHigh(EXTERNAL_I2C_DATA);
  delayMicroseconds(DELAY * 2);
  
	// let's get this gravy train moving'!
	i2c_bus.i2c_start(realAddress);
  if(read_mode){
    for(int i = 0; i < buffer_size; i++){
      data_buffer[i] = i2c_bus.i2c_read(false);
    }
  }else{
    for(int i = 0; i < buffer_size; i++){
      i2c_bus.i2c_write(data_buffer[i]);
    }
  }
	// done
	i2c_bus.i2c_stop();
}


