#ifndef EXTERNAL_I2C
#define EXTERNAL_I2C
#include "defines.h" 
#include "src/SlowSoftI2CMaster.h"
#include <stdint.h>
/*
 * This header (and the corresponding .cpp file) implement functions for 
 * communicating with add-on devices over the USB-C and other externally 
 * available pins. The preffered means of communication is via IIC (aka I2C).
 * 
 * 
*/

/**
This function sends and receives data on the USB-C I2C line. In read mode, the 
corresponding bytes in the data_array will be filled with the read byte from 
the I2C.

uint8_t address: 7-bit address for the target I2C device (do not include the R/W 
bit)

uint8_t* data_array: byte array of data to send and receive bytes.

uint8_t* read_mode: indicates whether this is a read or writer operation 
(true = read, false = write)

int buffer_size: Size of data_array
*/
void external_i2c(uint8_t address, uint8_t * data_buffer, bool read_mode, int buffer_size);

#endif
