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
This function sends and receives data on the USB-C I2C line. It is capable of 
mixed read and write communications by specifying which bytes are read or write 
with the rw_array variable. For each 1 in rw_array, the corresponding byte in the 
data_array will be filled with the read byte frmo the I2C.

uint8_t address: 7-bit address for the target I2C device (do not include the R/W 
bit)

uint8_t* data_array: byte array of data to send and receive bytes. Must be the 
same size as rw_array.

uint8_t* rw_array: byte array to indicate which bytes in the data_array are for 
writing (0) or reading (1). Must be the same size as data_array.

int buffer_size: Size of both data_array and rw_array
*/
void external_i2c(uint8_t address, uint8_t * data_array, uint8_t * rw_array, int buffer_size);

#endif
