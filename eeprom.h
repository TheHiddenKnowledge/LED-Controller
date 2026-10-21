#pragma once

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

// Initiates EEPROM read
void f_start_read(void);

// Reads single byte from EEPROM
uint8_t f_read_byte(void);

// Read array of bytes from EEPROM
void f_read_buffer(uint16_t a_length, uint8_t* a_buffer);

// Initiates EEPROM write
void f_start_write(void);

// Writes single byte to EEPROM
void f_write_byte(uint8_t a_data);

// Write array of bytes to EEPROM
void f_write_buffer(uint16_t a_length, uint8_t* a_buffer);
