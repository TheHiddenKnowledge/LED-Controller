#include "eeprom.h"

void f_start_read()
{
    // Resetting EEPROM position
    NVMADR = 0x380000;
    // Setting control register for read operation
    NVMCON1 = 0x01;
}

uint8_t f_read_byte()
{
    // Setting GO bit 
    NVMCON0 |= _NVMCON0_NVMGO_MASK;
    while(NVMCON0 & _NVMCON0_NVMGO_MASK){}
    return NVMDATL;
}

void f_read_buffer(uint16_t a_length, uint8_t* a_buffer)
{
    uint16_t l_read_pos = NVMADRH << 8 + NVMADRL;
    if((l_read_pos + a_length - 1) <= 0x01FF)
    {
        for(int i = 0; i < a_length; i++)
        {
            a_buffer[i] = f_read_byte();
        }
    }
}

void f_start_write()
{
    // Resetting EEPROM position 
    NVMADR = 0x380000;
    // Setting control register for read operation
    NVMCON1 = 0x04;
}

void f_write_byte(uint8_t a_data)
{
    // Data to be written
    NVMDATL = a_data;
    // Interrupts must be disabled during unlock sequence
    INTCON0 &= ~_INTCON0_GIE_MASK;
    // Performing unlock sequence
    asm("BANKSEL NVMLOCK");
    asm("movlw 0x55");
    asm("movwf " ___mkstr(BANKMASK(NVMLOCK)) ", b");
    asm("movlw 0xAA");
    asm("movwf " ___mkstr(BANKMASK(NVMLOCK)) ", b");
    asm("bsf " ___mkstr(BANKMASK(NVMCON0)) ", 0, b");
    while(NVMCON0 & _NVMCON0_NVMGO_MASK){}
    INTCON0 |= _INTCON0_GIE_MASK;
}

void f_write_buffer(uint16_t a_length, uint8_t* a_buffer)
{
    uint16_t l_read_pos = NVMADRH << 8 + NVMADRL;
    if((l_read_pos + a_length - 1) <= 0x01FF)
    {
        for(int i = 0; i < a_length; i++)
        {
            f_write_byte(a_buffer[i]);
        }
    }
}