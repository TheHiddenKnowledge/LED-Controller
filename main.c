#include <xc.h>
#include <string.h>
#include "eeprom.h"
#include "comms.h"
#include "millis.h"

#pragma config FEXTOSC = OFF, RSTOSC = HFINTOSC_64MHZ 
#pragma config CSWEN = OFF, FCMEN = OFF, FCMENP = OFF, FCMENS = OFF
#pragma config PWRTS = PWRT_64
#pragma config BORV = VBOR_2P85
#pragma config WDTE = OFF
#pragma config DEBUG = ON

// Status macros
#define STAT_IDLE 0
#define STAT_PEND 1
#define STAT_PROC 2

// Message macros
#define MSG_IDLE 0
#define MSG_RDY 1
#define MSG_ID 2
#define MSG_ERR 3

// Max transmission size
#define TMSN_SIZE 8

// Timeout durations in clock cycles
#define TIMEOUT_L 20000
#define TIMEOUT_R 5000

// Global variables used for communications
uint8_t g_id = 0;
bool g_new_data = false;
uint8_t g_stat[2] = {STAT_IDLE, STAT_IDLE};
uint8_t g_data[2][TMSN_SIZE];
uint8_t g_txmsg[2] = {MSG_IDLE, MSG_IDLE};
uint8_t g_rxmsg[2] = {MSG_IDLE, MSG_IDLE};
uint16_t g_timeout[2] = {0, 0};

// RGB structure
typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} s_rgb; 

// Message structure
typedef struct
{
    uint8_t msg_id;
    uint8_t msg_char;
    s_rgb msg_char_rgb;
    s_rgb msg_bg_rgb;
} s_msg;

// Macros for maximum RAM array sizes 
#define MAX_GRID_COUNT 12
#define MAX_TEXT_COUNT 5
#define MAX_TEXT_SIZE 24
#define MAX_PALETTE_COUNT 10
#define MAX_CODE_SIZE 100

// Global variables used to hold EEPROM data
uint8_t g_grid_count = 0;
uint8_t g_grid_index = 0;
s_msg g_grid_buffer[MAX_GRID_COUNT];
uint8_t g_texts[MAX_TEXT_COUNT][MAX_TEXT_SIZE];
uint8_t g_texts_sizes[MAX_TEXT_COUNT];
s_rgb g_palette[MAX_PALETTE_COUNT]; 
uint8_t g_code_size = 0;
uint8_t g_code[MAX_CODE_SIZE];

// Global variables used for grid manipulation 
uint8_t g_texts_index = 0;
uint16_t g_span_lapse_time = 0;
uint8_t g_text_flip_colors;
bool g_text_flip_index;
uint16_t g_text_flip_start_time = 0;
uint16_t g_text_flip_lapse_time = 0;
uint8_t g_bg_flip_colors;
bool g_bg_flip_index;
uint16_t g_bg_flip_start_time = 0;
uint16_t g_bg_flip_lapse_time = 0;
uint8_t g_shift_offset = 0;
uint16_t g_shift_start_time = 0;
uint16_t g_shift_lapse_time = 0;

// Macros for bytecode interpreter 
#define MAGIC_NUM 0x69
#define CODE_SET_SPAN 0x10
#define CODE_SET_TEXT 0x20
#define CODE_SET_TEXT_COLOR 0x30
#define CODE_SET_BG_COLOR 0x40
#define CODE_FLIP_TEXT_COLOR 0x50
#define CODE_FLIP_BG_COLOR 0x60
#define CODE_SHIFT_LEFT 0x70
#define CODE_SHIFT_BOTH 0x80

// Current code index
uint8_t g_code_index = 0;

// Macros for the interpreter status 
#define CODE_STAT_EN 0x01
#define CODE_STAT_FLIP_TEXT 0x02
#define CODE_STAT_FLIP_BG 0x04
#define CODE_STAT_SHIFT_LEFT 0x08
#define CODE_STAT_SHIFT_BOTH 0x10

// Status register for the interpreter
uint8_t g_code_status = 0x00;

// This is set if code is found in the EEPROM
bool g_code_found = false;

// Translates message structure into array
inline void _f_translate_s_msg(uint8_t *a_buf, s_msg a_msg)
{
    a_buf[0] = a_msg.msg_id;
    a_buf[1] = a_msg.msg_char;
    a_buf[2] = a_msg.msg_char_rgb.r;
    a_buf[3] = a_msg.msg_char_rgb.g;
    a_buf[4] = a_msg.msg_char_rgb.b;
    a_buf[5] = a_msg.msg_bg_rgb.r;
    a_buf[6] = a_msg.msg_bg_rgb.g;
    a_buf[7] = a_msg.msg_bg_rgb.b;
}

// Reads data from EEPROM into RAM
bool f_init_controller()
{
    f_start_read();
    // Only executes if the EEPROM is not empty 
    if(f_read_byte() == MAGIC_NUM)
    {
        // Getting total number of grids
        g_grid_count = f_read_byte();
        // Populating texts array
        uint8_t g_text_count = f_read_byte();
        for(uint8_t i = 0; i < g_text_count; i++)
        {
            g_texts_sizes[i] = f_read_byte();
            f_read_buffer(g_texts_sizes[i], g_texts[i]);
        }
        // Populating palette array
        uint8_t g_palette_count = f_read_byte();
        for(uint8_t i = 0; i < g_palette_count; i++)
        {
            g_palette[i].r = f_read_byte();
            g_palette[i].g = f_read_byte();
            g_palette[i].b = f_read_byte();
        }
        // Populating code array
        g_code_size = f_read_byte();
        f_read_buffer(g_code_size, g_code);
        for(uint8_t i = 0; i < g_grid_count; i++)
        {
            g_grid_buffer[i].msg_id = i + 1;
        }
        return true;
    }
    else
    {
        return false;
    }
}

// Handles communications on the left side
void f_left_comms(void)
{
    switch(g_stat[LEFT])
    {
        case STAT_IDLE:
            // Executed when the left module is ready to send data
            if(g_rxmsg[LEFT] == MSG_RDY)
            {
                // Send confirmation message 
                g_rxmsg[LEFT] = MSG_IDLE;
                g_txmsg[LEFT] = MSG_RDY;
                while(!f_send_bytes(LEFT, &g_txmsg[LEFT], 1))
                {f_proc_send_bytes(LEFT);}
                while(!f_can_send(LEFT))
                {f_proc_send_bytes(LEFT);}
                // Receive quantity of packets to be sent
                while(!f_receive_bytes(LEFT, &g_rxmsg[LEFT], 1))
                {f_proc_receive_bytes(LEFT);}
                while(!f_can_receive(LEFT))
                {f_proc_receive_bytes(LEFT);}
                f_start_write();
                g_stat[LEFT] = STAT_PEND;
            }
            // Idle and waiting for data 
            else
            {
                f_receive_bytes(LEFT, &g_rxmsg[LEFT], 1);
                g_txmsg[LEFT] = MSG_IDLE;
                f_send_bytes(LEFT, &g_txmsg[LEFT], 1);
            }
            break;
        case STAT_PEND:
            // Send confirmation message 
            g_txmsg[LEFT] = MSG_RDY;
            while(!f_send_bytes(LEFT, &g_txmsg[LEFT], 1))
            {f_proc_send_bytes(LEFT);}
            while(!f_can_send(LEFT))
            {f_proc_send_bytes(LEFT);}
            // Receive next packet 
            f_receive_bytes(LEFT, g_data[LEFT], TMSN_SIZE);
            g_stat[LEFT] = STAT_PROC;
            break;
        case STAT_PROC:
            // Data has been successfully received
            if(f_can_receive(LEFT))
            {
                g_timeout[LEFT] = 0;
                f_write_buffer(TMSN_SIZE, g_data[LEFT]);
                g_rxmsg[LEFT]--;
                if(g_rxmsg[LEFT] == 0)
                {
                    g_code_found = f_init_controller();
                    g_rxmsg[LEFT] = MSG_IDLE;
                    g_stat[LEFT] = STAT_IDLE;
                }
                else
                {
                    g_stat[LEFT] = STAT_PEND;
                }
            }
            // Data has not been received and timed out 
            if(g_timeout[LEFT] > TIMEOUT_L)
            {
                // Sending an error message and resetting communications
                f_stop_receive(LEFT);
                g_txmsg[LEFT] = MSG_ERR;
                f_send_bytes(LEFT, &g_txmsg[LEFT], 1);
                g_timeout[LEFT] = 0;
                g_stat[LEFT] = STAT_IDLE;
            }
            g_timeout[LEFT]++;
            break;
    }
    f_proc_receive_bytes(LEFT);
    f_proc_send_bytes(LEFT);
}

// Handles communications on the right side
void f_right_comms(void)
{
    // Constantly checks for messages from the right module
    f_receive_bytes(RIGHT, &g_rxmsg[RIGHT], 1);
    // Handles request for module ID
    if(g_rxmsg[RIGHT] == MSG_ID)
    {
        g_rxmsg[RIGHT] = MSG_IDLE;
        f_stop_receive(RIGHT);
        f_stop_send(RIGHT);
        g_txmsg[RIGHT] = MSG_ID;
        while(!f_send_bytes(RIGHT, &g_txmsg[RIGHT], 1))
        {f_proc_send_bytes(RIGHT);}
        while(!f_can_send(RIGHT))
        {f_proc_send_bytes(RIGHT);}
        while(!f_send_bytes(RIGHT, &g_id, 1))
        {f_proc_send_bytes(RIGHT);}
        while(!f_can_send(RIGHT))
        {f_proc_send_bytes(RIGHT);}
        g_stat[RIGHT] = STAT_IDLE;
        g_grid_index = 0;
        g_new_data = true;
    }
    // Handles error message
    else if(g_rxmsg[RIGHT] == MSG_ERR)
    {
        g_stat[RIGHT] = STAT_IDLE;
        g_grid_index = 0;
        g_new_data = true;
    }
    switch(g_stat[RIGHT])
    {
        case STAT_IDLE:
            // Sends a ready message if new data is present 
            if(g_rxmsg[RIGHT] == MSG_IDLE)
            {
                if(g_new_data)
                {
                    g_txmsg[RIGHT] = MSG_RDY;
                    while(!f_send_bytes(RIGHT, &g_txmsg[RIGHT], 1))
                    {f_proc_send_bytes(RIGHT);}
                    g_stat[RIGHT] = STAT_PEND;
                }
            }
            break;
        case STAT_PEND:
            // Confirmation message has been received
            if(g_rxmsg[RIGHT] == MSG_RDY)
            {
                // Translates message structure for UART transmission
                _f_translate_s_msg(g_data[RIGHT], g_grid_buffer[g_grid_index]);
                // Sends data to the right module 
                while(!f_send_bytes(RIGHT, g_data[RIGHT], TMSN_SIZE))
                {f_proc_send_bytes(RIGHT);}
                g_timeout[RIGHT] = 0;
                g_stat[RIGHT] = STAT_PROC;
            }
            // Confirmation message has not been received and timed out
            else
            {
                if(g_timeout[RIGHT] > TIMEOUT_R)
                {
                    g_timeout[RIGHT] = 0;
                    g_stat[RIGHT] = STAT_IDLE;
                }
                g_timeout[RIGHT]++;
            }
            break;
        case STAT_PROC:
            // Resets communication when all data has been sent
            if(f_can_send(RIGHT))
            {                  
                if(g_rxmsg[RIGHT] == MSG_IDLE)
                {
                    g_grid_index++;
                    if(g_grid_index >= g_grid_count)
                    {
                        g_grid_index = 0;
                        g_new_data = false;
                    }
                    g_stat[RIGHT] = STAT_IDLE;
                }
            }
            break;
    }
    f_proc_receive_bytes(RIGHT);
    f_proc_send_bytes(RIGHT);
}

// Sets the text of the grids 
void f_set_text()
{
    // Offset variable is used to shift the text left or right
    for(int i = 0; i < g_grid_count; i++)
    {
        if(i + g_shift_offset < g_texts_sizes[g_texts_index])
        {
            g_grid_buffer[i].msg_char = 
                    g_texts[g_texts_index][i + g_shift_offset];
        }
        else
        {
            g_grid_buffer[i].msg_char = ' ';
        }
    }
}

// Sets the text color of the grids
void f_set_text_color(uint8_t a_index)
{
    for(int i = 0; i < g_grid_count; i++)
    {
        g_grid_buffer[i].msg_char_rgb = g_palette[a_index];
    }
}

// Sets the background color of the grids
void f_set_bg_color(uint8_t a_index)
{
    for(int i = 0; i < g_grid_count; i++)
    {
        g_grid_buffer[i].msg_bg_rgb = g_palette[a_index];
    }
}

// Shifts the text left
void f_shift_left()
{
    if(g_grid_count + g_shift_offset <= g_texts_sizes[g_texts_index])
    {
        g_shift_offset++;
        f_set_text();
    }
}

// Shifts the text right
void f_shift_right()
{
    if(g_shift_offset > 0)
    {
        g_shift_offset--;
        f_set_text();
    }
}

// Reads and processes bytecode
void f_read_code()
{
    g_code_status = 0x00;
    bool l_start_span = false;
    bool l_read_done = false;
    // The code is read starting and ending between span instructions
    while(!l_read_done)
    {
        if(g_code_index >= g_code_size)
        {
            g_code_index = 0;
            break;
        }
        uint8_t l_byte_code = g_code[g_code_index]; 
        switch(0xF0 & l_byte_code)
        {
            case CODE_SET_SPAN:
                if(l_start_span)
                {
                    l_read_done = true;
                    g_code_index--;
                }
                else
                {
                    l_start_span = true;
                    g_code_index += 2;
                    g_span_lapse_time = (g_code[g_code_index - 1] << 8)
                            + g_code[g_code_index];
                }
                break;
            // The following three instructions are compressed into one byte
            case CODE_SET_TEXT:
                g_texts_index = 0x0F & g_code[g_code_index];
                g_shift_offset = 0;
                f_set_text();
                break;
            case CODE_SET_TEXT_COLOR:
                f_set_text_color(0x0F & g_code[g_code_index]);
                break;
            case CODE_SET_BG_COLOR:
                f_set_bg_color(0x0F & g_code[g_code_index]);
                break;
            // The following four instructions consist of multiple bytes
            case CODE_FLIP_TEXT_COLOR:
                g_code_index++;
                g_text_flip_colors = g_code[g_code_index];
                g_text_flip_index = 1;
                g_code_index += 2;
                g_text_flip_lapse_time = (g_code[g_code_index - 1] << 8)
                            + g_code[g_code_index];
                g_text_flip_start_time = 0;
                f_set_text_color(g_text_flip_colors >> 4);
                g_code_status |= CODE_STAT_FLIP_TEXT;
                break;
            case CODE_FLIP_BG_COLOR:
                g_code_index++;
                g_bg_flip_colors = g_code[g_code_index];
                g_bg_flip_index = 0;
                g_code_index += 2;
                g_bg_flip_lapse_time = (g_code[g_code_index - 1] << 8)
                            + g_code[g_code_index];
                g_bg_flip_start_time = 0;
                f_set_bg_color(g_bg_flip_colors >> 4);
                g_code_status |= CODE_STAT_FLIP_BG;
                break;
            case CODE_SHIFT_LEFT:
                g_shift_lapse_time = g_span_lapse_time 
                        / (g_texts_sizes[g_texts_index]);
                g_shift_start_time = 0;
                g_code_status |= CODE_STAT_SHIFT_LEFT;
                break;
            case CODE_SHIFT_BOTH:
                g_shift_lapse_time = g_span_lapse_time 
                        / (2 * g_texts_sizes[g_texts_index]);
                g_shift_start_time = 0;
                g_code_status |= CODE_STAT_SHIFT_BOTH;
                break;
        }
        g_code_index++;
    }
    g_new_data = true;
    g_grid_index = 0;
    g_code_status |= CODE_STAT_EN;
    f_start_millis();
}

// Executes the processed bytecode
void f_execute_code()
{
    if(g_code_status & CODE_STAT_FLIP_TEXT)
    {
        if(f_has_elapsed(g_text_flip_start_time, g_text_flip_lapse_time))
        {
            (!g_text_flip_index) ? f_set_text_color(g_text_flip_colors >> 4) 
                    : f_set_text_color(g_text_flip_colors & 0x0F);
            g_text_flip_index = !g_text_flip_index;
            g_text_flip_start_time = f_get_millis();
            g_grid_index = 0;
            g_new_data = true;
        }
    }
    if(g_code_status & CODE_STAT_FLIP_BG)
    {
        if(f_has_elapsed(g_bg_flip_start_time, g_bg_flip_lapse_time))
        {
            (!g_bg_flip_index) ? f_set_bg_color(g_bg_flip_colors >> 4) 
                    : f_set_bg_color(g_bg_flip_colors & 0x0F);
            g_bg_flip_index = !g_bg_flip_index;
            g_bg_flip_start_time = f_get_millis();
            g_grid_index = 0;
            g_new_data = true;
        }
    }
    if(g_code_status & CODE_STAT_SHIFT_LEFT)
    {
        if(f_has_elapsed(g_shift_start_time, g_shift_lapse_time))
        {
            if(g_shift_offset < g_texts_sizes[g_texts_index] - 1)
            {
                f_shift_left();
                g_shift_start_time = f_get_millis();
                g_grid_index = 0;
                g_new_data = true;
            }
            
        }
    }
    if(g_code_status & CODE_STAT_SHIFT_BOTH)
    {
        if(f_has_elapsed(g_shift_start_time, g_shift_lapse_time))
        {
            if(f_get_millis() < g_span_lapse_time / 2)
            {
                f_shift_left();
            }
            else
            {
                f_shift_right();
            }
            f_set_text();
            g_shift_start_time = f_get_millis();
            g_grid_index = 0;
            g_new_data = true;
        }
    }
    if(f_has_elapsed(0, g_span_lapse_time))
    {
        g_code_status &= ~CODE_STAT_EN;
        f_stop_millis();
    }
}

// Main function that include the setup and loop
void main(void) {
    // Calling initialization functions for the communications and grid
    g_code_found = f_init_controller();
    f_init_comms();
    f_init_millis();
    // Main loop
    while(1==1)
    {
        f_left_comms();
        f_right_comms();
        if(g_code_found)
        {
            if(g_code_status & CODE_STAT_EN)
            {
                f_execute_code();
            }
            else
            {
                f_read_code();
            }
        }
    }
    return;
}                                                              
