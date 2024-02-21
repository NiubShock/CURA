#ifndef _MCP2515_H_
#define _MCP2515_H_

#include <stdint.h>
#include <SPI.h>
#include "arduino.h"
#include "MCP2515_Registers.h"

class MCP2515 {
    public:
        typedef struct {
            uint8_t operating_mode;
            bool    oneshot_mode;
            bool    clockout_enable;
            uint8_t prescaler_clkout;
        } t_MCP2515_Mode;

        typedef struct {
            uint8_t synch;
            uint8_t ps1;
            uint8_t ps2;
            uint8_t propagation_delay;
            uint8_t prescaler;
            bool    triple_sample_point;

            uint8_t receive_buff_option;
            bool    rollover_enable;
        } t_MCP2515_Init_Param;

        typedef struct {
            uint16_t    ID;
            uint16_t    data_length;
            uint8_t     *data;
        } t_MCP2515_CAN_Frame;

        void set_cs_pin(int pin);

        bool begin(t_MCP2515_Init_Param param, t_MCP2515_Mode CANCTRL);
        void end();
        bool transfer(t_MCP2515_CAN_Frame frame);
        bool checkRXBuffer(uint8_t *ptr_array);
        bool checkRXBuffer(uint8_t *ptr_array, uint16_t delay_ms);

        bool write_register(uint8_t addressv, uint8_t data);
        uint16_t read_register(uint8_t address);

    private:

        int cs_pin = -1;
};

#endif