#include <SPI.h>
#include "MCP2515_Registers.h"
#include "MCP2515.h"

#define CAN_2515

const int SPI_CS_PIN  = 9;
const int CAN_INT_PIN = 2;

byte stmp[8] = {10, 20, 30, 40, 50, 60, 70, 80};
byte error;

// #define READ_INSTRUCTION    3
// #define WRITE_INSTRUCTION   2

// #define MCP2515_TXB1CTRL    0x30
// #define MCP2515_TXB2CTRL    0x40
// #define MCP2515_TXB3CTRL    0x50


MCP2515 mcp2515;
uint8_t data;

void setup() {

    mcp2515.set_cs_pin(SPI_CS_PIN);

    SERIAL_PORT_MONITOR.begin(115200);
    while(!Serial){};

    MCP2515::t_MCP2515_Init_Param   param;
    MCP2515::t_MCP2515_Mode         canctrl;

    canctrl.operating_mode      = MODE_NORMAL;
    canctrl.oneshot_mode        = true;


    /* 16 time quanta - prop + ps1 = 13 - Seg2 = 2 */
    param.synch                 = 0;
    param.prescaler             = 0;
    param.ps1                   = 2;
    param.ps2                   = 2;
    param.propagation_delay     = 0;
    param.triple_sample_point   = true;

    // param.receive_buff_option   = MCP_RX_FLT_ANY;
    // param.rollover_enable       = false;

    SERIAL_PORT_MONITOR.print(mcp2515.begin(param, canctrl));


}

uint8_t data_rx[8];

void loop() {
    MCP2515::t_MCP2515_CAN_Frame frame;
    uint8_t data_arr[] = {0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};

    frame.ID = 0x601;
    frame.data_length = 8;
    frame.data = data_arr;
    mcp2515.transfer(frame);

    delay(100);

    // if(mcp2515.checkRXBuffer(data_rx)){
    //     for (int i = 0; i < 8; i++){
    //         SERIAL_PORT_MONITOR.print(data_rx[i], HEX);
    //         SERIAL_PORT_MONITOR.print(" ");
    //     }
    //     SERIAL_PORT_MONITOR.println(" ");
    // }

}

// END FILE
