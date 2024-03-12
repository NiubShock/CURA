#include <SPI.h>
#include "N5_CANOpen.h"
#include "MCP2515_Registers.h"
#include "MCP2515.h"

#define CAN_2515

const int SPI_CS_PIN  = 9;

N5CANOpen N5;
MCP2515 mcp2515;
uint8_t data;

void setup() {

    // mcp2515.set_cs_pin(SPI_CS_PIN);

    SERIAL_PORT_MONITOR.begin(115200);
    while(!Serial){};

    N5.begin();

    // MCP2515::t_MCP2515_Init_Param   param;
    // MCP2515::t_MCP2515_Mode         canctrl;

    // canctrl.operating_mode      = MODE_NORMAL;
    // canctrl.oneshot_mode        = true;


    // /* 16 time quanta - prop + ps1 = 13 - Seg2 = 2 */
    // param.synch                 = 0;
    // param.prescaler             = 0;
    // param.ps1                   = 2;
    // param.ps2                   = 2;
    // param.propagation_delay     = 0;
    // param.triple_sample_point   = true;

    // param.receive_buff_option   = MCP_RX_FLT_ANY;
    // param.rollover_enable       = false;

    // SERIAL_PORT_MONITOR.print(mcp2515.begin(param, canctrl));
    N5CANOpen :: t_Motor_Data para;

    para.pole_pair                      = 3;
    para.max_current                    = 18000;
    para.max_duration_peak_current      = 1000;
    para.rated_current                  = 12000;
    para.max_perth_current              = 100;

  
    SERIAL_PORT_MONITOR.println(N5.setMotorData(para));

    N5.startAutoCalibration();

}

// uint8_t data_rx[8];

void loop() {

    
    // MCP2515::t_MCP2515_CAN_Frame frame;
    // uint8_t data_arr[] = {0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};

    // frame.ID = 0x601;
    // frame.data_length = 8;
    // frame.data = data_arr;
    // mcp2515.transfer(frame);

    // if(mcp2515.checkRXBuffer(data_rx, 100)){
    //     for (int i = 0; i < 8; i++){
    //         SERIAL_PORT_MONITOR.print(data_rx[i], HEX);
    //         SERIAL_PORT_MONITOR.print(" ");
    //     }
    //     SERIAL_PORT_MONITOR.println(" ");
    // }

    delay(1000);
    // SERIAL_PORT_MONITOR.println("cycle");

}

// END FILE
