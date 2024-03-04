#include "N5_CANOpen.h"
#include "MCP2515.h"

const int SPI_CS_PIN  = 9;

void N5CANOpen :: begin(){
    mcp2515.set_cs_pin(SPI_CS_PIN);

    MCP2515::t_MCP2515_Init_Param   param;
    MCP2515::t_MCP2515_Mode         canctrl;

    canctrl.operating_mode      = MODE_NORMAL;
    canctrl.oneshot_mode        = true;

    param.synch                 = 0;
    param.prescaler             = 0;
    param.ps1                   = 2;
    param.ps2                   = 2;
    param.propagation_delay     = 0;
    param.triple_sample_point   = true;

    param.receive_buff_option   = MCP_RX_FLT_ANY;
    param.rollover_enable       = false;

    SERIAL_PORT_MONITOR.print(mcp2515.begin(param, canctrl));

    /* Write the PDO mapping for the RX - 6040:00, 6060:00, 6042:00 */
    defPDOMapping();
}

bool N5CANOpen :: setMotorData(t_Motor_Data para) {
    t_N5_Frame frame;
    t_N5_Frame frame_rx;
    MCP2515::t_MCP2515_CAN_Frame frame_mcp;

    // mcp2515.checkRXBuffer(frame_rx.array, 100);

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x2030, 0x00, para.pole_pair, frame_rx.array);
    printCANData(frame_rx);

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x2031, 0x00, para.max_current, frame_rx.array);
    printCANData(frame_rx);

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x203B, 0x01, para.rated_current, frame_rx.array);
    printCANData(frame_rx);

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x203B, 0x02, para.max_duration_peak_current, frame_rx.array);
    printCANData(frame_rx);

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x3202, 0x00, 0x40, frame_rx.array);
    printCANData(frame_rx);

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x2059, 0x00, 0x00, frame_rx.array);
    printCANData(frame_rx);

    /* Autosetup */

    /* Load encoder para */


    // uint16_t add_reg = 0x6073;
    // uint8_t reg_subind = 0x00;
    // uint8_t reg_size = 0x10;

    // setRXPDO(&add_reg, &reg_subind, &reg_size, 1);

    // uint8_t data_sampe[8];
    // for (int i = 0; i < 8; i++) data_sampe[i] = 0;
    // /* PDO Configuration */
    // data_sampe[0] = 0x64;
    // frame_mcp.ID  = 0x201;

    // frame_mcp.data = data_sampe;
    // mcp2515.transfer(frame_mcp);

    // startAutoCalibration();

    return true;
}

void N5CANOpen :: setControlLoop() {

}

void N5CANOpen :: startAutoCalibration() {
    t_N5_RXPDO  rxpdo;
    t_N5_TXPDO  txpdo;
    t_N5_Frame  frame_rx;
    uint8_t     data_sampe[8];

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x2300, 0x00, 0x00, frame_rx.array);
    printCANData(frame_rx);

    for(int i = 0; i < 8; i++) rxpdo.array[i] = 0;
    rxpdo.b.r_6040 = 0x06;
    sendFrameWAnswer(0x201, 8, rxpdo.array, nullptr);
    printCANData(frame_rx);

    /* READ */
    do {
        sendFrameWAnswer(0x601, 8, N5_SDO_UP_REQ, 0x6041, 0x00, 0x00, txpdo.array);
        // printCANData(txpdo);
    } while(txpdo.b.r_6041 != 0x221);

    // rxpdo.b.r_6040 = 0x07;
    // rxpdo.b.r_6060 = 0xFE;
    // sendFrameWAnswer(0x201, 8, rxpdo.array);
    // printCANData(frame_rx);

    // /* READ */

    // rxpdo.b.r_6040 = 0x0F;
    // sendFrameWAnswer(0x201, 8, rxpdo.array);
    // printCANData(frame_rx);

    // /* READ */

    // /* READ */

    // rxpdo.b.r_6040 = 0x1F;
    // sendFrameWAnswer(0x201, 8, rxpdo.array);
    // printCANData(frame_rx);

    // /* READ */
    
    // rxpdo.b.r_6040 = 0x00;
    // sendFrameWAnswer(0x201, 8, rxpdo.array);
    // printCANData(frame_rx);

}

void N5CANOpen :: startPositionProfile() {

}
