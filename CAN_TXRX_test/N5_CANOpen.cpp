#include "N5_CANOpen.h"
#include "MCP2515.h"
#include "arduino.h"

const int SPI_CS_PIN  = 9;

void N5CANOpen :: begin(){
    t_N5_Frame frame_rx;
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

    /* Set to preoperational state */
    switchState(PREOP_STATE, 0x00);
    delay(1000);
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

    rxpdo.b.r_6073 = para.max_perth_current;
    sendFrameWAnswer(0x201, 8, rxpdo.array, nullptr);

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

    return true;
}

void N5CANOpen :: setControlLoop() {

}

void N5CANOpen :: startAutoCalibration() {
    t_N5_TXPDO  txpdo;
    t_N5_Frame  frame_rx;
    uint8_t     data_sampe[8];
    uint8_t     data_Empty[8];

    /* Set to operational state */
    switchState(OPERATIONAL_STATE, 0x00);
    delay(1000);

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x2300, 0x00, 0x00, frame_rx.array);
    printCANData(frame_rx);

    SERIAL_PORT_MONITOR.println("Send frame 1");
    rxpdo.b.r_6040 = 0x06;
    sendFrameWAnswer(0x201, 8, rxpdo.array, nullptr);

    /* READ */
    // do {
    //     sendFrameWAnswer(0x601, 8, N5_SDO_UP_REQ, 0x6041, 0x00, 0x00, frame_rx.array);
    //     delay(10);
    // } while((frame_rx.b.p.payload_32t & 0x221) != 0x221);

    SERIAL_PORT_MONITOR.println("Send frame 2");

    /* Set to operational state */
    switchState(OPERATIONAL_STATE, 0x00);
    delay(1000);

    rxpdo.b.r_6060 = 0xFE;
    sendFrameWAnswer(0x201, 8, rxpdo.array, nullptr);

    delay(100);
    rxpdo.b.r_6040 = 0x07;
    sendFrameWAnswer(0x201, 8, rxpdo.array, nullptr);

    /* Set to operational state */
    switchState(OPERATIONAL_STATE, 0x00);
    delay(1000);

    /* READ */
    // do {
    //     sendFrameWAnswer(0x601, 8, N5_SDO_UP_REQ, 0x6041, 0x00, 0x00, frame_rx.array);
    //     // SERIAL_PORT_MONITOR.println(frame_rx.b.p.payload_32t, HEX);
    //     delay(10);
    // } while((frame_rx.b.p.payload_32t & 0x233) != 0x233);

    SERIAL_PORT_MONITOR.println("Send frame 3");
    rxpdo.b.r_6040 = 0x0F;
    sendFrameWAnswer(0x201, 8, rxpdo.array, nullptr);
    printCANData(frame_rx);

    /* READ */
    // do {
    //     sendFrameWAnswer(0x601, 8, N5_SDO_UP_REQ, 0x6041, 0x00, 0x00, frame_rx.array);
    //     // SERIAL_PORT_MONITOR.println(frame_rx.b.p.payload_32t & 0x27, HEX);
    //     delay(10);
    // } while((frame_rx.b.p.payload_32t & 0x237) != 0x237);

    SERIAL_PORT_MONITOR.println("Reading 2 ");
    /* READ */
    // do {
    //     sendFrameWAnswer(0x601, 8, N5_SDO_UP_REQ, 0x6061, 0x00, 0x00, frame_rx.array);
    //     delay(10);
    // } while(frame_rx.b.p.payload_32t != 0xFE);

    SERIAL_PORT_MONITOR.println("Send frame 4");
    rxpdo.b.r_6040 = 0x1F;
    sendFrameWAnswer(0x201, 8, rxpdo.array, nullptr);
    printCANData(frame_rx);

    /* READ */
    do {
        sendFrameWAnswer(0x601, 8, N5_SDO_UP_REQ, 0x6041, 0x00, 0x00, frame_rx.array);
        // SERIAL_PORT_MONITOR.println(frame_rx.b.p.payload_32t, HEX);
        delay(10);
    } while((frame_rx.b.p.payload_32t & 0x1237) != 0x1237);
    
    SERIAL_PORT_MONITOR.println("Send frame 5");
    rxpdo.b.r_6040 = 0x00;
    sendFrameWAnswer(0x201, 8, rxpdo.array, nullptr);
    printCANData(frame_rx);

}

void N5CANOpen :: startPositionProfile() {

    SERIAL_PORT_MONITOR.println("Send frame 1");

    rxpdo.b.r_6060 = 0x02;
    sendFrameWAnswer(0x201, 8, rxpdo.array, nullptr);

    rxpdo.b.r_6042 = 0xC8;
    sendFrameWAnswer(0x201, 8, rxpdo.array, nullptr);

    rxpdo.b.r_6040 = 0x06;
    sendFrameWAnswer(0x201, 8, rxpdo.array, nullptr);
    delay(100);

    rxpdo.b.r_6040 = 0x07;
    sendFrameWAnswer(0x201, 8, rxpdo.array, nullptr);
    delay(100);

    rxpdo.b.r_6040 = 0x0F;
    sendFrameWAnswer(0x201, 8, rxpdo.array, nullptr);
    delay(100);


    // delay(10000);
    // rxpdo.b.r_6040 = 0x06;
    // sendFrameWAnswer(0x201, 8, rxpdo.array, nullptr);

}
