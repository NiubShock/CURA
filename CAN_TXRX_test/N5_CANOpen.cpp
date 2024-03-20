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
}

bool N5CANOpen :: setMotorData(t_Motor_Data para) {
    t_N5_Frame frame;
    t_N5_Frame frame_rx;
    MCP2515::t_MCP2515_CAN_Frame frame_mcp;

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

    return true;
}

bool N5CANOpen :: startAutoCalibration() {

    bool ret = false;
    t_N5_TXPDO  txpdo;
    t_N5_Frame  frame_rx;
    uint8_t     data_sampe[8];
    uint8_t     data_Empty[8];

    /* Set to operational state */
    switchState(OPERATIONAL_STATE, 0x00);
    delay(1000);

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x2300, 0x00, 0x00, frame_rx.array);
    printCANData(frame_rx);

    rxpdo.b.r_6040 = 0x06;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);

    /* READ */
    do {
        sendFrameWAnswer(0x601, 8, N5_SDO_UP_REQ, 0x6041, 0x00, 0x00, frame_rx.array);
        SERIAL_PORT_MONITOR.println(frame_rx.b.p.payload_32t, HEX);
        delay(10);
    } while((frame_rx.b.p.payload_32t & 0x221) != 0x221);

    rxpdo.b.r_6060 = 0xFE;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);

    delay(100);
    rxpdo.b.r_6040 = 0x07;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);

    ret = checkOBJbits(0x6041, 0x00, 0x233, 1000);
    if (ret == false) return false;

    rxpdo.b.r_6040 = 0x0F;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);
    printCANData(frame_rx);

    ret = checkOBJbits(0x6041, 0x00, 0x237, 1000);
    if (ret == false) return false;

    ret = checkOBJbits(0x6061, 0x00, 0xFE, 1000);
    if (ret == false) return false;

    rxpdo.b.r_6040 = 0x1F;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);
    printCANData(frame_rx);

    ret = checkOBJbits(0x6041, 0x00, 0x1237, 1000);
    if (ret == false) return false;
    
    rxpdo.b.r_6040 = 0x00;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);
    printCANData(frame_rx);

}

bool N5CANOpen :: startVelocityProfile(uint16_t speed) {

    t_N5_Frame  frame_rx;
    bool ret = false;

    SERIAL_PORT_MONITOR.println("Starting control loop");

    sendFrameWAnswer(0x601, 8, N5_SDO_UP_REQ, 0x3202, 0x00, 0x00, frame_rx.array);

    rxpdo.b.r_6060 = 0x02;
    rxpdo.b.r_3202 = frame_rx.b.p.payload_32t | 0x80;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);

    rxpdo.b.r_6042_6071 = speed;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);

    rxpdo.b.r_6040 = 0x06;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);
    ret = checkOBJbits(0x6041, 0x00, 0x221, 1000);
    if (ret == false) return false;

    rxpdo.b.r_6040 = 0x07;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);
    ret = checkOBJbits(0x6041, 0x00, 0x233, 1000);
    if (ret == false) return false;

    rxpdo.b.r_6040 = 0x0F;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);
    ret = checkOBJbits(0x6041, 0x00, 0x237, 1000);
    if (ret == false) return false;

    return true;
}

bool N5CANOpen :: startTorqueProfile(uint16_t torque) {

    t_N5_Frame  frame_rx;
    bool ret = false;

    SERIAL_PORT_MONITOR.println("Starting control loop");

    rxpdo.b.r_6060 = 0x04;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);

    rxpdo.b.r_6042_6071 = torque;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);

    rxpdo.b.r_6040 = 0x06;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);
    ret = checkOBJbits(0x6041, 0x00, 0x221, 1000);
    if (ret == false) return false;

    rxpdo.b.r_6040 = 0x07;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);
    ret = checkOBJbits(0x6041, 0x00, 0x233, 1000);
    if (ret == false) return false;

    rxpdo.b.r_6040 = 0x0F;
    sendFrameWAnswer(0x201, 7, rxpdo.array, nullptr);
    ret = checkOBJbits(0x6041, 0x00, 0x237, 1000);
    if (ret == false) return false;

    return true;
}

void N5CANOpen :: stopVelocityProfile() {
    rxpdo.b.r_6040 = 0x06;
    sendFrameWAnswer(0x201, 8, rxpdo.array, nullptr);
}
