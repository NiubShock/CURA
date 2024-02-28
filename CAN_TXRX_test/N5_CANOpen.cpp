#include "N5_CANOpen.h"
#include "MCP2515.h"

const int SPI_CS_PIN  = 9;
static MCP2515 mcp2515;

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
}

bool N5CANOpen :: setMotorData(t_Motor_Data para) {
    t_N5_Frame frame;
    t_N5_Frame frame_rx;
    MCP2515::t_MCP2515_CAN_Frame frame_mcp;

    for (int i = 0; i < 8; i++) frame.array[i] = 0;

    frame_mcp.ID            = 0x601;
    frame_mcp.data_length   = 8;

    /* Load polepairs in obj 0x2030:00 */
    frame.b.command         = N5_SDO_DOWN_CMD_4B;
    frame.b.i.index         = 0x2030;
    frame.b.subindex        = 0x00;
    frame.b.p.payload_32t   = para.pole_pair;

    frame_mcp.ID            = frame.b.ID;
    frame_mcp.data_length   = frame.b.data_length;
    frame_mcp.data          = frame.array;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);
    
    // if (!checkTXAnswer(frame, frame_rx)) return false;

    /* Load max current in obj 0x2031:00 */
    frame.b.i.index         = 0x2031;
    frame.b.subindex        = 0x00;
    frame.b.p.payload_32t   = para.max_current;

    frame_mcp.data          = frame.array;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);
    
    // if (!checkTXAnswer(frame, frame_rx)) return false;

    /* Load rated current in obj 0x6075:00 */
    frame.b.i.index       = 0x203B;
    frame.b.subindex    = 0x01;
    frame.b.p.payload_32t = para.rated_current;

    frame_mcp.data = frame.array;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);
    
    // if (!checkTXAnswer(frame, frame_rx)) return false;

    /* Load max duration in obj 0x203B:02 */
    frame.b.command         = N5_SDO_DOWN_CMD_4B;
    frame.b.i.index         = 0x203B;
    frame.b.subindex        = 0x02;
    frame.b.p.payload_32t   = para.max_duration_peak_current;

    frame_mcp.data = frame.array;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);
    
    // if (!checkTXAnswer(frame, frame_rx)) return false;

    /* Load BLDC type in 0x3202:00 -> 0x00000040*/
    frame.b.command         = N5_SDO_DOWN_CMD_4B;
    frame.b.i.index         = 0x3202;
    frame.b.subindex        = 0x00;
    frame.b.p.payload_32t   = 0x00000040;

    frame_mcp.data = frame.array;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);
    
    // if (!checkTXAnswer(frame, frame_rx)) return false;

    /* Load encoder type in obj 0x2059:00 */
    frame.command       = N5_SDO_DOWN_CMD_4B;
    frame.i.index       = 0x2059;
    frame.i.subindex    = 0x00;
    frame.p.payload_32t = 0x00;

    frame_mcp.data = frame;
    mcp2515.transfer(frame_mcp);

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

    defPDOMapping();

    startAutoCalibration();

    return true;
}

void N5CANOpen :: setControlLoop() {

}

void N5CANOpen :: startAutoCalibration() {
    t_N5_Frame frame;
    t_N5_Frame frame_rx;
    MCP2515::t_MCP2515_CAN_Frame frame_mcp;

    frame_mcp.ID            = 0x601;
    frame_mcp.data_length   = 8;

    /* Disable nanoJ */
    frame.b.command         = N5_SDO_DOWN_CMD_4B;
    frame.b.i.index         = 0x2030;
    frame.b.subindex        = 0x00;
    frame.b.p.payload_32t   = 0x00;

    frame_mcp.data = frame.array;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);

    uint16_t add_reg[] = {0x6060, 0x6040};
    uint8_t reg_subind[] = {0x00, 0x00};
    uint8_t reg_size[] = {0x08, 0x10};

    setRXPDO(add_reg, reg_subind, reg_size, 2);

    uint8_t data_sampe[8];
    for (int i = 0; i < 8; i++) data_sampe[i] = 0;

    data_sampe[7]           = 0xFE;     // select autosetup
    data_sampe[6]           = 0x08;     // start autosetup
    frame_mcp.ID            = 0x201;

    frame_mcp.data = data_sampe;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);

    frame_mcp.ID            = 0x601;

    do {
        delay(1000);
        frame.b.command         = N5_SDO_UP_REQ;
        frame.b.i.index           = 0x6041;
        frame.b.subindex        = 0x00;
        frame.b.p.payload_32t   = 0;

        frame_mcp.data = frame.array;
        mcp2515.transfer(frame_mcp);
        mcp2515.checkRXBuffer(frame_rx.array, 100);
    } while ((frame_rx.b.p.payload_32t & (1 << 12)) == 0);
}

void N5CANOpen :: startPositionProfile() {

}