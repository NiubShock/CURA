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

void N5CANOpen :: clearFrame(t_N5_Frame *frame){
    for (int i = 0; i < 8; i++){
        frame ->array[i] = 0;
    }
}

void N5CANOpen :: loadDictionary(uint8_t cmd, uint16_t index, uint8_t subindex, uint8_t ptr_payload[], uint8_t size, t_N5_Frame * frame){
    frame ->b.command     = cmd;
    frame ->b.subindex    = subindex;

    frame ->b.i.index = index;

    for (int i = 0; i < 4; i++){
        frame ->b.p.payload[i] = 0;
    }

    for (int i = 0; i < size; i++){
        frame ->b.p.payload[4 - i] = ptr_payload[i];
    }
}

bool N5CANOpen :: checkTXAnswer(t_N5_Frame frame_tx, t_N5_Frame frame_rx) {
    if (frame_rx.b.command != N5_SDO_DOWN_OK) {
        SERIAL_PORT_MONITOR.println("Error CMD");
        return false;
    }

    if (frame_rx.b.subindex != frame_tx.b.subindex) {
        SERIAL_PORT_MONITOR.println("Error Subindex");
        return false;
    }

    if (frame_rx.b.i.index != frame_tx.b.i.index){
        SERIAL_PORT_MONITOR.print(frame_rx.b.i.index);
        SERIAL_PORT_MONITOR.print(" != ");
        SERIAL_PORT_MONITOR.println(frame_tx.b.i.index);
        return false;
    }

    return true;
}

bool N5CANOpen :: setMotorData(t_Motor_Data para) {
    t_N5_Frame frame;
    t_N5_Frame frame_rx;
    MCP2515::t_MCP2515_CAN_Frame frame_mcp;

    clearFrame(&frame);

    frame_mcp.ID            = 0x601;
    frame_mcp.data_length   = 8;

    /* Load polepairs in obj 0x2030:00 */
    frame.b.command         = N5_SDO_DOWN_CMD_4B;
    frame.b.i.index         = 0x2030;
    frame.b.subindex        = 0x00;
    frame.b.p.payload_32t   = para.pole_pair;

    frame_mcp.data          = frame.array;
    // mcp2515.transfer(frame_mcp);
    // mcp2515.checkRXBuffer(frame_rx.array, 100);
    
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

    /* Load max current perth in obj 0x6073:00 */
    frame.b.command       = N5_SDO_DOWN_CMD_2B;
    frame.b.i.index       = 0x6073;
    frame.b.subindex    = 0x00;
    frame.b.p.payload_32t = para.max_perth_current;

    frame_mcp.data = frame.array;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);
    
    // if (!checkTXAnswer(frame, frame_rx)) return false;

    /* Load max duration in obj 0x203B:02 */
    frame.b.command       = N5_SDO_DOWN_CMD_4B;
    frame.b.i.index       = 0x203B;
    frame.b.subindex    = 0x02;
    frame.b.p.payload_32t = para.max_duration_peak_current;

    frame_mcp.data = frame.array;
    // mcp2515.transfer(frame_mcp);
    // mcp2515.checkRXBuffer(frame_rx.array, 100);
    
    // if (!checkTXAnswer(frame, frame_rx)) return false;

    /* Load BLDC type in 0x3202:00 -> 0x00000040*/
    frame.b.command       = N5_SDO_DOWN_CMD_4B;
    frame.b.i.index       = 0x203B;
    frame.b.subindex    = 0x02;
    frame.b.p.payload_32t = 0x00000040;

    frame_mcp.data = frame.array;
    // mcp2515.transfer(frame_mcp);
    // mcp2515.checkRXBuffer(frame_rx.array, 100);
    
    // if (!checkTXAnswer(frame, frame_rx)) return false;

    /* Load encoder type in obj 0x2059:00 */
    // frame.command       = N5_SDO_DOWN_CMD_4B;
    // frame.i.index       = 0x203B;
    // frame.i.subindex    = 0x02;
    // frame.p.payload_32t = ;

    // frame_mcp.data = frame;
    // mcp2515.transfer(frame_mcp);

    /* Autosetup */

    /* Load encoder para */

    return true;
}