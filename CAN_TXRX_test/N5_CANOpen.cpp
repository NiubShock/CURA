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
    // frame.command       = N5_SDO_DOWN_CMD_4B;
    // frame.i.index       = 0x203B;
    // frame.i.subindex    = 0x02;
    // frame.p.payload_32t = ;

    // frame_mcp.data = frame;
    // mcp2515.transfer(frame_mcp);

    /* Autosetup */

    /* Load encoder para */

    uint16_t add_reg = 0x6073;
    uint8_t reg_subind = 0x00;
    uint8_t reg_size = 0x10;

    setRXPDO(&add_reg, &reg_subind, &reg_size, 1);

    // frame.b.command         = N5_SDO_DOWN_CMD_4B;
    // frame.b.i.index         = 0x1400;
    // frame.b.subindex        = 0x01;
    // frame.b.p.payload_32t   = 0x80000201;

    // frame_mcp.data = frame.array;
    // mcp2515.transfer(frame_mcp);
    // mcp2515.checkRXBuffer(frame_rx.array, 100);

    // /* Load max duration in obj 0x1600:00 */
    // frame.b.command         = N5_SDO_DOWN_CMD_1B;
    // frame.b.i.index         = 0x1600;
    // frame.b.subindex        = 0x00;
    // frame.b.p.payload_32t   = 0x00;

    // frame_mcp.data = frame.array;
    // mcp2515.transfer(frame_mcp);
    // mcp2515.checkRXBuffer(frame_rx.array, 100);

    // /* Load the object 6073:00 16bits in 1600:01 */
    // frame.b.command         = N5_SDO_DOWN_CMD_4B;
    // frame.b.i.index         = 0x1600;
    // frame.b.subindex        = 0x01;
    // frame.b.p.payload_32t   = 0x60730010; 

    // frame_mcp.data = frame.array;
    // mcp2515.transfer(frame_mcp);
    // mcp2515.checkRXBuffer(frame_rx.array, 100);

    // /* Load max duration in obj 0x1600:00 */
    // frame.b.command         = N5_SDO_DOWN_CMD_1B;
    // frame.b.i.index         = 0x1600;
    // frame.b.subindex        = 0x00;
    // frame.b.p.payload_32t   = 0x01;

    // frame_mcp.data = frame.array;
    // mcp2515.transfer(frame_mcp);
    // mcp2515.checkRXBuffer(frame_rx.array, 100);

    uint8_t data_sampe[8];
    data_sampe[0] = 0x00;
    data_sampe[1] = 0x64;
    /* PDO Configuration */
    frame_mcp.ID            = 0x201;
    /* Load max current perth in obj 0x6073:00 */
    // frame.b.command         = N5_SDO_DOWN_CMD_2B;
    // frame.b.i.index         = 0x6073;
    // frame.b.subindex        = 0x00;
    // frame.b.p.payload_32t   = para.max_perth_current;

    frame_mcp.data = data_sampe;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);
    
    // if (!checkTXAnswer(frame, frame_rx)) return false;

    // startAutoCalibration();

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

    frame.b.command         = N5_SDO_DOWN_CMD_4B;
    frame.b.i.index         = 0x1400;
    frame.b.subindex        = 0x01;
    frame.b.p.payload_32t   = 0x80000201;

    frame_mcp.data = frame.array;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);

    /* Load max duration in obj 0x1600:00 */
    frame.b.command         = N5_SDO_DOWN_CMD_1B;
    frame.b.i.index         = 0x1600;
    frame.b.subindex        = 0x00;
    frame.b.p.payload_32t   = 0x00;

    frame_mcp.data = frame.array;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);

    /* Load the object 6060:00 16bits in 1600:01 */
    frame.b.command         = N5_SDO_DOWN_CMD_4B;
    frame.b.i.index         = 0x1600;
    frame.b.subindex        = 0x01;
    frame.b.p.payload_32t   = 0x60600008; 

    frame_mcp.data = frame.array;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);

    /* Load the object 6040:00 16bits in 1600:02 */
    frame.b.command         = N5_SDO_DOWN_CMD_4B;
    frame.b.i.index         = 0x1600;
    frame.b.subindex        = 0x02;
    frame.b.p.payload_32t   = 0x60400010; 

    frame_mcp.data = frame.array;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);

    /* Load max duration in obj 0x1600:00 */
    frame.b.command         = N5_SDO_DOWN_CMD_1B;
    frame.b.i.index         = 0x1600;
    frame.b.subindex        = 0x00;
    frame.b.p.payload_32t   = 0x02;

    frame_mcp.data = frame.array;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);

    uint8_t data_sampe[8];
    for (int i = 0; i < 8; i++) data_sampe[i] = 0;

    data_sampe[7]           = 0xFE;     // select autosetup
    data_sampe[6]           = 0x04;     // start autosetup
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

uint8_t N5CANOpen :: loadDownloadSize(uint8_t size) {
    if (size == 1) return N5_SDO_DOWN_CMD_1B;
    if (size == 2) return N5_SDO_DOWN_CMD_2B;
    if (size == 3) return N5_SDO_DOWN_CMD_3B;
    if (size == 4) return N5_SDO_DOWN_CMD_4B;

    return 0;
}

void N5CANOpen :: setRXPDO(uint16_t *ptr_register, uint8_t *ptr_subindex, uint8_t *ptr_reg_size, uint8_t size) {
    
    t_N5_Frame frame;
    t_N5_Frame frame_rx;
    MCP2515::t_MCP2515_CAN_Frame frame_mcp;

    frame_mcp.ID            = 0x601;
    frame_mcp.data_length   = 8;

    /* Enable the modification */
    frame.b.command         = N5_SDO_DOWN_CMD_4B;
    frame.b.i.index         = 0x1400;
    frame.b.subindex        = 0x01;
    frame.b.p.payload_32t   = 0x80000201;

    frame_mcp.data = frame.array;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);

    /* Disable the profile */
    frame.b.command         = N5_SDO_DOWN_CMD_1B;
    frame.b.i.index         = 0x1600;
    frame.b.subindex        = 0x00;
    frame.b.p.payload_32t   = 0x00;

    frame_mcp.data = frame.array;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);

    /* Load the objects */
    for (int i = 0; i < size; i++) {
        frame.b.command             = loadDownloadSize(*(ptr_reg_size + i));
        frame.b.i.index             = 0x1600;
        frame.b.subindex            = 1 + i;

        frame.b.p.rx_pdo.size       = *(ptr_reg_size + i);
        frame.b.p.rx_pdo.subindex   = *(ptr_subindex + i);
        frame.b.p.rx_pdo.index      = *(ptr_register + i);

        frame_mcp.data = frame.array;
        mcp2515.transfer(frame_mcp);
        mcp2515.checkRXBuffer(frame_rx.array, 100);
    }

    /* Load max duration in obj 0x1600:00 */
    frame.b.command         = N5_SDO_DOWN_CMD_1B;
    frame.b.i.index         = 0x1600;
    frame.b.subindex        = 0x00;
    frame.b.p.payload_32t   = size;

    frame_mcp.data = frame.array;
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_rx.array, 100);
}

void N5CANOpen :: startPositionProfile() {

}