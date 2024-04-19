#include "N5_CANOpen.h"
#include "MCP2515.h"

void N5CANOpen :: sendFrame(uint16_t ID, uint16_t data_length, uint8_t command, uint16_t index, uint8_t subindex, uint32_t payload, uint8_t *frame_answ){
    t_N5_Frame frame;
    MCP2515::t_MCP2515_CAN_Frame frame_mcp;
    MCP2515::t_MCP2515_CAN_Frame frame_mcp_rx;

    frame.b.command         = command;
    frame.b.i.index         = index;
    frame.b.subindex        = subindex;
    frame.b.p.payload_32t   = payload;

    /* Load the data in the MCP frame */
    frame_mcp.ID            = ID; 
    frame_mcp.data_length   = data_length;
    frame_mcp.data          = frame.array;

    frame_mcp.data          = frame.array;

    /* Send the data and wait for the answer */
    mcp2515.transfer(&frame_mcp);
}

void N5CANOpen :: readCAN(uint8_t *ptr_array){
    // uint8_t ptr_array[8];
    // /* wait for the cycle or just the read boolean becomes true */
    // // for (uint16_t i = 0; i < 10; i++){
    //     // CAN.sendMsgBuf(frame ->ID, 0, frame ->data_length, frame ->data);
    //     CAN.readMsgBufID(0x601, 8, ptr_array);
    //     // delay(1);
    // // }

    // printCANData(ptr_array);

    // return read;

    mcp2515.checkRXBuffer1(0x601, ptr_array, 100);
    // printCANData(ptr_array);
}

void N5CANOpen :: sendFrameWAnswer(uint16_t ID, uint16_t data_length, uint8_t command, uint16_t index, uint8_t subindex, uint32_t payload, uint8_t *frame_answ){
    t_N5_Frame frame;
    MCP2515::t_MCP2515_CAN_Frame frame_mcp;
    MCP2515::t_MCP2515_CAN_Frame frame_mcp_rx;

    frame.b.command         = command;
    frame.b.i.index         = index;
    frame.b.subindex        = subindex;
    frame.b.p.payload_32t   = payload;

    /* Load the data in the MCP frame */
    frame_mcp.ID            = ID; 
    frame_mcp.data_length   = data_length;
    frame_mcp.data          = frame.array;

    frame_mcp.data          = frame.array;

    /* Send the data and wait for the answer */
    mcp2515.transfer(&frame_mcp);
    if (frame_answ != nullptr)  mcp2515.checkRXBuffer(frame_mcp.ID, frame_answ, 100);
}

void N5CANOpen :: sendFrameWAnswer(uint16_t ID, uint16_t data_length, uint8_t *ptr_data, uint8_t *frame_answ) {
    t_N5_Frame frame;
    MCP2515::t_MCP2515_CAN_Frame frame_mcp;
    MCP2515::t_MCP2515_CAN_Frame frame_mcp_rx;

    frame_mcp.ID            = ID;
    frame_mcp.data_length   = data_length;
    frame_mcp.data          = ptr_data;

    mcp2515.transfer(&frame_mcp);
    if (frame_answ != nullptr)  mcp2515.checkRXBuffer(frame_mcp.ID, frame_answ, 100);
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

uint8_t N5CANOpen :: loadDownloadSize(uint8_t size) {
    if (size == 1) return N5_SDO_DOWN_CMD_1B;
    if (size == 2) return N5_SDO_DOWN_CMD_2B;
    if (size == 3) return N5_SDO_DOWN_CMD_3B;
    if (size == 4) return N5_SDO_DOWN_CMD_4B;

    return 0;
}

void N5CANOpen :: setIP(uint32_t IP) {
    t_N5_Frame frame_rx;

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x2010, 0x00, 0x1, frame_rx.array);
    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x2011, 0x00, IP, frame_rx.array);
    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x2012, 0x00, 0xFFFFFF00, frame_rx.array);

    /* Save the configuration */
    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x1010, 0x0C, 0x65766173, frame_rx.array);

    sendFrameWAnswer(0x601, 8, N5_SDO_UP_REQ, 0x2014, 0x00, 0x00, frame_rx.array);
}

void N5CANOpen :: switchState(uint8_t state, uint8_t node_ID) {
    t_N5_Frame frame_rx;
    uint8_t data[8];

    for (int i = 0; i < 8; i++) data[i] = 0;

    data[0] = state;
    data[1] = node_ID;

    sendFrameWAnswer(0x00, 2, data, frame_rx.array);
    printCANData(frame_rx);
}

void N5CANOpen :: setRXPDO(uint16_t *ptr_register, uint8_t *ptr_subindex, uint8_t *ptr_reg_size, uint8_t size) {
    
    t_N5_Frame frame;
    t_N5_Frame frame_rx;
    MCP2515::t_MCP2515_CAN_Frame frame_mcp;

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x1400, 0x01, 0x80000201, frame_rx.array);
    printCANData(frame_rx);

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_1B, 0x1600, 0x00, 0x00, frame_rx.array);
    printCANData(frame_rx);

    /* Load the objects */
    for (int i = 0; i < size; i++) {

        t_N5_Payload payload;

        payload.p.rx_pdo.size       = *(ptr_reg_size + i);
        payload.p.rx_pdo.subindex   = *(ptr_subindex + i);
        payload.p.rx_pdo.index      = *(ptr_register + i);

        sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x1600, 1 + i, payload.p.payload_32t, frame_rx.array);
        printCANData(frame_rx);
    }

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_1B, 0x1600, 0x00, size, frame_rx.array);
    printCANData(frame_rx);

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x1400, 0x01, 0x201, frame_rx.array);
    printCANData(frame_rx);

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x1010, 0x03, 0x65766173, frame_rx.array);
    printCANData(frame_rx);
}

void N5CANOpen :: PDOMapping_Velocity() {
    uint16_t add_reg[]      = {0x6040, 0x6060, 0x6042, 0x6073, 0x3202};
    uint8_t reg_subind[]    = {0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t reg_size[]      = {0x10, 0x8, 0x10, 0x10, 0x20};

    setRXPDO(add_reg, reg_subind, reg_size, 4);
}

void N5CANOpen :: PDO_Close_Loop() {
    uint16_t add_reg[]      = {0x3202};
    uint8_t reg_subind[]    = {0x00};
    uint8_t reg_size[]      = {0x20};

    uint8_t arr[4];
    for(int i = 0; i < 4; i++) arr[i] = 0;
    arr[0] = 0x41;

    setRXPDO(add_reg, reg_subind, reg_size, 1);

    sendFrameWAnswer(0x201, 4, arr, nullptr);
}

void N5CANOpen :: PDOMapping_Torque() {
    uint16_t add_reg[]      = {0x6040, 0x6060, 0x6071, 0x6073, 0x3202};
    uint8_t reg_subind[]    = {0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t reg_size[]      = {0x10, 0x8, 0x10, 0x10, 0x20};

    setRXPDO(add_reg, reg_subind, reg_size, 4);
}

void N5CANOpen :: printCANData(t_N5_Frame frame){
    for (int i = 0; i < 8; i++){
      SERIAL_PORT_MONITOR.print(frame.array[i], HEX);
      SERIAL_PORT_MONITOR.print(" ");
    }
    SERIAL_PORT_MONITOR.println(" ");
}

void N5CANOpen :: printCANData(uint8_t *array) {
    for (int i = 0; i < 8; i++){
      SERIAL_PORT_MONITOR.print(*(array + i), HEX);
      SERIAL_PORT_MONITOR.print(" ");
    }
    SERIAL_PORT_MONITOR.println(" ");
}

uint16_t N5CANOpen :: check6041Status(uint16_t status) {

    /* Ready to switch on is 0b01x0001 */
    if (((status & 0x21) == 0x21)/* && ((status & 0x4E) == 0)*/){
        return READY_TO_ON;
    }

    /* Switched on is 0b01x0011 */
    if (((status & 0x23) == 0x23)/* && ((status & 0x4C) == 0)*/){
        return CURRENTLY_ON;
    }

    /* Operation enabled is 0b01x0111 */
    if (((status & 0x27) == 0x27)/* && ((status & 0x48) == 0)*/){
        return READY_TO_OPERATE;
    }

    /* Operation enabled is 0b1xxxxxxxxxx */
    if (((status & 0x400) == 0x400)){
        return TARGET_REACHED;
    }

}

bool N5CANOpen :: checkOBJbits(uint16_t index, uint16_t subindex, uint16_t bits, uint16_t timeout) {

    N5CANOpen::t_N5_Frame  frame_rx;

    uint16_t    local_timeout = timeout;
    bool        ret_status = false;

    while (local_timeout > 0) {
        sendFrameWAnswer(0x601, 8, N5_SDO_UP_REQ, index, subindex, 0x00, frame_rx.array);
        delay(1);

        local_timeout = local_timeout - 1;

        if ((frame_rx.b.p.payload_32t & bits) == bits) {
            local_timeout = 0;
            ret_status = true;
        }
    }

    return(ret_status);
}

void N5CANOpen :: preopNode() {
    /* Go in operational state to enable PDO - Used for obj 6073:00*/
    switchState(PREOP_STATE, 0x00);
    delay(1000);
}

void N5CANOpen :: closeLoop() {
    N5CANOpen::t_N5_Frame  frame_rx;

    switchState(PREOP_STATE, 0x00);
    delay(1000);

    sendFrameWAnswer(0x601, 8, N5_SDO_DOWN_CMD_4B, 0x3202, 0x00, 0x41, frame_rx.array);
    printCANData(frame_rx);
}