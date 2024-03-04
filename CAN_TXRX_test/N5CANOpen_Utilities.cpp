#include "N5_CANOpen.h"
#include "MCP2515.h"

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
    mcp2515.transfer(frame_mcp);
    mcp2515.checkRXBuffer(frame_answ, 100);

    // for (int i = 0; i < 8; i++) frame_answ ->array[i] = *(frame_mcp_rx.data + i);
}

void N5CANOpen :: sendFrameWAnswer(uint16_t ID, uint16_t data_length, uint8_t *ptr_data, uint8_t *frame_answ) {
    t_N5_Frame frame;
    MCP2515::t_MCP2515_CAN_Frame frame_mcp;
    MCP2515::t_MCP2515_CAN_Frame frame_mcp_rx;

    frame_mcp.ID            = ID;
    frame_mcp.data_length   = data_length;
    frame_mcp.data          = ptr_data;

    mcp2515.transfer(frame_mcp);
    if (frame_answ != nullptr)  mcp2515.checkRXBuffer(frame_answ, 100);
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

void N5CANOpen :: defPDOMapping() {
    uint16_t add_reg[]      = {0x6040, 0x6060, 0x6042};
    uint8_t reg_subind[]    = {0x00, 0x00, 0x00};
    uint8_t reg_size[]      = {0x10, 0x8, 0x10};

    setRXPDO(add_reg, reg_subind, reg_size, 3);
}

void N5CANOpen :: printCANData(t_N5_Frame frame){
    for (int i = 0; i < 8; i++){
      SERIAL_PORT_MONITOR.print(frame.array[i], HEX);
      SERIAL_PORT_MONITOR.print(" ");
    }
    SERIAL_PORT_MONITOR.println(" ");
}