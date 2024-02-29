// #include "N5_CANOpen.h"
// #include "MCP2515.h"

// static MCP2515 mcp2515;

// void N5CANOpen :: sendFrameWAnswer(uint16_t ID, uint16_t data_length, uint8_t command, uint16_t index, uint8_t subindex, uint32_t payload, t_N5_Frame *frame_answ){
//     t_N5_Frame frame;
//     MCP2515::t_MCP2515_CAN_Frame frame_mcp;
//     MCP2515::t_MCP2515_CAN_Frame frame_mcp_rx;

//     /* Load the data in the frame */
//     // frame.b.ID              = ID;
//     // frame.b.data_length     = data_length;

//     // frame.b.command         = command;
//     // frame.b.i.index         = index;
//     // frame.b.subindex        = subindex;
//     // frame.b.p.payload_32t   = payload;

//     /* Load the data in the MCP frame */
//     // frame_mcp.ID            = ID; //frame.b.ID;
//     // frame_mcp.data_length   = data_length; //.b.data_length;
//     // frame_mcp.data          = frame.array;

//     frame_mcp.ID            = 0x601;
//     frame_mcp.data_length   = 8;

//     /* Load polepairs in obj 0x2030:00 */
//     frame.b.command         = N5_SDO_DOWN_CMD_4B;
//     frame.b.i.index         = 0x2030;
//     frame.b.subindex        = 0x00;
//     frame.b.p.payload_32t   = payload;

//     // frame_mcp.ID            = frame.b.ID;
//     // frame_mcp.data_length   = frame.b.data_length;
//     frame_mcp.data          = frame.array;

//     /* Send the data and wait for the answer */
//     mcp2515.transfer(frame_mcp);
//     mcp2515.checkRXBuffer(frame_answ ->array, 100);

//     // for (int i = 0; i < 8; i++) frame_answ ->array[i] = *(frame_mcp_rx.data + i);
// }

// bool N5CANOpen :: checkTXAnswer(t_N5_Frame frame_tx, t_N5_Frame frame_rx) {
//     if (frame_rx.b.command != N5_SDO_DOWN_OK) {
//         SERIAL_PORT_MONITOR.println("Error CMD");
//         return false;
//     }

//     if (frame_rx.b.subindex != frame_tx.b.subindex) {
//         SERIAL_PORT_MONITOR.println("Error Subindex");
//         return false;
//     }

//     if (frame_rx.b.i.index != frame_tx.b.i.index){
//         SERIAL_PORT_MONITOR.print(frame_rx.b.i.index);
//         SERIAL_PORT_MONITOR.print(" != ");
//         SERIAL_PORT_MONITOR.println(frame_tx.b.i.index);
//         return false;
//     }

//     return true;
// }

// uint8_t N5CANOpen :: loadDownloadSize(uint8_t size) {
//     if (size == 1) return N5_SDO_DOWN_CMD_1B;
//     if (size == 2) return N5_SDO_DOWN_CMD_2B;
//     if (size == 3) return N5_SDO_DOWN_CMD_3B;
//     if (size == 4) return N5_SDO_DOWN_CMD_4B;

//     return 0;
// }

// void N5CANOpen :: setRXPDO(uint16_t *ptr_register, uint8_t *ptr_subindex, uint8_t *ptr_reg_size, uint8_t size) {
    
//     t_N5_Frame frame;
//     t_N5_Frame frame_rx;
//     MCP2515::t_MCP2515_CAN_Frame frame_mcp;

//     frame_mcp.ID            = 0x601;
//     frame_mcp.data_length   = 8;

//     /* Enable the modification */
//     frame.b.command         = N5_SDO_DOWN_CMD_4B;
//     frame.b.i.index         = 0x1400;
//     frame.b.subindex        = 0x01;
//     frame.b.p.payload_32t   = 0x80000201;

//     frame_mcp.data = frame.array;
//     mcp2515.transfer(frame_mcp);
//     mcp2515.checkRXBuffer(frame_rx.array, 100);

//     /* Disable the profile */
//     frame.b.command         = N5_SDO_DOWN_CMD_1B;
//     frame.b.i.index         = 0x1600;
//     frame.b.subindex        = 0x00;
//     frame.b.p.payload_32t   = 0x00;

//     frame_mcp.data = frame.array;
//     mcp2515.transfer(frame_mcp);
//     mcp2515.checkRXBuffer(frame_rx.array, 100);

//     /* Load the objects */
//     for (int i = 0; i < size; i++) {
//         frame.b.command             = loadDownloadSize(4);
//         frame.b.i.index             = 0x1600;
//         frame.b.subindex            = 1 + i;

//         frame.b.p.rx_pdo.size       = *(ptr_reg_size + i);
//         frame.b.p.rx_pdo.subindex   = *(ptr_subindex + i);
//         frame.b.p.rx_pdo.index      = *(ptr_register + i);

//         frame_mcp.data = frame.array;
//         mcp2515.transfer(frame_mcp);
//         mcp2515.checkRXBuffer(frame_rx.array, 100);
//     }

//     /* Load max duration in obj 0x1600:00 */
//     frame.b.command         = N5_SDO_DOWN_CMD_1B;
//     frame.b.i.index         = 0x1600;
//     frame.b.subindex        = 0x00;
//     frame.b.p.payload_32t   = size;

//     frame_mcp.data = frame.array;
//     mcp2515.transfer(frame_mcp);
//     mcp2515.checkRXBuffer(frame_rx.array, 100);
// }

// void N5CANOpen :: defPDOMapping() {
//     uint16_t add_reg[]      = {0x6040, 0x6060, 0x6042};
//     uint8_t reg_subind[]    = {0x00, 0x00, 0x00};
//     uint8_t reg_size[]      = {0x10, 0x8, 0x10};

//     setRXPDO(add_reg, reg_subind, reg_size, 3);
// }