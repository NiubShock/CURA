#include "MCP2515.h"

void MCP2515 :: set_cs_pin(int pin){
    cs_pin = pin;
}

uint16_t MCP2515 :: read_register(uint8_t address) {

    uint16_t data;

    if (cs_pin < 0) return 0xFFFF;

    /* Enable the CS */
    digitalWrite(cs_pin, LOW);
    SPI.transfer(MCP_READ);
    SPI.transfer(address);
    data = SPI.transfer(0xFF);
    digitalWrite(cs_pin, HIGH);

    return data;
}

bool MCP2515 :: write_register(uint8_t address, uint8_t data) {

    if (cs_pin < 0) return false;

    /* Enable the CS */
    digitalWrite(cs_pin, LOW);
    SPI.transfer(MCP_WRITE);
    SPI.transfer(address);
    SPI.transfer(data);
    digitalWrite(cs_pin, HIGH);

}

bool MCP2515 :: transfer(t_MCP2515_CAN_Frame frame) {
    uint8_t id_h, id_l;

    id_h = frame.ID >> 3;
    id_l = frame.ID << 5;

    /* Load the ID */
    write_register(MCP_TXB0SIDH, id_h);
    write_register(MCP_TXB0SIDL, id_l);

    /* Load the DLC */
    write_register(MCP_TXB0DLC, frame.data_length);

    /* Load the data */
    for (int i = 0; i < frame.data_length; i++){
        write_register(MCP_TXB0D_START + i, frame.data[i]);
    }

    /* Send the data */
    write_register(MCP_TXB0CTRL, MCP_TXB_TXREQ_M);

    /* Check data tx completed */
    uint8_t data;
    bool    tx_completed = false;

    // for(int i = 0; i < CANSENDTIMEOUT && !tx_completed; i++){

    //     data = read_register(MCP_TXB0CTRL);
    //     if ((data & MCP_TXB_TXREQ_M) == MCP_TXB_TXREQ_M){
    //         tx_completed = true;
    //     }

    //     delay(1);
    // }

    return tx_completed;
}

bool MCP2515 :: checkRXBuffer(uint8_t *ptr_array){
    t_MCP_CANINTIF  CANINTF;
    t_MCP_RXBDLC    RXB0DLC;

    CANINTF.w = 0;
    RXB0DLC.w = 0;

    CANINTF.w = read_register(MCP_CANINTF);
    RXB0DLC.w = read_register(MCP_RXB0DLC);

    if (CANINTF.b.RX0IF > 0){
        // SERIAL_PORT_MONITOR.print(CANINTF.w, HEX);
        // SERIAL_PORT_MONITOR.print(" ");
        // SERIAL_PORT_MONITOR.print(RXB0DLC.w, HEX);

        write_register(MCP_CANINTF, 0);

        // SERIAL_PORT_MONITOR.println(" ");

        for (int i = 0; i < 8; i ++){
            *(ptr_array + i) = read_register(MCP_RXB0D_START + i);
        }
    }
    return((bool)CANINTF.b.RX0IF);
}

bool MCP2515 :: checkRXBuffer(uint8_t *ptr_array, uint16_t delay_ms){
    bool read = false;

    /* wait for the cycle or just the read boolean becomes true */
    for (uint16_t i = 0; i < delay_ms && !read; i++){
        read = MCP2515 :: checkRXBuffer(ptr_array);
        delay(1);
    }

    return read;
}

bool MCP2515 :: begin(t_MCP2515_Init_Param param, t_MCP2515_Mode mode) {

    t_MCP_CANCTRL   CANCTRL;
    t_MCP_CANSTAT   CANSTAT;

    t_MCP_CNF1      CNF1;
    t_MCP_CNF2      CNF2;
    t_MCP_CNF3      CNF3;

    t_MCP_RXB0CTRL  RXB0CTRL;
    t_MCP_RXB1CTRL  RXB1CTRL;

    uint8_t data;

    /* Set the CS pin as output and high to disable the chip */
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);

    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    CANCTRL.w = 0;
    CANSTAT.w = 0;

    CNF1.w = 0;
    CNF2.w = 0;
    CNF3.w = 0;

    CNF1.b.SJW      = param.synch;
    CNF1.b.BRP      = param.prescaler;

    CNF2.b.BTLMOD   = true;
    CNF2.b.SAM      = param.triple_sample_point;
    CNF2.b.PHSEG1   = param.ps1;
    CNF2.b.PHSEG    = param.propagation_delay;

    CNF3.b.PHSEG2   = param.ps2;

    /* ----------------------------------------------------- */
    /* Enter in configuration mode and check the device mode */
    /* ----------------------------------------------------- */
    CANCTRL.b.REQOP = 0x04;
    write_register(MCP_CANCTRL, CANCTRL.w);

    bool set_ok = false;
    for (int i = 0; i < 100 && !set_ok; i++){
        CANSTAT.w = read_register(MCP_CANSTAT);
        
        if (CANSTAT.b.OPMOD == MODE_CONFIG) {
            set_ok = true;
        }

        delay(1);
    }

    if (!set_ok)    return false;
    /* ----------------------------------------------------- */

    RXB0CTRL.w = 0;
    RXB1CTRL.w = 0;

    RXB0CTRL.b.BUKT = param.rollover_enable;
    RXB0CTRL.b.RXM  = param.receive_buff_option;
    RXB1CTRL.b.RXM  = param.receive_buff_option;

    write_register(MCP_RXB0CTRL, RXB0CTRL.w);
    // write_register(MCP_RXB1CTRL, RXB1CTRL.w);

    t_MCP_RXB0CTRL  RXB0CTRL_READ;
    RXB0CTRL_READ.w = read_register(MCP_RXB0CTRL);
    // if (RXB0CTRL_READ.b.RXM != RXB0CTRL.b.RXM) return false;
    SERIAL_PORT_MONITOR.println(RXB0CTRL_READ.w, HEX);

    t_MCP_RXB1CTRL  RXB1CTRL_READ;
    RXB1CTRL_READ.w = read_register(MCP_RXB1CTRL);
    // if (RXB1CTRL_READ.b.RXM != RXB1CTRL.b.RXM) return false;
    SERIAL_PORT_MONITOR.println(RXB1CTRL_READ.w, HEX);

    write_register(MCP_CNF1, CNF1.w);
    write_register(MCP_CNF2, CNF2.w);
    write_register(MCP_CNF3, CNF3.w);

    // SERIAL_PORT_MONITOR.println(CNF1.w, HEX);
    // SERIAL_PORT_MONITOR.println(CNF2.w, HEX);
    // SERIAL_PORT_MONITOR.println(CNF3.w, HEX);

    // write_register(MCP_CNF1, 0x00);
    // write_register(MCP_CNF2, 0xD0);
    // write_register(MCP_CNF3, 0x82);

    data = read_register(MCP_CNF1);
    if (data != CNF1.w) return false;

    data = read_register(MCP_CNF2);
    if (data != CNF2.w) return false;

    data = read_register(MCP_CNF3);
    if (data != CNF3.w) return false;

    CANCTRL.w           = 0;
    CANCTRL.b.CLKEN     = mode.clockout_enable;
    CANCTRL.b.OSM       = mode.oneshot_mode;
    CANCTRL.b.CLKPRE    = mode.prescaler_clkout;
    CANCTRL.b.REQOP     = mode.operating_mode;

    write_register(MCP_CANCTRL, CANCTRL.w);
    data = read_register(MCP_CANCTRL);
    if (data != CANCTRL.w) return false;

    return true;
}

void MCP2515 :: end() {
    SPI.endTransaction();
}