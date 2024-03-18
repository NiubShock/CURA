#include <SPI.h>
#include "N5_CANOpen.h"
#include "MCP2515_Registers.h"
#include "MCP2515.h"

#define CAN_2515

const int SPI_CS_PIN  = 9;

N5CANOpen N5;
MCP2515 mcp2515;
uint8_t data;

void setup() {

    // mcp2515.set_cs_pin(SPI_CS_PIN);

    SERIAL_PORT_MONITOR.begin(115200);
    while(!Serial){};

    N5.begin();

    N5CANOpen :: t_Motor_Data para;

    para.pole_pair                      = 3;
    para.max_current                    = 18000;
    para.max_duration_peak_current      = 1000;
    para.rated_current                  = 12000;
    para.max_perth_current              = 100;

  
    SERIAL_PORT_MONITOR.println(N5.setMotorData(para));

    N5.startAutoCalibration();
    N5.startVelocityProfile();

}

void loop() {

}

// END FILE
