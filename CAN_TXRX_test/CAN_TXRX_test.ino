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

    SERIAL_PORT_MONITOR.begin(115200);
    while(!Serial){};

    N5.begin();

    N5CANOpen :: t_Motor_Data para;
    para.pole_pair                      = 3;
    para.max_current                    = 18000;
    para.max_duration_peak_current      = 1000;
    para.rated_current                  = 12000;
    para.max_perth_current              = 100;


    N5.setMotorData(para);
    /* First turnon */
    N5.startAutoCalibration();


    N5.preopNode();
    N5.closeLoop();
    N5.startVelocityProfile(0x14);

}

void loop() {

    static uint8_t counter = 0;

    static N5CANOpen :: t_Data_Read data_read;

    // if (counter < 10) {
        // N5.setSpeedSetpoint(0x28);
    // }
    // else if (counter < 20){
    //     N5.setSpeedSetpoint(0x28);
    // } else {
    //     counter = 0;
    // }

    // delay(1000);

    delay(10);
    counter++;
    N5.readData(&data_read);

    if(counter >= 10){
        counter = 0;

        SERIAL_PORT_MONITOR.print("Speed ");
        SERIAL_PORT_MONITOR.print(data_read.speed);
        SERIAL_PORT_MONITOR.print(" Torque ");
        SERIAL_PORT_MONITOR.print(data_read.torque);
        SERIAL_PORT_MONITOR.print(" Position ");
        SERIAL_PORT_MONITOR.println(data_read.position);
    }

    // counter++;
    // SERIAL_PORT_MONITOR.print("Speed ");
    // SERIAL_PORT_MONITOR.print(N5.readSpeed());
    // SERIAL_PORT_MONITOR.print(" Torque ");
    // SERIAL_PORT_MONITOR.print(N5.readTorque());
    // SERIAL_PORT_MONITOR.print(" Position ");
    // SERIAL_PORT_MONITOR.println(N5.readPosition());
}

// END FILE
