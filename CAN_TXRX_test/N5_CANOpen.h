#ifndef _N5_CANOPEN_H_
#define _N5_CANOPEN_H_

#include <stdint.h>
#include "arduino.h"
#include "MCP2515.h"

#define N5_ADDRESS                  1

#define N5_SDO_FRAME_ID             600

/* ==================================== */
#define N5_SDO_DOWN_CMD_1B          0x2F
#define N5_SDO_DOWN_CMD_2B          0x2B
#define N5_SDO_DOWN_CMD_3B          0x27
#define N5_SDO_DOWN_CMD_4B          0x23

#define N5_SDO_DOWN_OK              0x60


#define N5_SDO_UP_REQ               0x40

#define N5_SDO_UP_CMD_1B            0x4F
#define N5_SDO_UP_CMD_2B            0x4B
#define N5_SDO_UP_CMD_3B            0x47
#define N5_SDO_UP_CMD_4B            0x43


#define N5_SDO_UD_ERR               0x80

#define OPERATIONAL_STATE           0x01
#define STOP_STATE                  0x02
#define PREOP_STATE                 0x80
#define NODE_RESET_STATE            0x81
#define COMM_RESET_STATE            0x82

#define READY_TO_ON                 0x01
#define CURRENTLY_ON                0x02
#define READY_TO_OPERATE            0x03
#define TARGET_REACHED              0x04



class N5CANOpen {

    private:

        MCP2515 mcp2515;
    
        typedef union {
            struct {
                uint8_t     command;

                union {
                    uint8_t     index_arr[2];
                    uint16_t    index;
                } i;

                uint8_t     subindex;
                
                union {
                    uint8_t     payload[4];
                    uint32_t    payload_32t;

                    struct {
                        uint8_t size;
                        uint8_t subindex;
                        uint16_t index;
                    }rx_pdo;

                } p;
            } b;
            uint8_t         array[8];
        } t_N5_Frame;

        typedef union {
            struct {
                uint16_t    r_6040;
                uint8_t     r_6060;
                uint16_t    r_6042_6071;
                uint16_t    r_6073;
                uint32_t    r_3202;
            } b;
            uint8_t         array[11];
        } t_N5_RXPDO;

        typedef union {
            struct {
                uint16_t    unused;
                uint16_t    r_6041;
                uint8_t     r_6061;
                uint8_t     empty[3];
            } b;
            uint8_t         array[8];
        } t_N5_TXPDO;

        typedef union {
            union {
                uint8_t     payload[4];
                uint32_t    payload_32t;

                struct {
                    uint8_t size;
                    uint8_t subindex;
                    uint16_t index;
                }rx_pdo;

            } p;
        } t_N5_Payload;

        t_N5_RXPDO  rxpdo;

        bool checkTXAnswer(t_N5_Frame frame_tx, t_N5_Frame frame_rx);
        
        void sendFrameWAnswer(uint16_t ID, uint16_t data_length, uint8_t command, uint16_t index, uint8_t subindex, uint32_t payload, uint8_t *frame_answ);
        void sendFrameWAnswer(uint16_t ID, uint16_t data_length, uint8_t *ptr_data, uint8_t *frame_answ);
        
        void setRXPDO(uint16_t *ptr_register, uint8_t *ptr_subindex, uint8_t *ptr_reg_size, uint8_t size);
        uint8_t loadDownloadSize(uint8_t size);

    public:

        typedef struct {
            uint32_t        pole_pair;
            uint32_t        max_current;
            uint32_t        rated_current;
            uint16_t        max_perth_current;
            uint32_t        max_duration_peak_current;
        } t_Motor_Data;

        typedef struct {
            uint32_t        max_speed;
            uint32_t        max_current;
        } t_Control_Para;

        void begin();
        void PDOMapping_Velocity();
        void PDOMapping_Torque();
        bool setMotorData(t_Motor_Data para);
        bool startAutoCalibration();
        void setControlLoop();
        bool startVelocityProfile(uint16_t speed);
        void stopVelocityProfile();

        bool startTorqueProfile(uint16_t torque);

        void printCANData(t_N5_Frame frame);
        void printCANData(uint8_t *array);

        void setIP(uint32_t IP);
        void switchState(uint8_t state, uint8_t node_ID);

        uint16_t check6041Status(uint16_t status);
        bool checkOBJbits(uint16_t index, uint16_t subindex, uint16_t bits, uint16_t timeout);
};



#endif