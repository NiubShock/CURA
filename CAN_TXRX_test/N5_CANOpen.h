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


class N5CANOpen {
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
        bool setMotorData(t_Motor_Data para);
        void startAutoCalibration();
        void setControlLoop();
        void startPositionProfile();
        

    private:
    
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

        bool checkTXAnswer(t_N5_Frame frame_tx, t_N5_Frame frame_rx);
        void sendFrameWAnswer(uint16_t ID, uint16_t data_length, uint8_t command, uint16_t index, uint8_t subindex, uint32_t payload, t_N5_Frame *frame_answ);
        void setRXPDO(uint16_t *ptr_register, uint8_t *ptr_subindex, uint8_t *ptr_reg_size, uint8_t size);
        uint8_t loadDownloadSize(uint8_t size);
        void defPDOMapping();
};



#endif