#ifndef _N5_CANOPEN_H_
#define _N5_CANOPEN_H_

#include <stdint.h>
#include "arduino.h"

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

        typedef union {
            struct {
                uint8_t     command;

                union {
                    uint8_t     index_arr[2];
                    uint16_t    index;
                } index;

                uint8_t     subindex;

                uint8_t     payload[4];
            } b;
            uint8_t         array[8];
        } t_N5_Frame;

        void loadDictionary(uint8_t cmd, uint16_t index, uint8_t subindex, uint8_t ptr_payload[], uint8_t size, t_N5_Frame * frame);
};



#endif