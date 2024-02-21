#include "N5_CANOpen.h"

void N5CANOpen :: loadDictionary(uint8_t cmd, uint16_t index, uint8_t subindex, uint8_t ptr_payload[], uint8_t size, t_N5_Frame * frame){
    frame ->b.command     = cmd;
    frame ->b.subindex    = subindex;

    frame ->b.index.index = index;

    for (int i = 0; i < 4; i++){
        frame ->b.payload[i] = 0;
    }

    for (int i = 0; i < size; i++){
        frame ->b.payload[4 - i] = ptr_payload[i];
    }
}