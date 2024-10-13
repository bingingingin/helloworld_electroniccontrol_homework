#ifndef _HW_CAN_H_
#define _HW_CAN_H_
#ifdef __cplusplus
extern "C" {
#endif
/* ------------------------------ Include ------------------------------ */
#include "stm32f103xb.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "can.h"
/* ------------------------------ Macro Definition ------------------------------ */



/* ------------------------------ Type Definition ------------------------------ */



/* ------------------------------ Extern Global Variable ------------------------------ */



/* ------------------------------ Function Declaration (used in other .c files) ------------------------------ */

struct CANCommData
{
    uint32_t tick;
    float value1;
    uint8_t value2;
    bool flag1;
    bool flag2;
    bool flag3;
    bool flag4;
};
void CanFilter_Init();

void CAN_Send_Msg(CAN_HandleTypeDef* hcan, uint8_t* msg, uint32_t id, uint8_t len);

void CANEncode(uint8_t *data, struct CANCommData *commData);
void CANDecode(struct CANCommData *commData, uint8_t *data);



#ifdef __cplusplus
}
#endif
#endif
