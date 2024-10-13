
#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
#include "main_task.hpp"
#include "math.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "HW_can.hpp"
#include "string.h"

uint32_t tick = 0;
uint8_t tx_byte = 0;
uint8_t rx_byte = 0;
uint8_t can_tx_data[8] = {0};

struct UartCommData {
    uint32_t tick;
    float value;
};
struct UartCommData uart_data_tx;
struct UartCommData uart_data_rx;


struct CANCommData can_data_tx;
struct CANCommData can_data_rx;

// 编码函数，将结构体数据编码为字节数组
void encode(uint8_t *data, struct UartCommData *commData) {
    data[0] = 0xAA;
    data[1] = 0xBB;
    data[2] = 0xCC;
    data[3] = (commData->tick >> 24) & 0xFF;
    data[4] = (commData->tick >> 16) & 0xFF;
    data[5] = (commData->tick >> 8) & 0xFF;
    data[6] = commData->tick & 0xFF;
    int16_t value_scaled = (int16_t)(commData->value * 30000.0f);
    data[7] = (value_scaled >> 8) & 0xFF;
    data[8] = value_scaled & 0xFF;
}

// 解码函数，将字节数组解码为结构体数据
void decode(struct UartCommData *commData, uint8_t *data) {
    commData->tick = ((uint32_t)data[3] << 24) | ((uint32_t)data[4] << 16) | ((uint32_t)data[5] << 8) | data[6];
    int16_t value_scaled = ((int16_t)data[7] << 8) | data[8];
    commData->value = value_scaled / 30000.0f;
}

void CANEncode(uint8_t *data, struct CANCommData *commData) {
    data[0] = (commData->tick >> 24) & 0xFF;
    data[1] = (commData->tick >> 16) & 0xFF;
    data[2] = (commData->tick >> 8) & 0xFF;
    data[3] = commData->tick & 0xFF;

    int16_t value1_scaled = (int16_t)(commData->value1 * 30000.0f);
    data[4] = (value1_scaled >> 8) & 0xFF;
    data[5] = value1_scaled & 0xFF;

    data[6] = commData->value2;

    data[7] = (commData->flag1 << 0) | (commData->flag2 << 1) | (commData->flag3 << 2) | (commData->flag4 << 3);
}

void CANDecode(struct CANCommData *commData, uint8_t *data) {
    commData->tick = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];

    int16_t value1_scaled = ((int16_t)data[4] << 8) | data[5];
    commData->value1 = value1_scaled / 30000.0f;

    commData->value2 = data[6];

    commData->flag1 = data[7] & 0x01;
    commData->flag2 = (data[7] >> 1) & 0x01;
    commData->flag3 = (data[7] >> 2) & 0x01;
    commData->flag4 = (data[7] >> 3) & 0x01;
}



void RobotInit(void) {
    tick = 0;
}

void MainInit(void) {
    tick = 0;

    CanFilter_Init();
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_UART_Receive_IT(&huart1, &rx_byte, 1); // 启动UART1接收
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1); // 启动UART2接收

    HAL_TIM_Base_Start_IT(&htim3); // 启动定时器3
}

GPIO_PinState pin_state = GPIO_PIN_RESET;

uint8_t tx_index = 0;
uint8_t tx_buffer[9];
uint8_t value2init = 0x55;

void MainTask(void) {
    tick++;

    if(tick % 1000 == 0){
        pin_state = (pin_state == GPIO_PIN_SET)? GPIO_PIN_RESET : GPIO_PIN_SET;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, pin_state);
    }
    if(tick % 100 == 0){
        uart_data_tx = {tick, sin(tick/1000.0f)};
        encode(tx_buffer, &uart_data_tx); // 编码数据
        // 发送数据
        for (tx_index = 0; tx_index < 9; ++tx_index) {
            HAL_UART_Transmit(&huart1, &tx_buffer[tx_index], 1, 0xffff);
        }
    }
    if (tick % 100 == 10) {
        can_data_tx.tick = tick;
        can_data_tx.value1 = sin(tick / 1000.0f);
        value2init += 1;
        if (value2init >= 0xff) {
            value2init = 0x01;
        }
        can_data_tx.value2 = value2init;
        can_data_tx.flag1 = true;
        can_data_tx.flag2 = false;
        can_data_tx.flag3 = true;
        can_data_tx.flag4 = false;

        uint8_t can_tx_data[8];
        CANEncode(can_tx_data, &can_data_tx);

        // 发送数据，帧ID为0x100
        CAN_Send_Msg(&hcan, can_tx_data, 0x100, 8);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim3) {
        MainTask();
    }
}

uint8_t uart_rx_buffer[9];
uint8_t rx_index = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        // 将接收到的字节存储到缓冲区
        uart_rx_buffer[rx_index++] = rx_byte;

        if (rx_index == 9) {

            decode(&uart_data_rx, uart_rx_buffer);

            rx_index = 0;
        }

       HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}
