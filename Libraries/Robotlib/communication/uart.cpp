#include "uart.hpp"
#include "Robotlib/maths/math.hpp"
#include <stdio.h>

UART::UART(UART_HandleTypeDef *_huart, uint8_t size, UARTMode _mode = RECEIVING)
    : huart(_huart), last_updated_tick(0), mode(_mode)
{
    if (mode == RECEIVING)
    {
        r_size = size;
        receive_state = WAITING_FOR_START_BYTE;
        status = DISCONNECTED;
    }
    else if (mode == TRANSMITTING)
    {
        t_size = size;
    }
}

UART::UART(UART_HandleTypeDef *_huart, uint8_t _r_size, uint8_t _t_size, UARTMode = BOTH)
    : huart(_huart), r_size(_r_size), t_size(_t_size), mode(BOTH)
{
    receive_state = WAITING_FOR_START_BYTE;
    status = DISCONNECTED;
}

void UART::init()
{
    if (mode == RECEIVING || mode == BOTH)
    {
        HAL_UART_Receive_DMA(huart, &first_byte, 1);
    }

    last_updated_tick = HAL_GetTick();
}

UARTStatus UART::receive()
{
    status = HASH_DIDNT_MATCH;
    if (receive_state == WAITING_FOR_START_BYTE)
    {
        if (first_byte == UART_START_BYTE)
        {
            HAL_UART_Receive_DMA(huart, &len, 1);
            receive_state = WAITING_FOR_LEN;
            first_byte = 0x00;
        }
        else
        {
            HAL_UART_Receive_DMA(huart, &first_byte, 1);
        }

        return status;
    }

    if (receive_state == WAITING_FOR_LEN)
    {
        if (len > r_size)
        {
            HAL_UART_Receive_DMA(huart, &first_byte, 1);
            receive_state = WAITING_FOR_START_BYTE;
        }

        HAL_UART_Receive_DMA(huart, receiving_data_dma, len);
        receive_state = RECEIVING_DATA;
        return status;
    }

    if (receive_state == RECEIVING_DATA)
    {
        HAL_UART_Receive_DMA(huart, &rem_byte, 1);
        receive_state = WAITING_FOR_REM;
        return status;
    }

    if (receive_state == WAITING_FOR_REM)
    {
        receive_state = WAITING_FOR_START_BYTE;
#if defined __IMPLEMENT_CRC__
        uint8_t hash = crc.get_Hash(receiving_data_dma, len);
#else
        uint8_t hash = get_checksum(receiving_data_dma, len);
#endif
        HAL_UART_Receive_DMA(huart, &first_byte, 1);
        if (hash == rem_byte)
        {
            id = receiving_data_dma[0];
            status = OK;
            return status;
        }
        else
        {
            printf("HASH_DIDNT_MATCH::");
            printf("expected:%u received:%u\n", hash, rem_byte);

            status = HASH_DIDNT_MATCH;

            static uint32_t prevTick = 0;
            uint32_t curTick = HAL_GetTick();

            /* Toggle LED to indicate data recive */
            if ((curTick - prevTick) > 50)
            {
                HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
                prevTick = curTick;
            }
        }
    }
    return status;
}

void UART::transmit(uint8_t *t_data)
{
    transmission_data[0] = UART_START_BYTE;
    memcpy(&transmission_data[1], t_data, t_size);
#if defined __IMPLEMENT_CRC__
    transmission_data[t_size + 1] = crc.get_Hash(transmission_data + 1, t_size);
#else
    transmission_data[t_size + 1] = get_checksum(transmission_data + 1, t_size);
#endif
    transmission_data[t_size + 2] = '\n';
    HAL_UART_Transmit_DMA(huart, transmission_data, t_size + 3);
}

UARTStatus UART::get_received_data(uint8_t *r_data)
{

    memcpy(r_data, receiving_data_dma + 1, len - 1);
    this->last_updated_tick = HAL_GetTick();
    return OK;
}

#ifdef __IMPLEMENT_CHECKSUM__
uint8_t UART::get_checksum(uint8_t *data, uint8_t size)
{
    uint8_t hash = data[0];
    for (int i = 1; i < size; ++i)
    {
        hash ^= data[i]; // XOR based checksum
    }
    return hash;
}
#endif

void UART::display()
{
    for (int i = 0; i < r_size; i++)
    {
        printf("%d\t  ", receiving_data_dma[i]);
    }
    // printf("\n");
}
