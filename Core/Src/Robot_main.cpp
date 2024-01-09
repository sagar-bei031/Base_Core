/**
 ******************************************************************************
 * @file    Robot_main.c
 * @brief   Implementation for Robot.h
 * @author  Robotics Team 2024, IOE Pulchowk Campus
 * @date    2023
 ******************************************************************************
 */

#include "Robot_main.h"
#include "Robot.hpp"
#include "usart.h"
#include <memory.h>

// #define __COUNT__

/* Create main implementing object for Robot */
Robot robot;

/**
 * @brief Main function for Robot to be linked in main.c file.
 *
 */
void Robot_main()
{
    /* Initialize every object and parameter of the robot system. */
    robot.init();

    /* Infinitely operate robot for gaming */
    while (1)
    {
        /* Run robot continuosly */
        robot.run();

        if (HAL_GetTick() - robot.ros.last_updated_tick > 500)
        {
            robot.ros.init();
            robot.ros.last_updated_tick = HAL_GetTick();
        }
    }
}

#ifdef __COUNT__
int32_t odata[3];
#endif

/* Handle UART Callbacks */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Flush the UART Data register */
    __HAL_UART_FLUSH_DRREGISTER(huart);

    if (huart->Instance == robot.ros.huart->Instance)
    {
        static uint32_t prevTick = 0;
        uint32_t curTick = HAL_GetTick();

        /* Toggle LED to indicate data receive */
        if ((curTick - prevTick) > 50)
        {
            HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
            prevTick = curTick;
        }

        robot.ros.get_received_data((uint8_t *)(&robot.recv_twist));
    }
}