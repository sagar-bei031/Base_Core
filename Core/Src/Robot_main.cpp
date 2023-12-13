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

    if (huart->Instance == robot.joystick.huart->Instance)
    {
        static uint32_t prevTick = 0;
        uint32_t curTick = HAL_GetTick();

        /* Toggle LED to indicate data recive */
        if ((curTick - prevTick) > 50)
        {
            HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
            prevTick = curTick;
        }

        robot.joystick.get_received_data((uint8_t *)(&robot.joystickData));
    }
    else if (huart->Instance == robot.deadMotor.deadWheel.huart->Instance)
    {
        if (robot.deadMotor.deadWheel.get_received_data(robot.deadMotor.odom_rx_data) == OK)
        {
#ifndef __COUNT__
            memcpy(&robot.deadMotor.odom, robot.deadMotor.odom_rx_data, 12);
            // printf("odom:: %f %f %f\n", robot.deadMotor.odom.x * 100.0f, robot.deadMotor.odom.y * 100.0f, robot.deadMotor.odom.theta * 180.0f / M_PI);
#endif

#ifdef __COUNT__
            memcpy(odata, robot.deadMotor.odom_rx_data, 12);
            printf("odom:: %ld %ld %ld\n", odata[0], odata[1], odata[2]);
#endif
        }
    }
}