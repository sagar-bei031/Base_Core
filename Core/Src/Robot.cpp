/**
 ******************************************************************************
 * @file    Robot_main.c
 * @brief   Implementation for Robot.hpp
 * @author  Robotics Team 2024, IOE Pulchowk Campus
 * @date    2023
 ******************************************************************************
 */

#include "Robot.hpp"
#include "gpio.h"
#include "usart.h"
#include "dma.h"
#include "tim.h"
#include "Robotlib/maths/math.hpp"
#include <stdio.h>

/**
 * @brief Initializes the robot system by calling init function of every object used.
 */
void Robot::init()
{
    HAL_Delay(100);

    while (!HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin))
    {
        __NOP();
    }

#ifdef __DEBUG_MODE__
    printf("robot init\n");
#endif

    deadMotor.init(); /**< Initialize motor and its components. */
    ros.init();
    robot_loop_tick = HAL_GetTick(); /**< Store current tick into robot loop counter. */
}

void Robot::run()
{
    if ((HAL_GetTick() - robot_loop_tick) > ROBOT_LOOP_TIME)
    {
        // ros.display();
        // printf("recv_twist:: %f, %f %f\n", recv_twist.vx, recv_twist.vy, recv_twist.w);

        if (HAL_GetTick() - ros.last_updated_tick > 500)
            deadMotor.base_twist = Twist(0, 0, 0);
        else
            deadMotor.base_twist = recv_twist;

        deadMotor.run(); /**< Move robot base according to control data .*/

        robot_loop_tick = HAL_GetTick(); /**< Store robot loop clock tick. */
    }
}

/**
 * @brief Handles emergency braking.
 *
 * Stop robot and abort operation in case of emergency.
 */
void Robot::EMERGENCY_BREAK()
{
    while (1)
    {
        for (int i = 0; i < 4; i++)
        {
            deadMotor.base_motors[i].set_speed(0.0);
            HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
        }
    }
}
