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
    // joystick.init();                      /**< Initialize joystick. */
    ros.init();
    robot_loop_tick = HAL_GetTick(); /**< Store current tick into robot loop counter. */
}

void Robot::run()
{
    if ((HAL_GetTick() - robot_loop_tick) > ROBOT_LOOP_TIME)
    {
        HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin); /**< Toggle Blue LED to represent robot is running or not. */

#ifdef __DEBUG_MODE__
        // joystick.ShowData();
        // joystick.ShowPacket();
#endif
        // joystick.GetData(joystickData);             /**< Get Joystick data for Robot control, */
        // set_state_from_joystick_data(joystickData); /**< Set the state of Robot based on joystick data. */

        // deadMotor.maintainCoordinates();
        // printf("vxp:: %lf  ", deadMotor.x_pid.Output);
        // printf("setpoint:: %.2f  %.2f  %.2f    ", round2(deadMotor.odom_setpoint.x * 100.0f), round2(deadMotor.odom_setpoint.y * 100.0f), round2(deadMotor.odom_setpoint.theta * 180.0f / PI));
        // printf("odom:: %.2f  %.2f  %.2f\n", round2(deadMotor.odom.x * 100.0f), round2(deadMotor.odom.y * 100.0f), round2(deadMotor.odom.theta * 180.0f / PI));

        // ros.display();
        // printf("recv_twist:: %f, %f %f\n", recv_twist.vx, recv_twist.vy, recv_twist.w);

        if (HAL_GetTick() - ros.last_updated_tick > 500)
            deadMotor.base_twist = Twist(0, 0, 0);
        else
            set_base_twist_from_recv_twist();

        deadMotor.run(); /**< Move robot base according to control data .*/

        robot_loop_tick = HAL_GetTick(); /**< Store robot loop clock tick. */
    }
}

// /**
//  * @brief Sets robot state based on joystick data.
//  *
//  * @param joytick_data Joystick data for receive.
//  */
// void Robot::set_state_from_joystick_data(const JoystickData &jdata)
// {

//     if ((HAL_GetTick() - joystick.last_updated_tick) > 100)
//     {
// #ifdef __DEBUG_MODE__
//         printf("delayed data\n");
// #endif
//         /**< Stop robot if there is data delay for a second. */
//         deadMotor.base_twist = Twist(0, 0, 0);
//         return;
//     }

//     // const uint8_t d_band = 50;
//     // float v = 0, theta = deadMotor.base_twist.get_theta(), omega = 0;

//     // /**< Donot move robot base if joytick data is less than band. */
//     // if (abs(jdata.l_hatx) < d_band && abs(jdata.l_haty) < d_band &&
//     //     jdata.lt < d_band && jdata.rt < d_band)
//     // {
//     //     deadMotor.base_twist = Twist(0, theta, 0);
//     // }

//     // /**< Move robot base based on left joystick value. */
//     // if (((abs(jdata.l_haty) > d_band) || (abs(jdata.l_hatx) > d_band)))
//     // {
//     //     float magnitude = sqrt(pow(jdata.l_haty, 2) + pow(jdata.l_hatx, 2));
//     //     v = map<float>(magnitude, 0, 128, 0, MAXIMUM_VELOCITY);
//     //     theta = atan2(jdata.l_haty, jdata.l_hatx);
//     // }

//     // /**< Slow down rotation if LB is pressed. */
//     // float speed_factor = 1.0;
//     // if (jdata.button1 & _BV(B_LB))
//     // {
//     //     speed_factor = 0.2;
//     // }

//     // /**< Rotate robot based on LT and RT value. */
//     // if (jdata.lt > 30)
//     // {
//     //     omega = map<float>(jdata.lt, 0, 255, 0, MAXIMUM_OMEGA * speed_factor);
//     // }
//     // else if (jdata.rt > 30)
//     // {
//     //     omega = -map<float>(jdata.rt, 0, 255, 0, MAXIMUM_OMEGA * speed_factor);
//     // }

//     // if ((HAL_GetTick() - last_button_tick) > 100)
//     // {
//     //     if ((jdata.button1 & _BV(B_UP)) && (!(prev_button1 & _BV(B_UP))))
//     //     {
//     //         deadMotor.odom_setpoint.y += 1.0;
//     //         // printf("Up::");
//     //         // printf("UP\n");
//     //     }
//     //     else if ((jdata.button1 & _BV(B_DOWN)) && (!(prev_button1 & _BV(B_DOWN))))
//     //     {
//     //         deadMotor.odom_setpoint.y -= 1.0;

//     //         // printf("Down::");
//     //     }
//     //     else if ((jdata.button2 & _BV(B_LEFT)) && (!(prev_button2 & _BV(B_LEFT))))
//     //     {
//     //         deadMotor.odom_setpoint.x -= 1.0;
//     //         // printf("Left::");
//     //     }
//     //     else if ((jdata.button2 & _BV(B_RIGHT)) && (!(prev_button2 & _BV(B_RIGHT))))
//     //     {
//     //         deadMotor.odom_setpoint.x += 1.0;
//     //         // printf("Right::");
//     //     }
//     //     else if ((jdata.button1 & _BV(B_X)) && (!(prev_button1 & _BV(B_X))))
//     //     {
//     //         deadMotor.changeTheta(deadMotor.odom_setpoint.theta + M_PI_2);
//     //     }
//     //     else if ((jdata.button1 & _BV(B_B)) && (!(prev_button1 & _BV(B_B))))
//     //     {
//     //         deadMotor.changeTheta(deadMotor.odom_setpoint.theta - M_PI_2);
//     //     }

//     //     prev_button1 = jdata.button1;
//     //     prev_button2 = jdata.button2;
//     //     last_button_tick = HAL_GetTick();
//     // }

//     /**< Stop and Abort if XBOX or PS4 button is pressed. */
//     if (jdata.button2 & _BV(B_XBOX))
//     {
//         EMERGENCY_BREAK();
//     }

//     /**< Finally apply the states. */
//     // deadMotor.base_twist = Twist::from_v_theta_omega(v, theta, omega);
//     // printf("v th w: %f %f %f\n", v, theta, omega);
// }

void Robot::set_base_twist_from_recv_twist()
{
    deadMotor.base_twist = recv_twist;
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
