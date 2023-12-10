/**
 ******************************************************************************
 * @file    Robot.hpp
 * @brief   Header file for Robot Class
 * @author  Robotics Team 2024, IOE Pulchowk Campus
 * @date    2023
 ******************************************************************************
 */

#ifndef ROBOT_HPP__
#define ROBOT_HPP__

#include "stm32f4xx_hal.h"
#include "definition.h"
#include "Robot_config.h"
#include "Robotlib/actuators/DeadMotor.hpp"
#include "Robotlib/communication/uart.h"
#include "Robotlib/communication/uart_hs.hpp"

#define ROBOT_LOOP_TIME 10

/**
 * @class Robot
 * @brief Manages robot functionalities.
 *
 * This class encapsulates functions for initializing and controlling the robot.
 * It includes components such as DeadMotor and Uart for communication.
 */
class Robot
{
public:
    /**
     * @brief Default constructor for Robot class.
     */
    Robot() = default;

    /**
     * @brief Default destructor for Robot class.
     */
    ~Robot() = default;

    /**
     * @brief Initializes the robot system by calling init function of every object used.
     */
    void init();

    /**
     * @brief Runs the main control loop for the robot.
     */
    void run();

    /* Components used by the Robot */
    DeadMotor deadMotor;             /**< Object for DeadMotor component. */
    UartHS joystick{&JOYSTICK_UART}; /**< Object for UartHS component for joystick communication. */
    uint32_t robot_loop_tick = 0;    /**< Robot main loop counter. */

private:
    /**
     * @brief Sets robot state based on joystick data.
     *
     * @param joytick_data Joystick data for receive.
     */
    void set_state_from_joystick_data(const JoystickData &joytick_data);

/**
 * @brief Handles emergency braking.
 * 
 * Stop robot and abort operation in case of emergency.
 */
    void EMERGENCY_BREAK();

    JoystickData joystickData;     /**< Joystick data received. */
    uint32_t last_button_tick = 0; /**< Last button clicked tick. */
    uint32_t prev_button1 = 0;     /**< Previous state of button 1. */
    uint32_t prev_button2 = 0;     /**< Previous state of button 2. */
};

#endif // __ROBOT