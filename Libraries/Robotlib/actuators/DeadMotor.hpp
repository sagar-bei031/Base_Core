#ifndef __DEAD_MOTOR_HPP__
#define __DEAD_MOTOR_HPP__

#include "stm32f4xx_hal.h"
#include "Robot_config.h"
#include "definition.h"
#include "Robotlib/actuators/motor.hpp"
#include "Robotlib/sensors/encoder.hpp"
#include "Robotlib/controllers/pid.hpp"
#include "Robotlib/controllers/fuzzy_pid.hpp"
#include "Robotlib/kinematics/omniwheel.hpp"
#include "Robotlib/communication/uart.hpp"

#define MOTOR_LOOP_TIME 10

#define DEFAULT_VELOCITY MAXIMUM_VELOCITY
#define DEFAULT_OMEGA MAXIMUM_OMEGA

struct Odometry
{
    float x;
    float y;
    float theta;
};

class DeadMotor
{
public:
    DeadMotor() = default;
    ~DeadMotor() = default;

    void init();
    void run();

    Twist base_twist;
    Motor base_motors[4];
    uint32_t motor_loop = 0;

    OmniwheelKinematics omniwheel_kinematics{BASE_DIAMETER, WHEEL_DIAMETER};
    PID base_motor_pid_controllers[4];
    Encoder base_motor_encoders[4];

    float motor_omegas[4] = {0, 0, 0, 0};
    float kp[4] = {0.7f, 0.5f, 0.5f, 0.7f};
    float ki[4] = {15.0f, 15.0f, 15.0f, 15.0f};
    float kd[4] = {0.0001f, 0.0001f, 0.0001f, 0.0001f};
};

#endif