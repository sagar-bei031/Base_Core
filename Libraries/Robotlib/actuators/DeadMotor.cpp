#include "DeadMotor.hpp"
#include <stdio.h>
#include "Robotlib/maths/math.hpp"

const float desired_max_motor_omega = 50.0f; // (rad/s) Lies on linear region of all motors
const float pwm_for_desired_max_motor_omega[4] = {0.45f, 0.66f, 0.84f, 0.45f};
const float cpr = 999;

void DeadMotor::init()
{
    // 0: M1 -> E3
    // 1: M3 -> E2
    // 2: M4 -> E1
    // 3: M2 -> E5

    base_motors[0] = Motor(&M1P_TIMER, M1D_GPIO_Port, M1P_TIMER_CHANNEL, M1D_Pin);
    base_motors[1] = Motor(&M3P_TIMER, M3D_GPIO_Port, M3P_TIMER_CHANNEL, M3D_Pin);
    base_motors[2] = Motor(&M4P_TIMER, M4D_GPIO_Port, M4P_TIMER_CHANNEL, M4D_Pin);
    base_motors[3] = Motor(&M2P_TIMER, M2D_GPIO_Port, M2P_TIMER_CHANNEL, M2D_Pin);

    base_motor_encoders[0] = Encoder(&ENC3_TIMER, cpr);
    base_motor_encoders[1] = Encoder(&ENC2_TIMER, cpr);
    base_motor_encoders[2] = Encoder(&ENC1_TIMER, cpr);
    base_motor_encoders[3] = Encoder(&ENC5_TIMER, cpr);

    for (int i = 0; i < 4; i++)
    {
        base_motors[i].init();
        base_motor_encoders[i].init();
        base_motor_pid_controllers[i] = PID(1.0, 0.0, 0.0, P_ON_E, DIRECT);

        base_motor_pid_controllers[i].Init();
        base_motor_pid_controllers[i].SetOutputLimits(-desired_max_motor_omega, desired_max_motor_omega);
        base_motor_pid_controllers[i].SetTunings(kp[i], ki[i], kd[i]);
        base_motor_pid_controllers[i].SetSampleTime(MOTOR_LOOP_TIME);
        base_motor_pid_controllers[i].SetMode(AUTOMATIC);
    }

    motor_loop = HAL_GetTick();
}

void DeadMotor::run()
{
    if ((HAL_GetTick() - motor_loop) >= MOTOR_LOOP_TIME)
    {
        omniwheel_kinematics.get_motor_omega(base_twist, motor_omegas);

        for (int i = 0; i < 4; ++i)
        {
            base_motor_pid_controllers[i].Input = base_motor_encoders[i].get_omega();

            printf("%f ", motor_omegas[i]);
            // printf("%f ", base_motor_encoders[i].omega);
            // printf("%ld ", base_motor_encoders[i].count_aggregate);

            base_motor_pid_controllers[i].Setpoint = motor_omegas[i];

            if (base_motor_pid_controllers[i].Compute())
            {
                base_motors[i].set_speed(base_motor_pid_controllers[i].Output / desired_max_motor_omega * pwm_for_desired_max_motor_omega[i]);
                //  base_motors[i].set_speed(0.1);
                base_motor_encoders[i].reset_encoder_count();
            }
        }
        printf("\n");

        motor_loop = HAL_GetTick();
    }
}