

#ifndef __KALMAN_H__
#define __KALMAN_H__

/*
    X = state vector (estimate)
    W = preticted state noise matrix
    U = control/input vector

    P = process covariance matrix (error in estimate)
    Q = process noise covariance matrix

    K = kalman gain
    R = sensor noise covariance matrix (measurement error)

    Y = actual measurement of state
    Z = measurement noise (due to sensor delay or instrumental error)

    A = State transition matrix
    B = Control transiton matrix
    C = measurement transition matrix
    H = Observation/measurement matrix depends on sensor data structure
    I = identity matrix


    X = A * X + B * U + W   
    P = A * P * A_T + Q

    Y = C * Y + Z                   
    K = P * H / (H * P * H_T + R)   [divide means inverse from right side]
    X = X + K * (Y - H * X)         [update with measurement]
    P = (I - K * H) * P
*/

/* For 3 variable with zero noise
Sx = Sx + Vx*t + 0.5*Ax*t^2
Sy = Sy + Vy*t + 0.5*Ay*t^2
Sa = Sa + Va*t + 0.5*Aa*t^2 ; (a) stands for angular

Similarly
Vx = Vx + Ax*t
....

[Sx]   [1 0 0 t 0 0] [Sx]   [0.5*t^2  0        0      ]
[Sy]   [0 1 0 0 t 0] [Sy]   [0        0.5*t^2  0      ] [Ax]
[Sa] = [0 0 1 0 0 t] [Sa] + [0        0        0.5*t^2] [Ay]
[Vx]   [0 0 0 1 0 0] [Vx]   [t        0        0      ] [Aa]
[Vy]   [0 0 0 0 1 0] [Vy]   [0        t        0      ]
[Va]   [0 0 0 0 0 1] [Va]   [0        0        t      ]
*/

/*
    deviation matrix = matrix - [1] * matrix
    covariance matrix = transpose of deviation matrix * deviation matrix
*/

#include "stm32f4xx_hal.h"
#include "Robotlib/maths/matrixf32.h"
#include <stdio.h>

template <uint16_t stateNum, uint16_t controlNum>
struct Kalman_InitStruct
{
    float32_t A[stateNum * stateNum];   // State transition matrix
    float32_t B[stateNum * controlNum]; // Control input transition matrix
    float32_t Q[stateNum * stateNum];   // Process noise covariance matrix
    float32_t H[stateNum * stateNum];   // Measurement matrix
    float32_t R[stateNum * stateNum];   // Measurement noise covariance matrix
    float32_t P[stateNum * stateNum];   // Process covariance matrix
    float32_t x[stateNum * 1];          // state vector
    float32_t u[controlNum * 1];        // control input vector
    float32_t dT;                       // time difference between two calculation
};

template <uint16_t stateNum, uint16_t controlNum>
class Kalman
{
public:
    const matrixf32_t<stateNum, stateNum> A;   // State transition matrix
    const matrixf32_t<stateNum, controlNum> B; // Control input transition matrix
    const matrixf32_t<stateNum, stateNum> Q;   // Process noise covariance matrix
    const matrixf32_t<stateNum, stateNum> H;   // Measurement matrix
    const matrixf32_t<stateNum, stateNum> R;   // Measurement noise covariance matrix
    matrixf32_t<stateNum, stateNum> P;         // Process covariance matrix

    matrixf32_t<stateNum, stateNum> K; // Kalman gain matrix
    matrixf32_t<stateNum, stateNum> I; // Identity matrix

public:
    matrixf32_t<stateNum, 1> x;   // state vector
    matrixf32_t<controlNum, 1> u; // control input vector
    matrixf32_t<stateNum, 1> y;   // measurement vector

    float32_t dT; // time difference between two measurement

    /* We don't need default contructor */
    Kalman() = delete;

    /* Initialize:
     *  transition matrices,
     *  covariance matrices,
     *  initial control input
     *  measurement matrix
     *  identity matrix
     *  initial state
     */
    Kalman(Kalman_InitStruct<stateNum, controlNum> initial)
        : A(initial.A), B(initial.B), Q(initial.Q), H(initial.H), R(initial.R), P(initial.P), x(initial.x), u(initial.u), dT(initial.dT)
    {
        I.setIdentity();
    }

    /* Compute and update */
    void update()
    {
        /* Predict state vector */
        x = A * x + B * u;
        printf("x:\n");
        x.printData();
        
        /* Predict state covariance matrix */
        P = A * P * A.trans() + Q;
        printf("P:\n");
        P.printData();

        /* Calculate kalman gain */
        matrixf32_t<stateNum, stateNum> K_denom = H * P * H.trans() + R;
        printf("K_denom:\n");
        K_denom.printData();
        K = P * H * K_denom.inverse();
        printf("K:\n");
        K.printData();

        x = x + K * (y - H * x);
        printf("x:\n");
        x.printData();

        /* Update covariance matrix */
        P = (I - K * H) * P;
        printf("P:\n");
        P.printData();
    }

    /* As usual, but need not to do explicitly */
    ~Kalman() = default;
};

#endif