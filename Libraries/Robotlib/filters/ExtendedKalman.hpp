#include "kalman.hpp"

template <uint16_t stateNum, uint16_t controlNum, uint16_t measurementNum>
class ExtendedKalman : public Kalman<stateNum, controlNum, measurementNum>
{

public:
    /* We don't need default contructor */
    ExtendedKalman() = delete;

    ExtendedKalman(Kalman_InitStruct<stateNum, controlNum, measurementNum> initial)
        : Kalman<stateNum, controlNum, measurementNum>(initial) {}

    /* Override the update method */

    void update() override
    {
        /* Predict state vector */
        this->x = this->A * this->x + this->B * this->u + this->w;

        /* Predict state covariance matrix */
        this->P = this->A * this->P * this->A.trans() + this->Q;

        /* Calculate Jacobian matrices */
        matrixf32_t<measurementNum, stateNum> JH = calculateJacobianH(this->x);
        matrixf32_t<stateNum, stateNum> JA = calculateJacobianA(this->x, this->u, this->dT);

        /* Calculate Kalman gain */
        matrixf32_t<measurementNum, measurementNum> S = JH * this->P * JH.trans() + this->R;
        this->K = this->P * JH.trans() * S.inverse();

        this->y = this->y + this->z;

        /* Update estimate using the measurement */
        this->x = this->x + this->K * (this->y - calculateMeasurement(this->x));

        /* Update covariance matrix */
        this->P = (matrixf32_t<stateNum, stateNum>::Identity() - this->K * JH) * this->P;
    }

private:
    // Functions to calculate Jacobian matrices and measurement predictions
    // Replace these with your actual implementation for the specific system
    matrixf32_t<measurementNum, stateNum> calculateJacobianH(const matrixf32_t<stateNum, 1> &x)
    {
        // Calculate the Jacobian matrix of the measurement function H at x
        // Return the Jacobian matrix
        return matrixf32_t<measurementNum, stateNum>::Zero(); // Replace with actual implementation
    }

    matrixf32_t<stateNum, stateNum> calculateJacobianA(const matrixf32_t<stateNum, 1> &x,
                                                       const matrixf32_t<controlNum, 1> &u, float32_t dT)
    {
        // Calculate the Jacobian matrix of the state transition function A at x and u
        // Return the Jacobian matrix
        return matrixf32_t<stateNum, stateNum>::Zero(); // Replace with actual implementation
    }

    matrixf32_t<measurementNum, 1> calculateMeasurement(const matrixf32_t<stateNum, 1> &x)
    {
        // Calculate the predicted measurement based on state x
        // Return the predicted measurement
        return matrixf32_t<measurementNum, 1>::Zero(); // Replace with actual implementation
    }

    ~ExtendedKalman() = default;
};
