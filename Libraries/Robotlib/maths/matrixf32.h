/****************************************************************************
 *                       MARIX_F32_H
 *
 *       - Created by Sagar Chaudhary on Nov 13, 2023
 *
 *       This matrixf32 library is dependent on arm_math library.
 *
 ****************************************************************************/

/*
    Dynamic memory allocation is avoided.
    Template struct is used, so wisely use only few sizes of matrix otherwise it will consume lots of memory.
    No exeption is handled, instead used static assertion.
    Assign or copy data carefully especialy from where array of data through pointer is used to do.
    constructor = default is avoided, otherwise one class pointer will point other class data.
*/

#ifndef __MATRIX_F32_H__
#define __MATRIX_F32_H__

#include "arm_math.h"
#include "stdio.h"
#include "memory.h"

template <uint16_t numRows, uint16_t numCols>
struct matrixf32_t
{
    float32_t data[numRows * numCols];
    arm_matrix_instance_f32 arm_mat;

    matrixf32_t() noexcept
    {
        arm_mat_init_f32(&arm_mat, numRows, numCols, data);
    }

    matrixf32_t(const matrixf32_t &other) noexcept
    {
        arm_mat_init_f32(&arm_mat, numRows, numCols, data);
        memcpy(arm_mat.pData, other.arm_mat.pData, numRows * numCols * 4);
    }
    

    // template <typename... Args>
    // matrixf32_t(Args... values)
    //     : data{static_cast<float32_t>(values)...}
    // {
    //     static_assert(sizeof...(values) == numRows * numCols,
    //                   "Incorrect number of values. If there is static_cast error just above this line, then it is probably be due to mismatched size of matrix passed to copy or assign.");
    //     arm_mat_init_f32(&arm_mat, numRows, numCols, data);

    //     printf("template\n");
    // }

    matrixf32_t(const float32_t *pSrc) noexcept
    {
        arm_mat_init_f32(&arm_mat, numRows, numCols, data);
        memcpy(arm_mat.pData, pSrc, numRows * numCols * 4);
    }

    matrixf32_t &operator=(const matrixf32_t &other) noexcept
    {
        memcpy(arm_mat.pData, other.arm_mat.pData, numRows * numCols * 4);
        return *this;
    }

    matrixf32_t &operator=(float32_t *pSrc) noexcept
    {
        memcpy(arm_mat.pData, pSrc, numRows * numCols * 4);
        return *this;
    }

    matrixf32_t operator+(const matrixf32_t &rhs) noexcept const
    {
        matrixf32_t dst;
        arm_mat_add_f32(&this->arm_mat, &rhs.arm_mat, &dst.arm_mat);
        return dst;
    }

    matrixf32_t operator-(const matrixf32_t &rhs) noexcept const
    {
        matrixf32_t dst;
        arm_mat_sub_f32(&this->arm_mat, &rhs.arm_mat, &dst.arm_mat);
        return dst;
    }

    template <uint16_t numAny>
    matrixf32_t<numRows, numAny> operator*(const matrixf32_t<numCols, numAny> &rhs) noexcept const
    {
        matrixf32_t<numRows, numAny> dst;
        arm_mat_mult_f32(&this->arm_mat, &rhs.arm_mat, &dst.arm_mat);
        return dst;
    }

    matrixf32_t<numCols, numRows> trans() noexcept const
    {
        matrixf32_t<numCols, numRows> dst;
        arm_mat_trans_f32(&this->arm_mat, &dst.arm_mat);
        return dst;
    }

    matrixf32_t inverse() noexcept const
    {
        static_assert(numRows == numCols,
                      "Inverse of rectangular matrix does not exists. Matrix must be square to have inverse.");
        matrixf32_t dst;
        arm_status status = arm_mat_inverse_f32(&this->arm_mat, &dst.arm_mat);
        if (status == ARM_MATH_SINGULAR)
        {
            printf("#RED#!! EXCEPTION:: MATRIX INVERSION FAILED !!\n");
        }

        return dst;
    }

    void fill(const float32_t &value) noexcept
    {
        arm_fill_f32(value, arm_mat.pData, numRows * numCols);
    }

    void setIdentity() noexcept
    {
        static_assert(numRows == numCols, "Only square matrix can be identity matrix");
        for (uint16_t i = 0; i < numRows; ++i)
        {
            for (uint16_t j = 0; j < numCols; ++j)
            {
                arm_mat.pData[i * numCols + j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    }

    void printData() noexcept const
    {
        for (uint16_t i = 0; i < numRows; ++i)
        {
            for (uint16_t j = 0; j < numCols; ++j)
            {
                printf("%f\t", arm_mat.pData[i * numCols + j]);
            }
            printf("\n");
        }
    }

    ~matrixf32_t() = default;
};

#endif
