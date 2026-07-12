/*
 * PoE_Kinematics.c
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */


#include "PoE_kinematics.h"
#include "PoE_lieTheory.h"
#include "arm_math.h"
#include <string.h>

void ForwardKinematics_Space(const float32_t *M, const float32_t *Slist, const float32_t *thetalist, uint8_t num_joints, float32_t *T_out)
{
    float32_t T_current[16] = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };

    float32_t T_next[16];
    float32_t twist[6];
    float32_t se3Mat[16];
    float32_t expMat[16];

    arm_matrix_instance_f32 arm_T_curr, arm_expMat, arm_T_next, arm_M, arm_T_out_inst;

    // Accumulate transformations from the base to the end-effector
    for (uint8_t i = 0; i < num_joints; i++)
    {
        // 1. Calculate the twist for the current joint: twist = S * theta
        for (uint8_t j = 0; j < 6; j++)
        {
            // Assuming Slist is stored as a flat array of size (6 * num_joints)
            twist[j] = Slist[i * 6 + j] * thetalist[i];
        }

        // 2. Convert twist to se(3) matrix
        Twist_to_se3(twist, se3Mat);

        // 3. Calculate matrix exponential
        exp_se3(se3Mat, expMat);

        // 4. Multiply T_current = T_current * expMat
        arm_mat_init_f32(&arm_T_curr, 4, 4, T_current);
        arm_mat_init_f32(&arm_expMat, 4, 4, expMat);
        arm_mat_init_f32(&arm_T_next, 4, 4, T_next);

        arm_mat_mult_f32(&arm_T_curr, &arm_expMat, &arm_T_next);

        // 5. Update T_current for the next iteration
        memcpy(T_current, T_next, sizeof(float32_t) * 16);
    }

    // Apply the home configuration M: T_out = T_current * M
    arm_mat_init_f32(&arm_T_curr, 4, 4, T_current);
    arm_mat_init_f32(&arm_M, 4, 4, (float32_t *)M); // Cast required by CMSIS-DSP
    arm_mat_init_f32(&arm_T_out_inst, 4, 4, T_out);

    arm_mat_mult_f32(&arm_T_curr, &arm_M, &arm_T_out_inst);
}
