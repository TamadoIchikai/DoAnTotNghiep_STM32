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

void Compute_Twist_Error(const float32_t Td[16], const float32_t T_EE[16], float32_t V_EE[6]) {
    float32_t T_EE_inv[16];
    float32_t T_error[16];
    float32_t se3_mat[16];
    // 1. Calculate inverse of T_EE: transInv(T_EE)
    transInv(T_EE, T_EE_inv);
    // 2. Perform matrix multiplication: T_error = Td * T_EE_inv
    // Need to initialize CMSIS-DSP matrix structures for 4x4 matrices
    arm_matrix_instance_f32 mat_Td;
    arm_matrix_instance_f32 mat_T_EE_inv;
    arm_matrix_instance_f32 mat_T_error;
    // (Note: discarding 'const' qualifier to satisfy arm_mat_init_f32, but it will not be modified)
    arm_mat_init_f32(&mat_Td, 4, 4, (float32_t *)Td);
    arm_mat_init_f32(&mat_T_EE_inv, 4, 4, T_EE_inv);
    arm_mat_init_f32(&mat_T_error, 4, 4, T_error);
    arm_mat_mult_f32(&mat_Td, &mat_T_EE_inv, &mat_T_error);
    // 3. Calculate mat_Log6: T_error -> se(3) matrix representation
    log_SE3(T_error, se3_mat);
    // 4. se3_To_vector: Convert the 4x4 se(3) matrix to a 6x1 Twist vector (V_EE)
    se3_to_Twist(se3_mat, V_EE);
}

void jacobianSpace(const float32_t *Slist, const float32_t *thetalist, uint8_t num_joints, float32_t *Js) {
    // 1. Initialize Js = S (reshape array of vectors to 6 x num_joints CMSIS matrix)
    for (uint8_t i = 0; i < num_joints; i++) {
        for (uint8_t j = 0; j < 6; j++) {
            Js[j * num_joints + i] = Slist[i * 6 + j];
        }
    }

    // 2. Initialize T = eye(4)
    float32_t T[16] = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };

    arm_matrix_instance_f32 mat_T, mat_T_step, mat_T_next;
    float32_t T_step[16];
    float32_t T_next[16];

    arm_mat_init_f32(&mat_T, 4, 4, T);
    arm_mat_init_f32(&mat_T_step, 4, 4, T_step);
    arm_mat_init_f32(&mat_T_next, 4, 4, T_next);

    for (uint8_t i = 1; i < num_joints; i++) {
        // twist = S(:, i-1) * theta(i-1)
        float32_t twist[6];
        for (uint8_t j = 0; j < 6; j++) {
            twist[j] = Slist[(i - 1) * 6 + j] * thetalist[i - 1];
        }

        // vector_To_se3
        float32_t se3_mat[16];
        Twist_to_se3(twist, se3_mat);

        // expMat_6
        exp_se3(se3_mat, T_step);

        // T = T * expMat
        arm_mat_mult_f32(&mat_T, &mat_T_step, &mat_T_next);
        for(int k = 0; k < 16; k++) T[k] = T_next[k];

        // Calculate adjoint of current T
        float32_t AdT[36];
        adjoint_T(T, AdT);

        // Compute new column: Js(:, i) = AdT * S(:, i)
        arm_matrix_instance_f32 mat_AdT, mat_Si, mat_Jsi;
        float32_t Si[6];
        float32_t Jsi[6];

        for (uint8_t j = 0; j < 6; j++) Si[j] = Slist[i * 6 + j];

        arm_mat_init_f32(&mat_AdT, 6, 6, AdT);
        arm_mat_init_f32(&mat_Si, 6, 1, Si);
        arm_mat_init_f32(&mat_Jsi, 6, 1, Jsi);

        arm_mat_mult_f32(&mat_AdT, &mat_Si, &mat_Jsi);

        // Update column in Js
        for (uint8_t j = 0; j < 6; j++) {
            Js[j * num_joints + i] = Jsi[j];
        }
    }
}

void DLS(const float32_t *Js, uint8_t num_joints, float32_t *Js_Inverse) {
    // lambda = 0.05 -> lambda^2 = 0.0025
    float32_t lambda_sq = 0.0025f;

    // Allocate memory for intermediate matrices
    float32_t Js_T[num_joints * 6]; // num_joints x 6
    float32_t Js_JsT[36];           // 6 x 6
    float32_t inv_mat_data[36];     // 6 x 6

    arm_matrix_instance_f32 mat_Js, mat_Js_T, mat_Js_JsT, mat_inv, mat_Js_Inv;

    arm_mat_init_f32(&mat_Js, 6, num_joints, (float32_t *)Js);
    arm_mat_init_f32(&mat_Js_T, num_joints, 6, Js_T);
    arm_mat_init_f32(&mat_Js_JsT, 6, 6, Js_JsT);
    arm_mat_init_f32(&mat_inv, 6, 6, inv_mat_data);
    arm_mat_init_f32(&mat_Js_Inv, num_joints, 6, Js_Inverse);

    // 1. Calculate transpose of Js -> Js'
    arm_mat_trans_f32(&mat_Js, &mat_Js_T);

    // 2. Calculate Js * Js'
    arm_mat_mult_f32(&mat_Js, &mat_Js_T, &mat_Js_JsT);

    // 3. Add Damping Factor: Js * Js' + lambda^2 * eye(6)
    // Only add to the diagonal
    for (int i = 0; i < 6; i++) {
        Js_JsT[i * 6 + i] += lambda_sq;
    }

    // 4. Calculate inverse: inv(Js * Js' + lambda^2 * eye(6))
    arm_mat_inverse_f32(&mat_Js_JsT, &mat_inv);

    // 5. Final pseudo-inverse: Js_Inverse = Js' * inv(...)
    arm_mat_mult_f32(&mat_Js_T, &mat_inv, &mat_Js_Inv);
}
