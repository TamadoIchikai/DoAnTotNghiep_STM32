/*
 * PoE_matrixManip.c
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#include "PoE_matrixManip.h"


void Vec_to_so3(const float32_t vector[3], float32_t so3_mat[9]) {
    so3_mat[0] = 0.0f; so3_mat[1] = -vector[2]; so3_mat[2] = vector[1];
    so3_mat[3] = vector[2]; so3_mat[4] = 0.0f; so3_mat[5] = -vector[0];
    so3_mat[6] = -vector[1]; so3_mat[7] = vector[0]; so3_mat[8] = 0.0f;
}

void Twist_to_se3(const float32_t twist_vec[6], float32_t se3_mat[16]) {
    se3_mat[0] = 0.0f; se3_mat[1] = -twist_vec[2]; se3_mat[2] = twist_vec[1]; se3_mat[3] = twist_vec[3];
    se3_mat[4] = twist_vec[2]; se3_mat[5] = 0.0f; se3_mat[6] = -twist_vec[0]; se3_mat[7] = twist_vec[4];
    se3_mat[8] = -twist_vec[1]; se3_mat[9] = twist_vec[0]; se3_mat[10] = 0.0f; se3_mat[11] = twist_vec[5];
    se3_mat[12] = 0.0f; se3_mat[13] = 0.0f; se3_mat[14] = 0.0f; se3_mat[15] = 0.0f;
}

void so3_to_Vec(const float32_t so3_mat[9], float32_t vector[3]) {
    vector[0] = so3_mat[7]; vector[1] = so3_mat[2]; vector[2] = so3_mat[3];
}

void se3_to_Twist(const float32_t se3_mat[16], float32_t twist_vec[6]) {
    twist_vec[0] = se3_mat[9]; twist_vec[1] = se3_mat[2]; twist_vec[2] = se3_mat[4];
    twist_vec[3] = se3_mat[3]; twist_vec[4] = se3_mat[7]; twist_vec[5] = se3_mat[11];
}

void TransMat_To_Rp(const float32_t T[16], float32_t R[9], float32_t p[3]) {
    // Extract 3x3 Rotation matrix R from 4x4 Transformation matrix T
    R[0] = T[0]; R[1] = T[1]; R[2] = T[2];
    R[3] = T[4]; R[4] = T[5]; R[5] = T[6];
    R[6] = T[8]; R[7] = T[9]; R[8] = T[10];

    // Extract 3x1 position vector p
    p[0] = T[3];
    p[1] = T[7];
    p[2] = T[11];
}

void transInv(const float32_t T[16], float32_t InvMat[16]) {
    float32_t R[9];
    float32_t p[3];
    float32_t RT[9];
    float32_t p_inv[3];

    // 1. Get R and p
    TransMat_To_Rp(T, R, p);

    // Initialize CMSIS-DSP matrix instances
    arm_matrix_instance_f32 mat_R, mat_RT, mat_p, mat_p_inv;
    arm_mat_init_f32(&mat_R, 3, 3, R);
    arm_mat_init_f32(&mat_RT, 3, 3, RT);
    arm_mat_init_f32(&mat_p, 3, 1, p);
    arm_mat_init_f32(&mat_p_inv, 3, 1, p_inv);

    // 2. RT = transpose(R)
    arm_mat_trans_f32(&mat_R, &mat_RT);

    // 3. p_inv = RT * p
    arm_mat_mult_f32(&mat_RT, &mat_p, &mat_p_inv);

    // 4. p_inv = -p_inv (Scale by -1 using CMSIS-DSP vector math)
    arm_scale_f32(p_inv, -1.0f, p_inv, 3);

    // 5. Construct the final 4x4 Inverse Matrix
    // Row 1
    InvMat[0] = RT[0]; InvMat[1] = RT[1]; InvMat[2]  = RT[2]; InvMat[3]  = p_inv[0];
    // Row 2
    InvMat[4] = RT[3]; InvMat[5] = RT[4]; InvMat[6]  = RT[5]; InvMat[7]  = p_inv[1];
    // Row 3
    InvMat[8] = RT[6]; InvMat[9] = RT[7]; InvMat[10] = RT[8]; InvMat[11] = p_inv[2];

    // Row 4
    InvMat[12] = 0.0f; InvMat[13] = 0.0f; InvMat[14] = 0.0f; InvMat[15] = 1.0f;
}
