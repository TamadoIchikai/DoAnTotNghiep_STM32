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
