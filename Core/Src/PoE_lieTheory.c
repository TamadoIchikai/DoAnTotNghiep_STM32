/*
 * PoE_lieTheory.c
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */
#include "PoE_lieTheory.h"
#include "PoE_helper.h"
#include "PoE_matrixManip.h"

void exp_so3(const float32_t mat_so3[9], float32_t R[9]) {
    float32_t omgtheta[3];
    float32_t omghat[3];
    float32_t theta;

    so3_to_Vec(mat_so3, omgtheta);
    Vec_to_Angle(omgtheta, omghat, &theta);

    float16_t thresh = 1e-6f;
    if (theta < thresh) {
        memset(R, 0, 9 * sizeof(float32_t));
        R[0] = 1.0f; R[4] = 1.0f; R[8] = 1.0f;
        return;
    }

    float32_t omghatMat[9];
    Vec_to_so3(omghat, omghatMat);

    float32_t omghatMatSq[9];
    arm_matrix_instance_f32 matA = {3, 3, omghatMat};
    arm_matrix_instance_f32 matB = {3, 3, omghatMat};
    arm_matrix_instance_f32 matC = {3, 3, omghatMatSq};
    arm_mat_mult_f32(&matA, &matB, &matC);

    float32_t stheta = arm_sin_f32(theta);
    float32_t ctheta = arm_cos_f32(theta);

    for (int i = 0; i < 9; i++) {
        R[i] = (i % 4 == 0 ? 1.0f : 0.0f) + (stheta * omghatMat[i]) + ((1.0f - ctheta) * omghatMatSq[i]);
    }
}

void exp_se3(const float32_t se3Mat[16], float32_t T[16]) {
    float32_t mat_so3[9] = {
        se3Mat[0], se3Mat[1], se3Mat[2],
        se3Mat[4], se3Mat[5], se3Mat[6],
        se3Mat[8], se3Mat[9], se3Mat[10]
    };

    float32_t v[3] = {se3Mat[3], se3Mat[7], se3Mat[11]};
    float32_t omgtheta[3];
    float32_t theta;

    so3_to_Vec(mat_so3, omgtheta);
    arm_sqrt_f32((omgtheta[0]*omgtheta[0]) + (omgtheta[1]*omgtheta[1]) + (omgtheta[2]*omgtheta[2]), &theta);

    memset(T, 0, 16 * sizeof(float32_t));
    T[15] = 1.0f;

    float16_t thresh = 1e-6f;
    if (theta < thresh) {
        T[0] = 1.0f; T[5] = 1.0f; T[10] = 1.0f;
        T[3] = v[0]; T[7] = v[1]; T[11] = v[2];
        return;
    }

    float32_t R[9];
    exp_so3(mat_so3, R);

    T[0] = R[0]; T[1] = R[1]; T[2] = R[2];
    T[4] = R[3]; T[5] = R[4]; T[6] = R[5];
    T[8] = R[6]; T[9] = R[7]; T[10] = R[8];

    float32_t omghat[3];
    Vec_to_Angle(omgtheta, omghat, &theta);

    float32_t omghatMat[9];
    Vec_to_so3(omghat, omghatMat);

    float32_t omghatMatSq[9];
    arm_matrix_instance_f32 m_omg = {3, 3, omghatMat};
    arm_matrix_instance_f32 m_omgSq = {3, 3, omghatMatSq};
    arm_mat_mult_f32(&m_omg, &m_omg, &m_omgSq);

    float32_t stheta = arm_sin_f32(theta);
    float32_t ctheta = arm_cos_f32(theta);
    float32_t G[9];

    for (int i = 0; i < 9; i++) {
        G[i] = (i % 4 == 0 ? theta : 0.0f) + ((1.0f - ctheta) * omghatMat[i]) + ((theta - stheta) * omghatMatSq[i]);
    }

    float32_t trans[3];
    arm_matrix_instance_f32 m_G = {3, 3, G};
    arm_matrix_instance_f32 m_v = {3, 1, v};
    arm_matrix_instance_f32 m_trans = {3, 1, trans};
    arm_mat_mult_f32(&m_G, &m_v, &m_trans);

    T[3] = trans[0] / theta;
    T[7] = trans[1] / theta;
    T[11] = trans[2] / theta;
}

void log_SO3(const float32_t R[9], float32_t omgtheta[9]) {
    float32_t trace = R[0] + R[4] + R[8];
    float32_t acos_Input = (trace - 1.0f) / 2.0f;

    float16_t acos_f16 = (float16_t)acos_Input;

    if (acos_f16 >= 1.0f) {
        memset(omgtheta, 0, 9 * sizeof(float32_t));
        return;
    }

    if (acos_f16 <= -1.0f) {
        float32_t omg[3];
        float16_t thresh = 1e-6f;

        if ((1.0f + R[0]) >= thresh) {
            float32_t sq;
            arm_sqrt_f32(2.0f * (1.0f + R[0]), &sq);
            float32_t inv = 1.0f / sq;
            omg[0] = (1.0f + R[0]) * inv;
            omg[1] = R[3] * inv;
            omg[2] = R[6] * inv;
        } else if ((1.0f + R[4]) >= thresh) {
            float32_t sq;
            arm_sqrt_f32(2.0f * (1.0f + R[4]), &sq);
            float32_t inv = 1.0f / sq;
            omg[0] = R[1] * inv;
            omg[1] = (1.0f + R[4]) * inv;
            omg[2] = R[7] * inv;
        } else {
            float32_t sq;
            arm_sqrt_f32(2.0f * (1.0f + R[8]), &sq);
            float32_t inv = 1.0f / sq;
            omg[0] = R[2] * inv;
            omg[1] = R[5] * inv;
            omg[2] = (1.0f + R[8]) * inv;
        }
        omg[0] *= PI; omg[1] *= PI; omg[2] *= PI;
        Vec_to_so3(omg, omgtheta);
        return;
    }

    float32_t theta;
    fast_acos_f32(acos_Input, &theta);
    float32_t factor = theta / (2.0f * arm_sin_f32(theta));

    omgtheta[0] = factor * (R[0] - R[0]);
    omgtheta[1] = factor * (R[1] - R[3]);
    omgtheta[2] = factor * (R[2] - R[6]);
    omgtheta[3] = factor * (R[3] - R[1]);
    omgtheta[4] = factor * (R[4] - R[4]);
    omgtheta[5] = factor * (R[5] - R[7]);
    omgtheta[6] = factor * (R[6] - R[2]);
    omgtheta[7] = factor * (R[7] - R[5]);
    omgtheta[8] = factor * (R[8] - R[8]);
}

void log_SE3(const float32_t T[16], float32_t expMat[16]) {
    float32_t R[9] = {T[0], T[1], T[2], T[4], T[5], T[6], T[8], T[9], T[10]};
    float32_t p[3] = {T[3], T[7], T[11]};
    float32_t omgtheta_mat[9];

    log_SO3(R, omgtheta_mat);
    memset(expMat, 0, 16 * sizeof(float32_t));

    uint8_t is_zero = 1;
    for (int i = 0; i < 9; i++) {
        if (omgtheta_mat[i] != 0.0f) {
            is_zero = 0; break;
        }
    }

    if (is_zero) {
        expMat[3] = p[0]; expMat[7] = p[1]; expMat[11] = p[2];
        return;
    }

    float32_t trace = R[0] + R[4] + R[8];
    float32_t theta;
    fast_acos_f32((trace - 1.0f) / 2.0f, &theta);

    float32_t omgtheta_sq[9];
    arm_matrix_instance_f32 m_omg = {3, 3, omgtheta_mat};
    arm_matrix_instance_f32 m_omgSq = {3, 3, omgtheta_sq};
    arm_mat_mult_f32(&m_omg, &m_omg, &m_omgSq);

    float32_t cot_theta_half = arm_cos_f32(theta / 2.0f) / arm_sin_f32(theta / 2.0f);
    float32_t factor = (1.0f / theta) - (cot_theta_half / 2.0f);
    float32_t G_inv[9];

    for (int i = 0; i < 9; i++) {
        G_inv[i] = (i % 4 == 0 ? 1.0f : 0.0f) - (0.5f * omgtheta_mat[i]) + ((factor / theta) * omgtheta_sq[i]);
    }

    float32_t v[3];
    arm_matrix_instance_f32 m_Ginv = {3, 3, G_inv};
    arm_matrix_instance_f32 m_p = {3, 1, p};
    arm_matrix_instance_f32 m_v = {3, 1, v};
    arm_mat_mult_f32(&m_Ginv, &m_p, &m_v);

    expMat[0] = omgtheta_mat[0]; expMat[1] = omgtheta_mat[1]; expMat[2] = omgtheta_mat[2]; expMat[3] = v[0];
    expMat[4] = omgtheta_mat[3]; expMat[5] = omgtheta_mat[4]; expMat[6] = omgtheta_mat[5]; expMat[7] = v[1];
    expMat[8] = omgtheta_mat[6]; expMat[9] = omgtheta_mat[7]; expMat[10] = omgtheta_mat[8]; expMat[11] = v[2];
}
