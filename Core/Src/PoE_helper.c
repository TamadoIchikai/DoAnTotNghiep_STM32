/*
 * PoE_helper.c
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */
#include "PoE_helper.h"

void Vec_to_Angle(const float32_t omgtheta[3], float32_t omghat[3], float32_t *theta) {
    arm_sqrt_f32((omgtheta[0] * omgtheta[0]) +
                 (omgtheta[1] * omgtheta[1]) +
                 (omgtheta[2] * omgtheta[2]), theta);

    float16_t thresh = 1e-6f;
    if (*theta < thresh) {
        omghat[0] = 0.0f; omghat[1] = 0.0f; omghat[2] = 0.0f;
    } else {
        omghat[0] = omgtheta[0] / *theta;
        omghat[1] = omgtheta[1] / *theta;
        omghat[2] = omgtheta[2] / *theta;
    }
}

/* Helper to map math.h acos to CMSIS fast math atan2 */
void fast_acos_f32(float32_t x, float32_t *out) {
    float32_t sqrt_val;
    arm_sqrt_f32(1.0f - (x * x), &sqrt_val);
    arm_atan2_f32(sqrt_val, x, out);
}
