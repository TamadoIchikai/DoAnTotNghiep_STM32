/*
 * PoE_helper.h
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#ifndef INC_POE_HELPER_H_
#define INC_POE_HELPER_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Include CMSIS-DSP if you are using float32_t or its matrix structs */
#include "arm_math.h"

/* Check for float16_t support or map to float32_t to guarantee compilation */
#if defined(__ARM_FP16_FORMAT_IEEE)
#include <arm_fp16.h>
#else
typedef float32_t float16_t;
#endif

void Vec_to_Angle(const float32_t omgtheta[3], float32_t omghat[3], float32_t *theta);
void fast_acos_f32(float32_t x, float32_t *out);

#endif /* INC_POE_HELPER_H_ */
