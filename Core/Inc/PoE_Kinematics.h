/*
 * PoE_Kinematics.h
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#ifndef INC_POE_KINEMATICS_H_
#define INC_POE_KINEMATICS_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Include CMSIS-DSP if you are using float32_t or its matrix structs */
#include "arm_math.h"
#include "PoE_matrixManip.h"
/* Check for float16_t support or map to float32_t to guarantee compilation */
#if defined(__ARM_FP16_FORMAT_IEEE)
#include <arm_fp16.h>
#else
typedef float32_t float16_t;
#endif
void ForwardKinematics_Space(const float32_t *M, const float32_t *Slist, const float32_t *thetalist, uint8_t num_joints, float32_t *T_out);

#endif /* INC_POE_KINEMATICS_H_ */
