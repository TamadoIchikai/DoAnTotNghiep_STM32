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
void Compute_Twist_Error(const float32_t Td[16], const float32_t T_EE[16], float32_t V_EE[6]);
void jacobianSpace(const float32_t *Slist, const float32_t *thetalist, uint8_t num_joints, float32_t *Js);
void DLS(const float32_t *Js, uint8_t num_joints, float32_t *Js_Inverse);

#endif /* INC_POE_KINEMATICS_H_ */
