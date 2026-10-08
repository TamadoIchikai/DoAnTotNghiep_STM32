/*
 * PoE_lieTheory.h
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#ifndef INC_POE_LIETHEORY_H_
#define INC_POE_LIETHEORY_H_

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


void exp_so3(const float32_t mat_so3[9], float32_t R[9]);
void exp_se3(const float32_t se3Mat[16], float32_t T[16]);
void log_SO3(const float32_t R[9], float32_t omgtheta[9]);
void log_SE3(const float32_t T[16], float32_t expMat[16]);
void adjoint_T(const float32_t T[16], float32_t AdT[36]);


#endif /* INC_POE_LIETHEORY_H_ */
