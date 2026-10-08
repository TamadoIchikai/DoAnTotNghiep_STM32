/*
 * PoE_matrixManip.h
 *
 *  Created on: Jul 11, 2026
 *      Author: User
 */

#ifndef INC_POE_MATRIXMANIP_H_
#define INC_POE_MATRIXMANIP_H_

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

void Vec_to_so3(const float32_t vector[3], float32_t so3_mat[9]);
void Twist_to_se3(const float32_t twist_vec[6], float32_t se3_mat[16]);
void so3_to_Vec(const float32_t so3_mat[9], float32_t vector[3]);
void se3_to_Twist(const float32_t se3_mat[16], float32_t twist_vec[6]);
void TransMat_To_Rp(const float32_t T[16], float32_t R[9], float32_t p[3]);
void transInv(const float32_t T[16], float32_t InvMat[16]);

#endif /* INC_POE_MATRIXMANIP_H_ */
