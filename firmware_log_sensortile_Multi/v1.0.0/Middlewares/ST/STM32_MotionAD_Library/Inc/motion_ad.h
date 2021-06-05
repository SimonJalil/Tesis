/**
  ******************************************************************************
  * @file    motion_ad.h
  * @author  MEMS Application Team
  * @version V1.0.3
  * @date    28-May-2020
  * @brief   Header for motion_ad module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ********************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOTION_AD_H_
#define _MOTION_AD_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup MIDDLEWARES
  * @{
  */

/** @defgroup MOTION_AD MOTION_AD
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup MOTION_AD_Exported_Types MOTION_AD_Exported_Types
  * @{
  */

typedef struct
{
  float Acc[3];        /* Acceleration in X, Y, Z axis in [g] */
  unsigned int Press;  /* Pressure in [Pa] (e.g. 101325) */
  float Temp;          /* Temperature in [degC] */
} MAD_input_t;

typedef enum
{
  MAD_ONLAND  = 0x00,
  MAD_TAKEOFF = 0x01,
  MAD_LANDING = 0x02
} MAD_output_t;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup MOTION_AD_Exported_Functions MOTION_AD_Exported_Functions
 * @{
 */

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Initialize the MotionAD engine
  * @param  xl_odr  accelerometer ODR in Hz (nearrest int)
  * @retval None
  */
void MotionAD_Initialize(int xl_odr);

/**
  * @brief  Run airplane mode algorithm
  * @param  data_in   pointer to acceleration [g], pressure [hBar] and temperature [degC]
  * @param  data_out  pointer to algorithm result (ONLAND, TAKEOFF, LANDING)
  * @retval None
  */
void MotionAD_Update(MAD_input_t *data_in, MAD_output_t *data_out);

/**
  * @brief  Get the library version
  * @param  version pointer to an array of 35 char
  * @retval Number of characters in the version string
  */
uint8_t MotionAD_GetLibVersion(char *version);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* _MOTION_AD_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
