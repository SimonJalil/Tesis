/**
******************************************************************************
* @file    MotionFX_Manager.c
* @author  Central LAB
* @version V3.1.0
* @date    14-July-2017
* @brief   Header for MotionFX_Manager.c
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "MotionFX_Manager.h"
#include "motion_fx.h"
#include "main.h"

/* Private defines -----------------------------------------------------------*/
#define FROM_MDPS_TO_DPS    0.001
#define FROM_MGAUSS_TO_UT50 (0.1f/50.0f)
#define SAMPLETODISCARD 15
#define GBIAS_ACC_TH_SC_6X (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC_6X (2.0f*0.002f)
#define GBIAS_MAG_TH_SC_6X (2.0f*0.001500f)
#define GBIAS_ACC_TH_SC_9X (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC_9X (2.0f*0.002f)
#define GBIAS_MAG_TH_SC_9X (2.0f*0.001500f)


/* Delta time mSec for Deltafusion */
#define MOTIONFX_ENGINE_DELTATIME 0.01f

/* Exported Variables -------------------------------------------------------------*/
MFX_output_t iDataOUT;
MFX_input_t iDataIN;

/* Imported Variables -------------------------------------------------------------*/
extern float sensitivity_Mul_acc;
extern float sensitivity_Mul_gyr;

extern SensorAxes_t MAG_Offset;

/* Private Variables -------------------------------------------------------------*/
static int discardedCount = 0;

static MFX_knobs_t iKnobs;
static MFX_knobs_t* ipKnobs;

/* Private function prototypes -----------------------------------------------*/

extern void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec );
/**
  * @brief  Initialize MotionFX engine
  * @retval None
  */
void MotionFX_manager_init(void)
{
  char LibVersion[36];
  
  MotionFX_initialize();
  MotionFX_GetLibVersion(LibVersion);

  //ST MotionFX Engine Initializations
  ipKnobs = &iKnobs;

  MotionFX_getKnobs(ipKnobs);

  ipKnobs->gbias_acc_th_sc_6X = GBIAS_ACC_TH_SC_6X;
  ipKnobs->gbias_gyro_th_sc_6X = GBIAS_GYRO_TH_SC_6X;
  ipKnobs->gbias_mag_th_sc_6X = GBIAS_MAG_TH_SC_6X;

  ipKnobs->gbias_acc_th_sc_9X = GBIAS_ACC_TH_SC_9X;
  ipKnobs->gbias_gyro_th_sc_9X = GBIAS_GYRO_TH_SC_9X;
  ipKnobs->gbias_mag_th_sc_9X = GBIAS_MAG_TH_SC_9X;
  

  ipKnobs->acc_orientation[0] ='w';
  ipKnobs->acc_orientation[1] ='s';
  ipKnobs->acc_orientation[2] ='u';

  ipKnobs->gyro_orientation[0] = 'w';
  ipKnobs->gyro_orientation[1] = 's';
  ipKnobs->gyro_orientation[2] = 'u';
  
  ipKnobs->mag_orientation[0] = 's';
  ipKnobs->mag_orientation[1] = 'w';
  ipKnobs->mag_orientation[2] = 'u';


  ipKnobs->output_type = MFX_ENGINE_OUTPUT_ENU;
  ipKnobs->LMode = 1;
  ipKnobs->MTime = 10;
  ipKnobs->modx  = 1;

  MotionFX_setKnobs(ipKnobs);
  
  MotionFX_enable_6X(MFX_ENGINE_DISABLE);
  MotionFX_enable_9X(MFX_ENGINE_DISABLE);

  discardedCount = 0;

  //TargetBoardFeatures.MotionFXIsInitalized=1;
  //ALLMEMS1_PRINTF("Initialized %s\n\r", LibVersion);
  if(SendOverUSB){
      	  CDC_Fill_Buffer(( uint8_t * )LibVersion, strlen( LibVersion ));
        }
}

void MotionFX_set_new_values(void)
{

   MotionFX_initialize();
   ipKnobs = &iKnobs;

   MotionFX_getKnobs(ipKnobs);

   ipKnobs->gbias_acc_th_sc_6X = (2.0f*0.000765f);
   ipKnobs->gbias_gyro_th_sc_6X = (2.0f*0.002f);
   ipKnobs->gbias_mag_th_sc_6X = (2.0f*0.001500f);

   ipKnobs->gbias_acc_th_sc_9X = (2.0f*0.000765f);
   ipKnobs->gbias_gyro_th_sc_9X = (2.0f*0.002f);
   ipKnobs->gbias_mag_th_sc_9X = (2.0f*0.001500f);

   ipKnobs->output_type = MFX_ENGINE_OUTPUT_ENU;
   ipKnobs->LMode = 1;   //defecto en 1 ajuste de manera estatica Si lo pongo en
							// 2 se va rotando el dado en sentido antihorario. Con 0 pasa lo mismo
   ipKnobs->modx  = 1;	//

   MotionFX_setKnobs(ipKnobs);

   MotionFX_enable_6X(MFX_ENGINE_DISABLE);
   MotionFX_enable_9X(MFX_ENGINE_DISABLE);

   discardedCount = 0;

   //TargetBoardFeatures.MotionFXIsInitalized=1;

}

void MotionFX_print_config(void)
{
	float flt;
	int32_t d1, d2;
	MFX_MagCal_output_t data_out;

	ALLMEMS1_PRINTF("*****MotionFX values*****\r\n");
	ALLMEMS1_PRINTF("Samples to discard: %d\r\n", SAMPLETODISCARD);
	flt = MOTIONFX_ENGINE_DELTATIME;
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("MotionFx_Engine_Deltatime: %d.%02d\r\n", d1, d2);
	flt = ipKnobs->ATime;
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("ATime:  %d.%02d\r\n", d1, d2);
	flt = ipKnobs->FrTime;
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("FrTime: %d.%02d\r\n", d1, d2);
	ALLMEMS1_PRINTF("LMode: %d\r\n", ipKnobs->LMode);
	flt = ipKnobs->MTime;
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("MTime: %d.%02d\r\n", d1, d2);
	flt = ipKnobs->gbias_acc_th_sc_6X;
	floatToInt(flt, &d1, &d2, 6);
	ALLMEMS1_PRINTF("gbias_acc_th_sc_6X: %d.%06d\r\n", d1, d2);
	flt = ipKnobs->gbias_gyro_th_sc_6X;
	floatToInt(flt, &d1, &d2, 6);
	ALLMEMS1_PRINTF("gbias_gyro_th_sc_6X: %d.%06d\r\n", d1, d2);
	flt = ipKnobs->gbias_mag_th_sc_6X;
	floatToInt(flt, &d1, &d2, 6);
	ALLMEMS1_PRINTF("gbias_mag_th_sc_6X: %d.%06d\r\n", d1, d2);
	flt = ipKnobs->gbias_acc_th_sc_9X;
	floatToInt(flt, &d1, &d2, 6);
	ALLMEMS1_PRINTF("gbias_acc_th_sc_9X: %d.%06d\r\n", d1, d2);
	flt = ipKnobs->gbias_gyro_th_sc_9X;
	floatToInt(flt, &d1, &d2, 6);
	ALLMEMS1_PRINTF("gbias_gyro_th_sc_9X: %d.%06d\r\n", d1, d2);
	flt = ipKnobs->gbias_mag_th_sc_9X;
	floatToInt(flt, &d1, &d2, 6);
	ALLMEMS1_PRINTF("gbias_mag_th_sc_9X: %d.%06d\r\n", d1, d2);
	floatToInt(flt, &d1, &d2, 6);
	ALLMEMS1_PRINTF("modx: %d\r\n", ipKnobs->modx);
	ALLMEMS1_PRINTF("\r\n");
	ALLMEMS1_PRINTF("*************MagCal***********\r\n");
	MotionFX_MagCal_getParams(&data_out);
	flt = data_out.hi_bias[0];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("Hi-bias x: %d.%02d\r\n", d1, d2);
	flt = data_out.hi_bias[1];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("Hi-bias y: %d.%02d\r\n", d1, d2);
	flt = data_out.hi_bias[2];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("Hi-bias z: %d.%02d\r\n", d1, d2);
	ALLMEMS1_PRINTF("Quality: %d\r\n", data_out.cal_quality);
	ALLMEMS1_PRINTF("\r\n");

}

void MotionFx_print_qrh(void){

	float flt;
	int32_t d1, d2;
	char auxC[100];

	ALLMEMS1_PRINTF("\r\n");
	//MotionFX_manager_getDataOUT();
	ALLMEMS1_PRINTF("*********Quaternion********\r\n");
	flt = iDataOUT.quaternion_6X[0];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("Q6X: %d.%02d, ", d1, d2);
	flt = iDataOUT.quaternion_6X[1];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("%d.%02d, ", d1, d2);
	flt = iDataOUT.quaternion_6X[2];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("%d.%02d, ", d1, d2);
	flt = iDataOUT.quaternion_6X[3];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("%d.%02d\r\n", d1, d2);
	flt = iDataOUT.quaternion_9X[0];
	floatToInt(flt, &d1, &d2, 3);
	ALLMEMS1_PRINTF("Q9X: %d.%03d, ", (int)d1, (int)d2);
	flt = iDataOUT.quaternion_9X[1];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("%d.%02d, ", d1, d2);
	flt = iDataOUT.quaternion_9X[2];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("%d.%02d, ", d1, d2);
	flt = iDataOUT.quaternion_9X[3];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("%d.%02d\r\n", d1, d2);
	ALLMEMS1_PRINTF("*********Rotation********\r\n");
	flt = iDataOUT.rotation_6X[0];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("R6X: %d.%02d, ", d1, d2);
	flt = iDataOUT.rotation_6X[1];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("%d.%02d, ", d1, d2);
	flt = iDataOUT.rotation_6X[2];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("%d.%02d, ", d1, d2);
	flt = iDataOUT.rotation_6X[3];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("%d.%02d\r\n", d1, d2);
	flt = iDataOUT.rotation_9X[0];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("R9X: %d.%02d, ", d1, d2);
	flt = iDataOUT.rotation_9X[1];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("%d.%02d, ", d1, d2);
	flt = iDataOUT.rotation_9X[2];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("%d.%02d, ", d1, d2);
	flt = iDataOUT.rotation_9X[3];
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("%d.%02d\r\n", d1, d2);
	ALLMEMS1_PRINTF("*********Heading********\r\n");
	flt = iDataOUT.heading_6X;
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("H6X: %d.%02d, \r\n", d1, d2);

	flt = iDataOUT.heading_9X;
	floatToInt(flt, &d1, &d2, 2);
	ALLMEMS1_PRINTF("H9X: %d.%02d\r\n", d1, d2);

}

/**
  * @brief  Run sensor fusion algorithm
  * @param SensorAxesRaw_t ACC_Value_Raw Acceleration value (x/y/z)
  * @param SensorAxes_t GYR_Value Gyroscope value (x/y/z)
  * @param SensorAxes_t MAG_Value Magneto value (x/y/z)
  * @retval None
  */
void MotionFX_manager_run( SensorAxesRaw_t ACC_Value_Raw,SensorAxes_t GYR_Value,SensorAxes_t MAG_Value)
{  
  float deltaTimeA = MOTIONFX_ENGINE_DELTATIME;

  iDataIN.gyro[0] = GYR_Value.AXIS_X  * FROM_MDPS_TO_DPS;
  iDataIN.gyro[1] = GYR_Value.AXIS_Y  * FROM_MDPS_TO_DPS;
  iDataIN.gyro[2] = GYR_Value.AXIS_Z  * FROM_MDPS_TO_DPS;

  //iDataIN.gyro[0] = GYR_Value.AXIS_X  * sensitivity_Mul_gyro;
  //iDataIN.gyro[1] = GYR_Value.AXIS_Y  * sensitivity_Mul_gyro;
  //iDataIN.gyro[2] = GYR_Value.AXIS_Z  * sensitivity_Mul_gyro;

  iDataIN.acc[0] = ACC_Value_Raw.AXIS_X * sensitivity_Mul_acc;
  iDataIN.acc[1] = ACC_Value_Raw.AXIS_Y * sensitivity_Mul_acc;
  iDataIN.acc[2] = ACC_Value_Raw.AXIS_Z * sensitivity_Mul_acc;


  iDataIN.mag[0] = (MAG_Value.AXIS_X - MAG_Offset.AXIS_X) * FROM_MGAUSS_TO_UT50;
  iDataIN.mag[1] = (MAG_Value.AXIS_Y - MAG_Offset.AXIS_Y) * FROM_MGAUSS_TO_UT50;
  iDataIN.mag[2] = (MAG_Value.AXIS_Z - MAG_Offset.AXIS_Z) * FROM_MGAUSS_TO_UT50;



  if(discardedCount == SAMPLETODISCARD)
  {
    //MotionFX_propagate(&iDataOUT, &iDataIN, &deltaTimeA);  //version nueva de la libreria
    //MotionFX_update(&iDataOUT, &iDataIN, &deltaTimeA, NULL);  //version nueva de la libreria
	 MotionFX_propagate(&iDataOUT, &iDataIN, MOTIONFX_ENGINE_DELTATIME);  //version vieja de la libreria
	 MotionFX_update(&iDataOUT, &iDataIN, MOTIONFX_ENGINE_DELTATIME, NULL);  //version vieja de la libreria
  }
  else
  {
    discardedCount++;
  }
}

/**
 * @brief  Start 6 axes MotionFX engine
 * @retval None
 */
void MotionFX_manager_start_6X(void)
{
  MotionFX_enable_6X(MFX_ENGINE_ENABLE);
}

/**
 * @brief  Stop 6 axes MotionFX engine
 * @retval None
 */
void MotionFX_manager_stop_6X(void)
{
  MotionFX_enable_6X(MFX_ENGINE_DISABLE);
}

/**
 * @brief  Start 9 axes MotionFX engine
 * @retval None
 */
void MotionFX_manager_start_9X(void)
{
  MotionFX_enable_9X(MFX_ENGINE_ENABLE);
}

/**
 * @brief  Stop 9 axes MotionFX engine
 * @retval None
 */
void MotionFX_manager_stop_9X(void)
{
  MotionFX_enable_9X(MFX_ENGINE_DISABLE);
}

/**
* @brief  Get MotionFX Engine data Out
* @param  None
* @retval MFX_output *iDataOUT MotionFX Engine data Out
*/
MFX_output_t* MotionFX_manager_getDataOUT(void)
{
  return &iDataOUT;
}

/**
* @brief  Get MotionFX Engine data IN
* @param  None
* @retval MFX_input *iDataIN MotionFX Engine data IN
*/
MFX_input_t* MotionFX_manager_getDataIN(void)
{
  return &iDataIN;
}

/**
  * @brief  Run magnetometer calibration algorithm
  * @param  None
  * @retval None
  */
void MotionFX_manager_MagCal_run(MFX_MagCal_input_t *data_in, MFX_MagCal_output_t *data_out)
{
   MotionFX_MagCal_run(data_in);
   MotionFX_MagCal_getParams(data_out);
}


/**
 * @brief  Start magnetometer calibration
 * @param  None
 * @retval None
 */
void MotionFX_manager_MagCal_start(int sampletime)
{
  MotionFX_MagCal_init(sampletime, 1);
}


/**
 * @brief  Stop magnetometer calibration
 * @param  None
 * @retval None
 */
void MotionFX_manager_MagCal_stop(int sampletime)
{
  MotionFX_MagCal_init(sampletime, 0);
}


/**
 * @brief  Load calibration parameter from memory
 * @param  dataSize length ot the data
 * @param  data pointer to the data
 * @retval (1) fail, (0) success
 */
char MotionFX_LoadMagCalFromNVM(unsigned short int dataSize, unsigned int *data)
{
   char Success=0;
   //Success= ReCallCalibrationFromMemory(dataSize / 4, (uint32_t*) data);
   return Success;
}

/**
 * @brief  Save calibration parameter to memory
 * @param  dataSize length ot the data
 * @param  data pointer to the data
 * @retval (1) fail, (0) success
 */
char MotionFX_SaveMagCalInNVM(unsigned short int dataSize, unsigned int *data)
{
  char Success=0;
  //Success= SaveCalibrationToMemory(dataSize / 4, (uint32_t*) data);
  return Success;
}
/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
