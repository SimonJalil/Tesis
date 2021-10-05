/**
******************************************************************************
* @file    DataLog/Src/main.c
* @author  Central Labs
* @version V1.1.1
* @date    06-Dec-2016
* @brief   Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

#include <string.h> /* strlen */
#include <stdio.h>  /* sprintf */
#include <math.h>   /* trunc */
#include "main.h"

#include "SensorTile_conf.h"
//#include "SensorTile2.h"
#include "SensorTile_bus.h"
#include "SensorTile_motion_sensors.h"
#include "SensorTile_motion_sensors_ex.h"
#include "motion_fx.h"
#include "MotionFX_Manager.h"
#include "motion_pw.h"
#include "motion_aw.h"
#include "MetaDataManager.h"
#include "bluenrg_utils.h"
#include "motion_fd.h"

#include "sensor_service.h"
#include "bluenrg_types.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "string.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "hci_const.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"
#include "hci.h"
#include "hci_le.h"
#include "sm.h"


#include "datalog_application.h"
volatile uint16_t rateCDC = 100;  //milliseconds
#include "usbd_cdc_interface.h"



/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Data acquisition period [ms] */
#define DATA_PERIOD_MS 1
#define DATA_PERIOD_CLOSE_FILE 10000 + 1500
#define SIZE_BUFFER_RAM  500
#define PACKET_LENGTH 10
#define SAMPLES_PER_BLOCK 100
//#define SAMPLES_PER_BLOCK 50
#define DATA_BLOCK_LENGTH 25000
#define QUATERNION_PERIOD_MS 10
#define FALLDET_PERIOD_MS 20
//#define DATALOG_SAMPLES
//#define DEBUG_ALGO
//#define COMPUTE_QUATERNION

#define FROM_MG_TO_G    0.001
#define FROM_MDPS_TO_DPS    0.001
#define BLUEMSYS_CHECK_CALIBRATION ((uint32_t)0x12345678)



extern void *MotionCompObj[MOTION_INSTANCES_NBR];
extern volatile uint32_t HCI_ProcessEvent;
extern int connected;
extern SensorAxes_t acceleration;
extern SensorAxesRaw_t acceleration_raw;
extern SensorAxes_t angular_velocity;
extern SensorAxes_t magnetic_field;
SensorAxes_t quat_axes;
MFX_output_t *MotionFX_Engine_Out;

float sensitivity_acc;
float sensitivity_gyro;
/* Acc sensitivity multiply by FROM_MG_TO_G constant */
float sensitivity_Mul_acc;
float sensitivity_Mul_gyro;

MFX_MagCal_output_t magOffset;
SensorAxes_t MAG_Offset;

uint32_t CalibrationData[30];

TIM_HandleTypeDef	 TimHandleM;

/* Code for MotionFX integration - Start Section */
static volatile uint32_t Quaternion      =0;
/* Code for MotionFX integration - End Section */

unsigned char isCal = 1;  //deberia estar en 0 en la app final
//por el momento se supone que est� calibrado


uint32_t ForceReCalibration    =0;

//static void MX_CRC_Init(void);

//static unsigned char ResetCalibrationInMemory(void);

//static void MagCalibTest(void);
//static void ReCalibration(void);

///* Code for MotionFX integration - Start Section */
//static void ComputeQuaternions(void);
///* Code for MotionFX integration - End Section */
//
//
//void printQuaternion( void );

//static void InitTimers(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
static void Init_BlueNRG_Custom_Services(void);
static void Init_BlueNRG_Stack(void);
extern void doubleToInt( double in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec );

//#define NOT_DEBUGGING

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* SendOverUSB = 1  --> Send sensors data via USB */
uint8_t SendOverUSB = 1;
uint8_t enabledUSBCDC = 1;
uint8_t timerEnabled = 0;

extern uint8_t set_connectable;

TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

USBD_HandleTypeDef  USBD_Device;
static volatile uint8_t MEMSInterrupt = 0;
static volatile uint8_t no_H_HTS221 = 0;
static volatile uint8_t no_T_HTS221 = 0;


uint8_t bdaddr[6];

static RTC_HandleTypeDef RtcHandle;
static void *LSM6DSM_X_0_handle = NULL;
static void *LSM6DSM_G_0_handle = NULL;
static void *LSM303AGR_X_0_handle = NULL;
static void *LSM303AGR_M_0_handle = NULL;
static void *LPS22HB_P_0_handle = NULL;
static void *LPS22HB_T_0_handle = NULL;
static void *HTS221_H_0_handle = NULL;
static void *HTS221_T_0_handle = NULL;


static char dataTimeStamp[100];
volatile char Mydata[256];

volatile uint8_t saveSamplesSD = 0;
volatile uint8_t pingPongSampleBlock = 0;
volatile uint16_t countSamples = 0;
volatile uint16_t samplesToSave = 0;

volatile uint16_t lenStrPing = 0;
volatile uint16_t lenStrPong = 0;

static uint32_t	msInit = 0;

static uint32_t msTickDebug;
static uint32_t msTickPrevDebug;
static uint32_t msTickPrevQuaternion;

// 	tamano de la ventana
#define	ventana 70
//	indice del buffer
int	indice;

//	umbrales para valores de aceler�metro y gir�scopo
float umbralACC;
float umbralGYRO;
uint16_t amountShots;

//	buffers del tama�o de la ventana para aceler�metro y gir�scopo
//	habr�a que inicializarlos en cero
float bufferACC[ventana];
float bufferGYRO[ventana];

//	acumuladores para toda la ventana
float	acumACC;
float	acumGYRO;

//	variable de salida (true es disparo detectado)
uint8_t	disparo;

float AccX;
float AccY;
float AccZ;
float GyroX;
float GyroY;
float GyroZ;
float pressure;

/* Private function prototypes -----------------------------------------------*/

static void Error_Handler( void );
static void MX_CRC_Init(void);
static void RTC_Config( void );
static void RTC_TimeStampConfig( void );
static void initializeAllSensors( void );
static void configSensors(void);
static void disableUnnecessarySensors( void );
 void HCI_Event_CB(void *pckt);

//static void setParametersFiltersSensorsLsm6dsm();
//static void getParametersFiltersSensorsLsm6dsm();
//int computeShotAlgov1( void );
//void initAlgov1( void );
//void printDisparo( void );
void getSensorSensivity();

/* Private functions ---------------------------------------------------------*/
 
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main( void )
{
  uint32_t msTick, msTickPrev = 0, FFmod = 0;
  uint8_t lenStr = 0;
  uint8_t resampling = 0;
  uint8_t flagFD = 0, flagFF = 0, flagFF1 , flagFFM = 0;



  /* STM32L4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
  */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize CRC */
  MX_CRC_Init();

  /* Initialize RTC */  RTC_Config();
  RTC_TimeStampConfig();


  /* enable USB power on Pwrctrl CR2 register */
  HAL_PWREx_EnableVddUSB();

  if(SendOverUSB || enabledUSBCDC) /* Configure the USB */
  	{
    /*** USB CDC Configuration ***/
    /* Init Device Library 2*/
    USBD_Init(&USBD_Device, &VCP_Desc, 0);
    /* Add Supported Class */
    USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
    /* Add Interface callbacks for AUDIO and CDC Class */
    USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
    /* Start Device Process */
    USBD_Start(&USBD_Device);
  	}

  /* Initialize the BlueNRG */
  Init_BlueNRG_Stack();

  /* Initialize the BlueNRG Custom services */
  Init_BlueNRG_Custom_Services();

  /* Configure and disable all the Chip Select pins */
  Sensor_IO_SPI_CS_Init_All();
  
  /* Initialize and Enable the available sensors */
  initializeAllSensors();
  enableAllSensors();
  disableUnnecessarySensors();
  configSensors();

  msInit = HAL_RCC_GetSysClockFreq() / 1000000;
  NecessityToSaveMetaDataManager = 0;


  #ifdef COMPUTE_QUATERNION
  MX_CRC_Init();
  MotionFX_manager_init();
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  MotionFX_manager_start_9X();
  #endif

  MotionFD_Initialize();
  MotionPW_Initialize();
  MotionAW_Initialize();
  {
  char acc_orientation[3];
  acc_orientation[0] ='e';
  acc_orientation[1] ='n';
  acc_orientation[2] ='u';
  MotionAW_SetOrientation_Acc(acc_orientation);
  }
  getSensorSensivity();
  msTickDebug = HAL_GetTick();
  msTickPrevQuaternion = msTickDebug;
  amountShots = 0;

  while (1) {
	  /* handle BLE event */
	  if(HCI_ProcessEvent)
	 {
	   HCI_ProcessEvent=0;
	   hci_user_evt_proc();
	 }
	  /* Update the BLE advertise data and make the Board connectable */
	 if(set_connectable)
	 {
	   setConnectable();
	   set_connectable = FALSE;
	 }
    /* Get sysTick value and check if it's time to execute the task */
    msTick = HAL_GetTick();
    msTickDebug = HAL_GetTick();
    if(((msTick % DATA_PERIOD_MS) == 0) && (msTickPrev != msTick) && (timerEnabled == 0))
    {
      RTC_Handler( &RtcHandle, dataTimeStamp);
      Accelero_Sensor_Handler(LSM6DSM_X_0_handle);
      Gyro_Sensor_Handler( LSM6DSM_G_0_handle);
      Magneto_Sensor_Handler( LSM303AGR_M_0_handle);
      BSP_PRESSURE_Get_Press(LPS22HB_P_0_handle, &pressure);
      msTickPrev = msTick;
    }
    if(((msTickDebug % FALLDET_PERIOD_MS) == 0) && (msTickPrevDebug != msTickDebug) && (timerEnabled == 0))
     {
    	MFD_input_t data_in;
    	MFD_output_t data_out;
		MAW_input_t data_inAW;
		MAW_output_t data_outAW;
    	MPW_input_t data_inPW;
		MPW_output_t data_outPW;

		data_in.AccX = (float)acceleration.AXIS_X;           /* Acceleration in X axis in [mg] */
    	data_in.AccY = (float)acceleration.AXIS_Y;           /* Acceleration in Y axis in [mg] */
    	data_in.AccZ = (float)acceleration.AXIS_Z;           /* Acceleration in Z axis in [mg] */
    	data_in.Press = pressure;

    	data_inPW.AccX = data_inAW.AccX = (float)acceleration.AXIS_X / 1000.0f;
    	data_inPW.AccY = data_inAW.AccY = (float)acceleration.AXIS_Y / 1000.0f;
    	data_inPW.AccZ = data_inAW.AccZ = (float)acceleration.AXIS_Z / 1000.0f;

		MotionAW_Update(&data_inAW,&data_outAW, msTick);
		int32_t activityConversion = 0;
		activityConversion = data_outAW.current_activity - 4;
		if (activityConversion < MPW_UNKNOWN_ACTIVITY || activityConversion > MPW_JOGGING)
			{
			activityConversion = MPW_UNKNOWN_ACTIVITY;
			}
		data_inPW.CurrentActivity = (MPW_activity_t)activityConversion;
		MotionPW_Update(&data_inPW,&data_outPW);

		MotionFD_Update(&data_in, &data_out);
		if(data_out){
			flagFD = 1;
		}

		BSP_ACCELERO_Get_Free_Fall_Detection_Status_Ext(LSM6DSM_X_0_handle, &flagFF1);
		if(flagFF1){
			flagFF = 1;
		}


		FFmod = (uint32_t)sqrt(acceleration.AXIS_X * acceleration.AXIS_X + acceleration.AXIS_Y * acceleration.AXIS_Y +acceleration.AXIS_Z * acceleration.AXIS_Z) ;
//		if(FFmod < 200){
//			flagFF1 = 1;
//		}
		msTickPrevDebug = msTickDebug;
		lenStr = 0;
		//sprintf(data, "%s", dataTimeStamp);
		//lenStr += strlen(dataTimeStamp);
		//sprintf(data + lenStr, ",");
		//lenStr++;
		resampling++;
		if (resampling == 50)
				{
				resampling = 0;
//				lenStr = lenStr + sprintf(data + lenStr, "%d,%d,%d", (int)acceleration.AXIS_X, (int)acceleration.AXIS_Y, (int)acceleration.AXIS_Z);
				lenStr = lenStr + sprintf(Mydata + lenStr, "%4d", (int)FFmod);
				sprintf(Mydata + lenStr, ",");
				lenStr++;

				lenStr = lenStr + sprintf(Mydata + lenStr, "%d", flagFD);
				sprintf(Mydata + lenStr, ",");
				lenStr++;

				lenStr = lenStr + sprintf(Mydata + lenStr, "%d", flagFF);
			    sprintf(Mydata + lenStr, ",");
				lenStr++;

//				lenStr = lenStr + sprintf(data + lenStr, "FFM %d", flagFFM);
//			    sprintf(data + lenStr, ",");
//				lenStr++;

				lenStr = lenStr + sprintf(Mydata + lenStr, "%d", (uint16_t)data_outAW.current_activity );
				sprintf(Mydata + lenStr, ",");
				lenStr++;

				lenStr = lenStr + sprintf(Mydata + lenStr, "%5d", (uint16_t)data_outPW.Nsteps);


				sprintf(Mydata + lenStr, "\r\n");
				lenStr = lenStr + 2;
				CDC_Fill_Buffer(( uint8_t * )Mydata, strlen( Mydata ));
				if(connected== TRUE){
					Environmental_Update(( uint8_t * )Mydata, strlen( Mydata ));
				}

			}
     }
    ///* Go to Sleep */
    //__WFI();
  }
}

//
//void printDisparo( void ){
//	  int32_t d1;
//	  int32_t d2;
//	  uint8_t lenStr;
//
//	  lenStr = 0;
//	  sprintf(data, "disparo,");
//	  lenStr += 8;
//
//	  sprintf(data + lenStr, "%s", dataTimeStamp);
//	  lenStr += strlen(dataTimeStamp);
//	  sprintf(data + lenStr, ",");
//	  lenStr++;
//
//	  //amountShots
//	  lenStr = lenStr + sprintf(data + lenStr, "%ld", amountShots);
//	  sprintf(data + lenStr, ",");
//	  lenStr++;
//
//	  //acumACC
//	  floatToInt(acumACC, &d1, &d2, 3);
//	  sprintf(dataAux, "%d.%02d", d1, d2);
//	  sprintf(data + lenStr, "%s", dataAux);
//	  lenStr += strlen(dataAux);
//	  sprintf(data + lenStr, ",");
//	  lenStr++;
//
//	  //acumGYRO
//	  floatToInt(acumGYRO, &d1, &d2, 3);
//	  sprintf(dataAux, "%d.%02d", d1, d2);
//	  sprintf(data + lenStr, "%s", dataAux);
//	  lenStr += strlen(dataAux);
//	  sprintf(data + lenStr, ",");
//	  lenStr++;
//
//	  sprintf(data + lenStr, "\r\n");
//	  lenStr = lenStr + 2;
//
//	  if(SendOverUSB){
//		  CDC_Fill_Buffer(( uint8_t * )data, strlen( data ));
//	  }
//}
//
//void printQuaternion( void ){
//	int32_t d1;
//	int32_t d2;
//	uint8_t lenStr;
//
//	lenStr = 0;
//	//sprintf(data, "quat,");
//	//lenStr += 5;
//
//	sprintf(data + lenStr, "%s", dataTimeStamp);
//	lenStr += strlen(dataTimeStamp);
//	sprintf(data + lenStr, ",");
//	lenStr++;
//
//	//lenStr = lenStr + sprintf(data + lenStr, "%ld,%ld,%ld,", (int)quat_axes.AXIS_X, (int)quat_axes.AXIS_Y, (int)quat_axes.AXIS_Z);
//	sprintf(data + lenStr, ",");
//	lenStr++;
//
//	sprintf(data + lenStr, ",");
//	lenStr++;
//
//	sprintf(data + lenStr, ",");
//	lenStr++;
//
//	sprintf(data + lenStr, ",");
//	lenStr++;
//
//	sprintf(data + lenStr, ",");
//	lenStr++;
//
//	sprintf(data + lenStr, ",");
//	lenStr++;
//
//	sprintf(data + lenStr, ",");
//	lenStr++;
//
//	sprintf(data + lenStr, ",");
//	lenStr++;
//
//	sprintf(data + lenStr, ",");
//	lenStr++;
//
//	//floatToInt(MotionFX_Engine_Out->quaternion_9X[0], &d1, &d2, 4);
//	floatToInt(MotionFX_Engine_Out->rotation_9X[0], &d1, &d2, 4);
//	sprintf(dataAux, "%d.%02d", d1, d2);
//	sprintf(data + lenStr, "%s", dataAux);
//	lenStr += strlen(dataAux);
//	sprintf(data + lenStr, ",");
//	lenStr++;
//
//	//floatToInt(MotionFX_Engine_Out->quaternion_9X[1], &d1, &d2, 4);
//	floatToInt(MotionFX_Engine_Out->rotation_9X[1], &d1, &d2, 4);
//	sprintf(dataAux, "%d.%02d", d1, d2);
//	sprintf(data + lenStr, "%s", dataAux);
//	lenStr += strlen(dataAux);
//	sprintf(data + lenStr, ",");
//	lenStr++;
//
//	//floatToInt(MotionFX_Engine_Out->quaternion_9X[2], &d1, &d2, 4);
//	floatToInt(MotionFX_Engine_Out->rotation_9X[2], &d1, &d2, 4);
//	sprintf(dataAux, "%d.%02d", d1, d2);
//	sprintf(data + lenStr, "%s", dataAux);
//	lenStr += strlen(dataAux);
//	sprintf(data + lenStr, ",");
//	lenStr++;
///*
//	floatToInt(MotionFX_Engine_Out->quaternion_9X[3], &d1, &d2, 4);
//	sprintf(dataAux, "%d.%02d", d1, d2);
//	sprintf(data + lenStr, "%s", dataAux);
//	lenStr += strlen(dataAux);*/
//	//sprintf(data + lenStr, ",");
//	//lenStr++;
//
//
//	sprintf(data + lenStr, "\r\n");
//	lenStr = lenStr + 2;
//	//sprintf(data + lenStr, "\r\n");
//	//lenStr = lenStr + 2;
//
//
//	if(SendOverUSB){
//		CDC_Fill_Buffer(( uint8_t * )data, strlen( data ));
//	}
//}
//
//void initAlgov1(){
//
//	//	�ndice del buffer
//	indice		= 0;
//
//	//	umbrales para valores de aceler�metro y gir�scopo
//	//umbralACC 	= 500;	//correcto
//	//umbralGYRO 	= 100000;	//correcto
//	umbralACC = 500000;	//correcto /1000
//	umbralGYRO = 100000000;	//correcto /1000
//	//umbralACC 	= 50000;	//de simulacion de disparo
//	//umbralGYRO 	=  10000000;	//de simulacion de disparo
//
//	memset(bufferACC, 0, ventana);
//	memset(bufferGYRO, 0, ventana);
//
//	//	acumuladores para toda la ventana
//	acumACC 	= 0;
//	acumGYRO 	= 0;
//
//}
//
////llamar a la funcion cada 1 ms con los valores cargados en la variable con los ejes
//int computeShotAlgov1(){
//	//	variable de salida (true es disparo detectado)
//	uint8_t	disparo 	= 0;
//
//	AccX = (float)acceleration.AXIS_X;
//	AccY = (float)acceleration.AXIS_Y;
//	AccZ = (float)acceleration.AXIS_Z;
//	GyroX = (float)angular_velocity.AXIS_X;
//	GyroY = (float)angular_velocity.AXIS_Y;
//	GyroZ = (float)angular_velocity.AXIS_Z;
//
//	//AccX = 5;
//	//AccY = 5;
//	//AccZ = 5;
//	//GyroX = 10;
//	//GyroY = 10;
//	//GyroZ = 10;
//
//	if (indice >= ventana){
//		indice = 0;
//	}
//
//	acumACC = acumACC - bufferACC[indice];
//	bufferACC[indice]=abs(AccX)+ abs(AccY)+ abs(AccZ);
//	acumACC = acumACC + bufferACC[indice];
//
//	acumGYRO = acumGYRO - bufferGYRO[indice];
//	bufferGYRO[indice]= abs(GyroX)+abs(GyroY)+abs(GyroZ);
//	acumGYRO = acumGYRO + bufferGYRO[indice];
//	indice++;
//
//
////	acumACC 	= 0;
////	acumGYRO 	= 0;
//
////	for (int k=0;k<ventana;k++){
////		acumACC += bufferACC[k];
////		acumGYRO += bufferGYRO[k];
////	}
//
//	disparo = 0;
//
//	if (( acumACC > umbralACC )&&( acumGYRO > umbralGYRO)){
//		disparo	= 1;
//	}
//	return disparo;
//}
//


#ifdef COMPUTE_QUATERNION
//toma aproximadamente 5 ms el procesado del algoritmo de quaternion
/* Code for MotionFX integration - Star Section */
/* @brief  MotionFX Working function
 * @param  None
 * @retval None
 */
static void ComputeQuaternions(void)
{
  static int32_t calibIndex =0;

  MFX_MagCal_input_t mag_data_in;

  /* Check if is calibrated */
  if(isCal!=0x01){
    /* Run Compass Calibration @ 25Hz */
    calibIndex++;
    if (calibIndex == 4){
      calibIndex = 0;

      mag_data_in.mag[0]= magnetic_field.AXIS_X * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[1]= magnetic_field.AXIS_Y * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[2]= magnetic_field.AXIS_Z * FROM_MGAUSS_TO_UT50;
      mag_data_in.time_stamp = mag_time_stamp;
      mag_time_stamp += SAMPLE_PERIOD;
      MotionFX_manager_MagCal_run(&mag_data_in, &magOffset);

      /* Control the calibration status */
      if( (magOffset.cal_quality == MFX_MAGCALOK) ||
          (magOffset.cal_quality == MFX_MAGCALGOOD) )
      {
        isCal= 1;

        MAG_Offset.AXIS_X= (int32_t)(magOffset.hi_bias[0] * FROM_UT50_TO_MGAUSS);
        MAG_Offset.AXIS_Y= (int32_t)(magOffset.hi_bias[1] * FROM_UT50_TO_MGAUSS);
        MAG_Offset.AXIS_Z= (int32_t)(magOffset.hi_bias[2] * FROM_UT50_TO_MGAUSS);

        /* Disable magnetometer calibration */
        MotionFX_manager_MagCal_stop(SAMPLE_PERIOD);
      }

    }
  }else {
    calibIndex=0;
  }


  MotionFX_manager_run(acceleration_raw,angular_velocity,magnetic_field);

  MotionFX_Engine_Out = MotionFX_manager_getDataOUT();

  /* Read the quaternions */

    /* Save the quaternions values */
    if(MotionFX_Engine_Out->quaternion_9X[3] < 0){
      quat_axes.AXIS_X = (int32_t)(MotionFX_Engine_Out->quaternion_9X[0] * (-10000));
      quat_axes.AXIS_Y = (int32_t)(MotionFX_Engine_Out->quaternion_9X[1] * (-10000));
      quat_axes.AXIS_Z = (int32_t)(MotionFX_Engine_Out->quaternion_9X[2] * (-10000));
    } else {
      quat_axes.AXIS_X = (int32_t)(MotionFX_Engine_Out->quaternion_9X[0] * 10000);
      quat_axes.AXIS_Y = (int32_t)(MotionFX_Engine_Out->quaternion_9X[1] * 10000);
      quat_axes.AXIS_Z = (int32_t)(MotionFX_Engine_Out->quaternion_9X[2] * 10000);
    }

}
/* Code for MotionFX integration - End Section */
#endif

/**
* @brief  Initialize all sensors
* @param  None
* @retval None
*/
static void initializeAllSensors( void )
{
  if (BSP_ACCELERO_Init( LSM6DSM_X_0, &LSM6DSM_X_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }
  
  if (BSP_MOTION_SENSOR_Init(LSM6DSM_0, MOTION_ACCELERO) != BSP_ERROR_NONE){
	while(1);
  }

  if (BSP_GYRO_Init( LSM6DSM_G_0, &LSM6DSM_G_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }

  if (BSP_MOTION_SENSOR_Init(LSM6DSM_0, MOTION_GYRO) != BSP_ERROR_NONE){
  	while(1);
  }

  if (BSP_ACCELERO_Init( LSM303AGR_X_0, &LSM303AGR_X_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }
  
  if (BSP_MAGNETO_Init( LSM303AGR_M_0, &LSM303AGR_M_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }
  
  if (BSP_PRESSURE_Init( LPS22HB_P_0, &LPS22HB_P_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }
  
  if (BSP_TEMPERATURE_Init( LPS22HB_T_0, &LPS22HB_T_0_handle ) != COMPONENT_OK)
  {
    while(1);
  }
  
  if(BSP_TEMPERATURE_Init( HTS221_T_0, &HTS221_T_0_handle ) == COMPONENT_ERROR)
  {
    no_T_HTS221 = 1;
  }
  
  if(BSP_HUMIDITY_Init( HTS221_H_0, &HTS221_H_0_handle ) == COMPONENT_ERROR)
  {
    no_H_HTS221 = 1;
  }
  
}

/**
* @brief  Enable all sensors
* @param  None
* @retval None
*/
void enableAllSensors( void )
{
  BSP_ACCELERO_Sensor_Enable( LSM6DSM_X_0_handle );
  BSP_MOTION_SENSOR_Enable(LSM6DSM_0, MOTION_ACCELERO);
  BSP_MOTION_SENSOR_Enable(LSM6DSM_0, MOTION_GYRO);
  BSP_GYRO_Sensor_Enable( LSM6DSM_G_0_handle );
  BSP_ACCELERO_Sensor_Enable( LSM303AGR_X_0_handle );
  BSP_MAGNETO_Sensor_Enable( LSM303AGR_M_0_handle );
  BSP_PRESSURE_Sensor_Enable( LPS22HB_P_0_handle );
  BSP_TEMPERATURE_Sensor_Enable( LPS22HB_T_0_handle );
  if(!no_T_HTS221)
  {
    BSP_TEMPERATURE_Sensor_Enable( HTS221_T_0_handle );
    BSP_HUMIDITY_Sensor_Enable( HTS221_H_0_handle );
  }
  
}



/**
* @brief  Disable all sensors
* @param  None
* @retval None
*/
void disableAllSensors( void )
{
  BSP_ACCELERO_Sensor_Disable( LSM6DSM_X_0_handle );
  BSP_MOTION_SENSOR_Disable(LSM6DSM_0, MOTION_ACCELERO);
  BSP_MOTION_SENSOR_Disable(LSM6DSM_0, MOTION_GYRO);
  BSP_ACCELERO_Sensor_Disable( LSM303AGR_X_0_handle );
  BSP_GYRO_Sensor_Disable( LSM6DSM_G_0_handle );
  BSP_MAGNETO_Sensor_Disable( LSM303AGR_M_0_handle );
  BSP_HUMIDITY_Sensor_Disable( HTS221_H_0_handle );
  BSP_TEMPERATURE_Sensor_Disable( HTS221_T_0_handle );
  BSP_TEMPERATURE_Sensor_Disable( LPS22HB_T_0_handle );
  BSP_PRESSURE_Sensor_Disable( LPS22HB_P_0_handle );
}

/**
* @brief  Disable unnecessary sensors
* @param  None
* @retval None
*/
void disableUnnecessarySensors( void )
{
  //BSP_MAGNETO_Sensor_Disable( LSM303AGR_M_0_handle );
  BSP_HUMIDITY_Sensor_Disable( HTS221_H_0_handle );
  BSP_TEMPERATURE_Sensor_Disable( HTS221_T_0_handle );
  BSP_TEMPERATURE_Sensor_Disable( LPS22HB_T_0_handle );
  //BSP_PRESSURE_Sensor_Disable( LPS22HB_P_0_handle );
}

void configSensors(void){

	BSP_ACCELERO_Set_ODR_Value(LSM6DSM_X_0_handle, 50.0f);
	BSP_ACCELERO_Set_FS_Value(LSM6DSM_X_0_handle, 8.0f);
    BSP_ACCELERO_Enable_Free_Fall_Detection_Ext(LSM6DSM_X_0_handle);
    BSP_ACCELERO_Set_Free_Fall_Threshold_Ext(LSM6DSM_X_0_handle, LSM6DSM_ACC_GYRO_FF_THS_219mg);

	//BSP_ACCELERO_Set_FS_Value(LSM6DSM_X_0_handle, 16.0f); // 2.0f, 4.0f, 8.0f, 16.0f
	//BSP_ACCELERO_Set_ODR_Value(LSM6DSM_X_0_handle, 1660.0f); // 13.0f, 26.0f, 52.0f, 104.0f, 208.0f,
	  	  	  	  	  	  	  	  	  	  	  	  	  	  	 //	416.0f, 833.0f, 1660.0f, 3330.0f, 6660.0f
	BSP_GYRO_Set_FS_Value(LSM6DSM_G_0_handle, 2000.0f);	// 245.0f, 500.0f, 1000.0f, 2000.0f
	BSP_GYRO_Set_ODR_Value(LSM6DSM_G_0_handle, 1660.0f);	// 13.0f, 26.0f, 52.0f, 104.0f, 208.0f,
	 	  	  	  	  	 	 	 	 	 	 	 	 	//	416.0f, 833.0f, 1660.0f, 3330.0f, 6660.0f

	BSP_ACCELERO_Set_FS_Value(LSM303AGR_X_0_handle, 16.0f);	// 2.0f, 4.0f, 8.0f, 16.0f
	BSP_ACCELERO_Set_ODR_Value(LSM303AGR_X_0_handle, 5376.0f);	// 13.0f, 26.0f, 52.0f, 104.0f, 208.0f,
		 	  	  	  	  	 	 	 	 	 	 	 	 	//	416.0f, 833.0f, 1660.0f, 3330.0f, 6660.0f
	BSP_MAGNETO_Set_FS_Value(LSM303AGR_M_0_handle, 50.0f);
	BSP_MAGNETO_Set_ODR_Value(LSM303AGR_M_0_handle, 50.0f);

	BSP_PRESSURE_Set_ODR_Value(LPS22HB_P_0_handle, 25.0f);
}

void getSensorSensivity(){
	/* Read the Acc Sensitivity */
	BSP_ACCELERO_Get_Sensitivity(LSM6DSM_X_0_handle,&sensitivity_acc);
	sensitivity_Mul_acc = sensitivity_acc* ((float) FROM_MG_TO_G);

	/* Read the Gyro Sensitivity */
	BSP_GYRO_Get_Sensitivity(LSM6DSM_G_0_handle, &sensitivity_gyro);
	sensitivity_Mul_gyro = sensitivity_gyro* ((float) FROM_MDPS_TO_DPS);
}

///**
//  * @brief  CRC init function.
//  * @param  None
//  * @retval None
//  */
//static void MX_CRC_Init(void)
//{
//  hcrc.Instance = CRC;
//
//  if (HAL_CRC_Init(&hcrc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

/**
* @brief  Configures the RTC
* @param  None
* @retval None
*/
static void RTC_Config( void )
{
  /*##-1- Configure the RTC peripheral #######################################*/
  RtcHandle.Instance = RTC;
  
  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
  - Hour Format    = Format 12
  - Asynch Prediv  = Value according to source clock
  - Synch Prediv   = Value according to source clock
  - OutPut         = Output Disable
  - OutPutPolarity = High Polarity
  - OutPutType     = Open Drain */
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_12;
  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
  
  if ( HAL_RTC_Init( &RtcHandle ) != HAL_OK )
  {
    
    /* Initialization Error */
    Error_Handler();
  }
}

/**
* @brief  Configures the current time and date
* @param  None
* @retval None
*/
static void RTC_TimeStampConfig( void )
{
  
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;
  
  /*##-3- Configure the Date using BCD format ################################*/
  /* Set Date: Monday January 1st 2000 */
  sdatestructure.Year    = 0x00;
  sdatestructure.Month   = RTC_MONTH_JANUARY;
  sdatestructure.Date    = 0x01;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;
  
  if ( HAL_RTC_SetDate( &RtcHandle, &sdatestructure, FORMAT_BCD ) != HAL_OK )
  {
    
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-4- Configure the Time using BCD format#################################*/
  /* Set Time: 00:00:00 */
  stimestructure.Hours          = 0x00;
  stimestructure.Minutes        = 0x00;
  stimestructure.Seconds        = 0x00;
  stimestructure.SubSeconds     = 0;
  stimestructure.SecondFraction = 999;
  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  if ( HAL_RTC_SetTime( &RtcHandle, &stimestructure, FORMAT_BCD ) != HAL_OK )
  {   
    /* Initialization Error */
    Error_Handler();
  }
}

/**
* @brief  Configures the current time and date
* @param  hh the hour value to be set
* @param  mm the minute value to be set
* @param  ss the second value to be set
* @retval None
*/
void RTC_TimeRegulate( uint8_t hh, uint8_t mm, uint8_t ss )
{
  
  RTC_TimeTypeDef stimestructure;
  
  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.Hours          = hh;
  stimestructure.Minutes        = mm;
  stimestructure.Seconds        = ss;
  stimestructure.SubSeconds     = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  if ( HAL_RTC_SetTime( &RtcHandle, &stimestructure, FORMAT_BIN ) != HAL_OK )
  {
    /* Initialization Error */
    Error_Handler();
  }
}

static void MX_CRC_Init(void)
{
  __CRC_CLK_ENABLE();
}

/**
* @brief  EXTI line detection callbacks
* @param  GPIO_Pin: Specifies the pins connected EXTI line
* @retval None
*/
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
  MEMSInterrupt=1;
}

/** @brief Initialize the BlueNRG Stack
* @param None
* @retval None
*/
static void Init_BlueNRG_Stack(void)
{
  const char BoardName[8] = {NAME_STLBLE,0};
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;
  uint8_t  hwVersion;
  uint16_t fwVersion;

#ifdef STATIC_BLE_MAC
  {
    uint8_t tmp_bdaddr[6]= {STATIC_BLE_MAC};
    int32_t i;
    for(i=0;i<6;i++)
      bdaddr[i] = tmp_bdaddr[i];
  }
#endif /* STATIC_BLE_MAC */

  /* Initialize the BlueNRG HCI */
  hci_init(HCI_Event_CB, NULL);

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  /*
  * Reset BlueNRG again otherwise we won't
  * be able to change its MAC address.
  * aci_hal_write_config_data() must be the first
  * command after reset otherwise it will fail.
  */
  hci_reset();

  HAL_Delay(100);

  /* Create a Unique BLE MAC */
  {
    bdaddr[0] = (STM32_UUID[1]>>24)&0xFF;
    bdaddr[1] = (STM32_UUID[0]    )&0xFF;
    bdaddr[2] = (STM32_UUID[2] >>8)&0xFF;
    bdaddr[3] = (STM32_UUID[0]>>16)&0xFF;
    bdaddr[4] = (((STLBLE_VERSION_MAJOR-48)*10) + (STLBLE_VERSION_MINOR-48)+100)&0xFF;
    bdaddr[5] = 0xC0; /* for a Legal BLE Random MAC */
  }

  ret = aci_gatt_init();
  if(ret)
  {
    /*GATT_Init failed\r\n*/
    goto fail;
  }


    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

  if(ret != BLE_STATUS_SUCCESS)
  {
    /*GAP_Init failed\r\n*/
    goto fail;
  }

  ret = hci_le_set_random_address(bdaddr);

  if(ret)
  {
    /*Setting the Static Random BD_ADDR failed*/
    goto fail;
  }

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   7/*strlen(BoardName)*/, (uint8_t *)BoardName);

  if(ret)
  {
   /*aci_gatt_update_char_value failed*/
    while(1);
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL, 7, 16,
                                     USE_FIXED_PIN_FOR_PAIRING, 123456,
                                     BONDING);
  if (ret != BLE_STATUS_SUCCESS)
  {
    /*GAP setting Authentication failed*/
    goto fail;
  }

  /*SERVER: BLE Stack Initialized \r\n"
                "\t\tBoard type=%s HWver=%d, FWver=%d.%d.%c\r\n"
                  "\t\tBoardName= %s\r\n"
                    "\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n\n",
                    "SensorTile",
                    hwVersion,
                    fwVersion>>8,
                    (fwVersion>>4)&0xF,
                    (hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a',
                    BoardName,
                    bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]*/

  /* Set output power level */
  aci_hal_set_tx_power_level(1,4);

  return;

fail:
  return;
}


/** @brief Initialize all the Custom BlueNRG services
* @param None
* @retval None
*/
static void Init_BlueNRG_Custom_Services(void)
{
  int ret;

  ret = Add_HW_SW_ServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
  //  STLBLE_PRINTF("HW      Service W2ST added successfully\r\n");
  }
  else
  {
  //  STLBLE_PRINTF("\r\nError while adding HW Service W2ST\r\n");
  }

}

/**
//static void InitTimers(void)
//{
//  uint32_t uwPrescalerValue;
//
///* Compute the prescaler value to have TIM2? counter clock equal to 10 KHz */
//    //uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);
//    uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2000) - 1);
//
//    /* Set TIM4 instance */
//    TimHandleM.Instance = TIM4;
//    //TimShotHandle.Init.Period = 65535;
//    TimHandleM.Init.Period = 2*DATA_PERIOD_MS - 1;
//    TimHandleM.Init.Prescaler = uwPrescalerValue;
//    TimHandleM.Init.ClockDivision = 0;
//    TimHandleM.Init.CounterMode = TIM_COUNTERMODE_UP;
//    if(HAL_TIM_Base_Init(&TimHandleM) != HAL_OK) {
//      /* Initialization Error */
//      Error_Handler();
//    }
//}

///**
//  * @brief  Period elapsed callback in non blocking mode
//  * @param  htim : TIM handle
//  * @retval None
//  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//
//  if(htim == (&TimHandleM)){
//	//correr algoritmo y guardar datos
//	  sprintf(data, "T:%d\r\n", HAL_GetTick());
//	  if(SendOverUSB){
//	  		CDC_Fill_Buffer(( uint8_t * )data, strlen( data ));
//	  	}
//  }
//
//}



/**
* @brief  This function is executed in case of error occurrence
* @param  None
* @retval None
*/
static void Error_Handler( void )
{

  while (1)
  {}
}
//
//  void	MemManage_Handler	(void){
//while(1); }
//
//  void	BusFault_Handler	(void){
//while(1); }
//
//	void	UsageFault_Handler	(void){
//while(1); }
//
//	void	SVC_Handler	(void){
//while(1); }
//
//
//
//	void	PendSV_Handler	(void){
//while(1); }
//
//
//
//	void	WWDG_IRQHandler	(void){
//while(1); }
//
//	void	PVD_PVM_IRQHandler	(void){
//while(1); }
//
//	void	TAMP_STAMP_IRQHandler	(void){
//while(1); }
//
//	void	RTC_WKUP_IRQHandler	(void){
//while(1); }
//
//	void	FLASH_IRQHandler	(void){
//while(1); }
//
//	void	RCC_IRQHandler	(void){
//while(1); }
//
//	void	EXTI0_IRQHandler	(void){
//while(1); }
//
//	void	EXTI1_IRQHandler	(void){
//while(1); }
//
////	void	EXTI2_IRQHandler	(void){
////while(1); }
//
//	void	EXTI3_IRQHandler	(void){
//while(1); }
//
//	void	EXTI4_IRQHandler	(void){
//while(1); }
//
//	void	DMA1_Channel1_IRQHandler	(void){
//while(1); }
//
//	void	DMA1_Channel2_IRQHandler	(void){
//while(1); }
//
//	void	DMA1_Channel3_IRQHandler	(void){
//while(1); }
//
//	void	DMA1_Channel4_IRQHandler	(void){
//while(1); }
//
//	void	DMA1_Channel5_IRQHandler	(void){
//while(1); }
//
//	void	DMA1_Channel6_IRQHandler	(void){
//while(1); }
//
//	void	DMA1_Channel7_IRQHandler	(void){
//while(1); }
//
//	void	ADC1_2_IRQHandler	(void){
//while(1); }
//
//	void	CAN1_TX_IRQHandler	(void){
//while(1); }
//
//	void	CAN1_RX0_IRQHandler	(void){
//while(1); }
//
//	void	CAN1_RX1_IRQHandler	(void){
//while(1); }
//
//	void	CAN1_SCE_IRQHandler	(void){
//while(1); }
//
//	void	EXTI9_5_IRQHandler	(void){
//while(1); }
//
//	void	TIM1_BRK_TIM15_IRQHandler	(void){
//while(1); }
//
//	void	TIM1_UP_TIM16_IRQHandler	(void){
//while(1); }
//
//	void	TIM1_TRG_COM_TIM17_IRQHandler	(void){
//while(1); }
//
//	void	TIM1_CC_IRQHandler	(void){
//while(1); }
//
//	void	TIM2_IRQHandler	(void){
//while(1); }
//
////	void	TIM3_IRQHandler	(void){
////while(1); }
//
////	void	TIM4_IRQHandler	(void){
////while(1); }
//
//	void	I2C1_EV_IRQHandler	(void){
//while(1); }
//
//	void	I2C1_ER_IRQHandler	(void){
//while(1); }
//
//	void	I2C2_EV_IRQHandler	(void){
//while(1); }
//
//	void	I2C2_ER_IRQHandler	(void){
//while(1); }
//
//	void	SPI1_IRQHandler	(void){
//while(1); }
//
//	void	SPI2_IRQHandler	(void){
//while(1); }
//
//	void	USART1_IRQHandler	(void){
//while(1); }
//
//	void	USART2_IRQHandler	(void){
//while(1); }
//
//	void	USART3_IRQHandler	(void){
//while(1); }
//
//	void	EXTI15_10_IRQHandler	(void){
//while(1); }
//
//	void	RTC_Alarm_IRQHandler	(void){
//while(1); }
//
//	void	DFSDM1_FLT3_IRQHandler	(void){
//while(1); }
//
//	void	TIM8_BRK_IRQHandler	(void){
//while(1); }
//
//	void	TIM8_UP_IRQHandler	(void){
//while(1); }
//
//	void	TIM8_TRG_COM_IRQHandler	(void){
//while(1); }
//
//	void	TIM8_CC_IRQHandler	(void){
//while(1); }
//
//	void	ADC3_IRQHandler	(void){
//while(1); }
//
//	void	FMC_IRQHandler	(void){
//while(1); }
//
//	void	SDMMC1_IRQHandler	(void){
//while(1); }
//
//	void	TIM5_IRQHandler	(void){
//while(1); }
//
//	void	SPI3_IRQHandler	(void){
//while(1); }
//
//	void	UART4_IRQHandler	(void){
//while(1); }
//
//	void	UART5_IRQHandler	(void){
//while(1); }
//
//	void	TIM6_DAC_IRQHandler	(void){
//while(1); }
//
//	void	TIM7_IRQHandler	(void){
//while(1); }
//
//	void	DMA2_Channel1_IRQHandler	(void){
//while(1); }
//
//	void	DMA2_Channel2_IRQHandler	(void){
//while(1); }
//
//	void	DMA2_Channel3_IRQHandler	(void){
//while(1); }
//
//	void	DMA2_Channel4_IRQHandler	(void){
//while(1); }
//
//	void	DMA2_Channel5_IRQHandler	(void){
//while(1); }
//
//	void	DFSDM1_FLT0_IRQHandler	(void){
//while(1); }
//
//	void	DFSDM1_FLT1_IRQHandler	(void){
//while(1); }
//
//	void	DFSDM1_FLT2_IRQHandler	(void){
//while(1); }
//
//	void	COMP_IRQHandler	(void){
//while(1); }
//
//	void	LPTIM1_IRQHandler	(void){
//while(1); }
//
//	void	LPTIM2_IRQHandler	(void){
//while(1); }
//
////	void	OTG_FS_IRQHandler	(void){
////while(1); }
//
//	void	DMA2_Channel6_IRQHandler	(void){
//while(1); }
//
//	void	DMA2_Channel7_IRQHandler	(void){
//while(1); }
//
//	void	LPUART1_IRQHandler	(void){
//while(1); }
//
//	void	QUADSPI_IRQHandler	(void){
//while(1); }
//
//	void	I2C3_EV_IRQHandler	(void){
//while(1); }
//
//	void	I2C3_ER_IRQHandler	(void){
//while(1); }
//
//	void	SAI1_IRQHandler	(void){
//while(1); }
//
//	void	SAI2_IRQHandler	(void){
//while(1); }
//
//	void	SWPMI1_IRQHandler	(void){
//while(1); }
//
//	void	TSC_IRQHandler	(void){
//while(1); }
//
//	void	LCD_IRQHandler	(void){
//while(1); }
//
//	void	RNG_IRQHandler	(void){
//while(1); }
//
//	void	FPU_IRQHandler	(void){
//while(1); }


#ifdef  USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*   where the assert_param error has occurred
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed( uint8_t *file, uint32_t line )
{
  
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  while (1)
  {}
}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
