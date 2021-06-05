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
#include "SensorTile2.h"
#include "SensorTile_bus.h"
#include "SensorTile_motion_sensors.h"
#include "SensorTile_motion_sensors_ex.h"

#include "datalog_application.h"
extern TIM_HandleTypeDef TimSampleHandle;
volatile uint16_t sampleRate = 100;  //milliseconds
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

extern void *MotionCompObj[MOTION_INSTANCES_NBR];

//#define NOT_DEBUGGING

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* SendOverUSB = 0  --> Save sensors data on SDCard (enable with double click) */
/* SendOverUSB = 1  --> Send sensors data via USB */
uint8_t SendOverUSB = 1;
uint8_t enabledUSBCDC = 1;
uint8_t SaveSD = 0;   //sin buffer, directo en SD
uint8_t SaveRAMSD = 0;  //guardar en SD previo buffer en RAM
uint8_t bleData = 0;
uint8_t enabledBufferRam = 0;
uint8_t debugBufferRam = 1;
//uint8_t packetData = 1;
uint8_t timerEnabled = 0;

TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;

USBD_HandleTypeDef  USBD_Device;
static volatile uint8_t MEMSInterrupt = 0;
//static volatile uint8_t acquire_data_enable_request  = 1;
//static volatile uint8_t acquire_data_disable_request = 0;
static volatile uint8_t no_H_HTS221 = 0;
static volatile uint8_t no_T_HTS221 = 0;
//static volatile uint8_t no_GG = 0;

static RTC_HandleTypeDef RtcHandle;
static void *LSM6DSM_X_0_handle = NULL;
static void *LSM6DSM_G_0_handle = NULL;
static void *LSM303AGR_X_0_handle = NULL;
static void *LSM303AGR_M_0_handle = NULL;
static void *LPS22HB_P_0_handle = NULL;
static void *LPS22HB_T_0_handle = NULL;
static void *HTS221_H_0_handle = NULL;
static void *HTS221_T_0_handle = NULL;
//static void *GG_handle = NULL;

static char data[250];
static char dataAux[100];
static uint8_t dataAuxBuff[20];
static char pingDataBlock[DATA_BLOCK_LENGTH];
static char pongDataBlock[DATA_BLOCK_LENGTH];

volatile uint8_t saveSamplesSD = 0;
volatile uint8_t pingPongSampleBlock = 0;
volatile uint16_t countSamples = 0;
volatile uint16_t samplesToSave = 0;

volatile uint16_t lenStrPing = 0;
volatile uint16_t lenStrPong = 0;

//static char uSDSpeedBuff[1001];
static char bufferRAM[SIZE_BUFFER_RAM + 100];
//static uint32_t bufferParameters[3000][7];
static uint32_t i;
static uint32_t countBuffer = 0;
static uint32_t bufft = 0;
static uint32_t msToSaveBuff = 0;
static uint32_t msPrevBuff = 0;
static uint32_t	msInit = 0;
static uint32_t msEnd = 0;

/* Private function prototypes -----------------------------------------------*/

static void Error_Handler( void );
static void RTC_Config( void );
static void RTC_TimeStampConfig( void );
static void InitTimers(void);
void ChangeShotSampleRate(void);
static void initializeAllSensors( void );
static void configSensors(void);
static void disableUnnecessarySensors( void );
static void MX_GPIO_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
void pwmStart(void);
void changePWM(uint8_t pwmOp, uint8_t pwmValue);
void collectDataIMU(void);
static void setParametersFiltersSensorsLsm6dsm();
static void getParametersFiltersSensorsLsm6dsm();
static void setParametersFiltersSensorsLsm303agr();
static void getParametersFiltersSensorsLsm303agr();

//cantidad maxima de bytes por trama sin magnetometro 7,6,6,6,7,7,7,2 = 48
//para sectores de 512 bytes => 10 tramas
/* Private functions ---------------------------------------------------------*/
 
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main( void )
{
  uint32_t msTick, msTickPrev = 0;
  uint32_t msTickPrevTotal;
  uint8_t doubleTap = 0;
  uint8_t lenStr = 0;

  /* STM32L4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
  */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initialize LED */
  //BSP_LED_Init(LED1);
  //BSP_LED_On(LED1);
  
  /* Initialize RTC */
  RTC_Config();
  RTC_TimeStampConfig();
  
  /* enable USB power on Pwrctrl CR2 register */
  HAL_PWREx_EnableVddUSB();
  
  if(SendOverUSB || enabledUSBCDC) /* Configure the USB */
  {
    /*** USB CDC Configuration ***/
    /* Init Device Library */
    USBD_Init(&USBD_Device, &VCP_Desc, 0);
    /* Add Supported Class */
    USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
    /* Add Interface callbacks for AUDIO and CDC Class */
    USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
    /* Start Device Process */
    USBD_Start(&USBD_Device);
  }

  if(SaveSD){
	  Timer_Sample_Itf_Init();
	  DATALOG_SD_Init();
  }

  if(SaveSD || SaveRAMSD){
	  SD_Log_Enabled = 1;
  	  HAL_Delay(200);
  }
  
  /* Configure and disable all the Chip Select pins */
  Sensor_IO_SPI_CS_Init_All();
  
  /* Initialize and Enable the available sensors */
  initializeAllSensors();
  enableAllSensors();
  
  disableUnnecessarySensors();

  configSensors();

  getParametersFiltersSensorsLsm6dsm();
  //setParametersFiltersSensorsLsm6dsm();
  //getParametersFiltersSensorsLsm6dsm();


  //MX_MEMS_Init(); //initialize fifo mode

  //**************************init_PWM configuration****************
  //Hay 2 leds rojo y verde, se va a usar el TIM15 que tiene 2 salidas. PG10 y PG11
  //PG10 SAI_FS . Pin 9 sensortile
  //PG11 SAI_MCLK . Pin 10 sensortile

  if(!SaveSD && !SaveRAMSD){
  MX_GPIO_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();

  pwmStart();
  //valores de PWM de 0 - 199
  //50 => 25 %
  //99 => 50 %
  //150 => 75 %

  changePWM(1, 99);
  changePWM(2, 150);
  changePWM(3, 50);
  }

  memset (pingDataBlock,0,DATA_BLOCK_LENGTH);
  memset (pongDataBlock,0,DATA_BLOCK_LENGTH);

  //*************************test escritura tarjeta SD**************
//  SD_Log_Enabled = 1;
//
//  uSDWriteSpeedOpenFile();
//
//  DATALOG_SD_NewLine();
//  msTickPrev = HAL_GetTick();
//
//  for(i = 0; i < 1000; i++){
//	  uSDSpeedBuff[i] = 'a';
//  }
//  uSDSpeedBuff[1001] = '\0';
//
//  uSdWriteSpeedTest(uSDSpeedBuff);
//  msTick = HAL_GetTick();
//  DATALOG_SD_Log_Disable();
//
//  if(SendOverUSB) /* Write data on the USB */
//      {
//        sprintf( data, "\n\rWritten bytes: %d time: %d", 1000, msTick - msTickPrev);
//        CDC_Fill_Buffer(( uint8_t * )data, strlen( data ));
//      }
//
//
//  while(1){
//
//  }
//
//***********************fin test escritura************************

  if(SD_Log_Enabled){
	  DATALOG_SD_Log_Enable();
	  HAL_Delay(500);
  }

  //*************init timer sample************************************
  if(timerEnabled == 1){
	  if(HAL_TIM_Base_Start_IT(&TimSampleHandle) != HAL_OK)
	  {
		  /* Starting Error */
		  Error_Handler();
	  }
  }
  //*****************************************************************

  msTickPrevTotal = msTick;
  while (1)
  {

	if(saveSamplesSD){
		if(pingPongSampleBlock == 0){
			if(SendOverUSB){
				CDC_Fill_Buffer(( uint8_t * )pongDataBlock, lenStrPong);
			}
			if(SD_Log_Enabled) /* Write data to the file on the SDCard */
			{
			 //DATALOG_SD_NewLine();
			 saveToSD(pongDataBlock);   //save .csv format
			}
			lenStrPong = 0;
		}
		if(pingPongSampleBlock == 1){
			if(SendOverUSB){
				CDC_Fill_Buffer(( uint8_t * )pingDataBlock, lenStrPing);
			}
			 if(SD_Log_Enabled) /* Write data to the file on the SDCard */
			{
			 //DATALOG_SD_NewLine();
			  saveToSD(pingDataBlock);   //save .csv format
			}
			lenStrPing = 0;
		}
		saveSamplesSD = 0;
	}

    /* Get sysTick value and check if it's time to execute the task */
    msTick = HAL_GetTick();
    if(((msTick % DATA_PERIOD_MS) == 0) && (msTickPrev != msTick) && (timerEnabled == 0))
    //if(((msTick - msTickPrev) >= DATA_PERIOD_MS) && (msTickPrev != msTick))
    {
      msTickPrev = msTick;

      //BSP_LED_On(LED1);

      lenStr = 0;
      SysTick->VAL;
      msInit = HAL_RCC_GetSysClockFreq() / 1000000;
      RTC_Handler( &RtcHandle, dataAux);
      sprintf(data, "%s", dataAux);
      lenStr += strlen(dataAux);
      sprintf(data + lenStr, ",");
      lenStr++;

      Accelero_Sensor_Handler(LSM6DSM_X_0_handle, dataAux);
      //Accelero_Sensor_Handler(LSM303AGR_X_0_handle, dataAux);
      sprintf(data + lenStr, "%s", dataAux);
      lenStr += strlen(dataAux);
      sprintf(data + lenStr, ",");
      lenStr++;

      Gyro_Sensor_Handler( LSM6DSM_G_0_handle, dataAux);
      sprintf(data + lenStr, "%s", dataAux);
      lenStr += strlen(dataAux);
      sprintf(data + lenStr, ",");
      lenStr++;
      
      Magneto_Sensor_Handler( LSM303AGR_M_0_handle, dataAux);
      sprintf(data + lenStr, "%s", dataAux);
      lenStr += strlen(dataAux);
      sprintf(data + lenStr, ",");
      lenStr++;

      //sprintf(data + lenStr, ";");
      
      sprintf(data + lenStr, "\r\n");
      lenStr = lenStr + 2;
      msEnd = HAL_RCC_GetSysClockFreq() / 1000000;

      if(SendOverUSB){
    	  CDC_Fill_Buffer(( uint8_t * )data, strlen( data ));
      }

      if(SD_Log_Enabled) /* Write data to the file on the SDCard */
      {
        //DATALOG_SD_NewLine();
        saveToSD(data);   //save .csv format
      }

      if(enabledBufferRam){
    	  if(countBuffer == 0){
    		  msPrevBuff = HAL_GetTick();
    		  sprintf(bufferRAM, "%s", data);
    		  //CDC_Fill_Buffer(( uint8_t * )bufferRAM, strlen( bufferRAM ));
    	  }
    	  countBuffer = countBuffer + lenStr;
    	  if(countBuffer >= SIZE_BUFFER_RAM){
    		  countBuffer = 0;
    		  if(debugBufferRam){
    			  bufft = strlen( bufferRAM );
    			  //CDC_Fill_Buffer(( uint8_t * )bufferRAM, SIZE_BUFFER_RAM);
    			  //CDC_Fill_Buffer(( uint8_t * )bufferRAM, strlen( bufferRAM ));
    			  msToSaveBuff = HAL_GetTick() - msPrevBuff;
    		  }
    		  if(SD_Log_Enabled){
    			 saveToSD(bufferRAM);
    			 DATALOG_SD_Log_Disable();
    			 HAL_Delay(500);
    			 DATALOG_SD_Log_Enable();
    			 HAL_Delay(500);
    			 msToSaveBuff = HAL_GetTick() - msPrevBuff;
    		  }
    	  }
    	  else{
    		  sprintf(bufferRAM + countBuffer, "%s", data);
    		  //CDC_Fill_Buffer(( uint8_t * )data, strlen( data ));
    		  //CDC_Fill_Buffer(( uint8_t * )bufferRAM, strlen( bufferRAM ));
    	  }
      }

      //BSP_LED_Off(LED1);
      //BSP_LED_On(LED1);

    }

    if((msTick - msTickPrevTotal) > DATA_PERIOD_CLOSE_FILE){
  	  msTickPrevTotal = msTick;
  	  if(SD_Log_Enabled && SaveSD){
  		  DATALOG_SD_Log_Disable();
  		  HAL_Delay(500);
  		  DATALOG_SD_Log_Enable();
  		  HAL_Delay(500);
  	  }
    }
      
    /* Check LSM6DSM Double Tap Event  */
    if(MEMSInterrupt)
    {
      MEMSInterrupt = 0;
      BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext(LSM6DSM_X_0_handle,&doubleTap);
      if(doubleTap) { /* Double Tap event */
        if (SD_Log_Enabled) 
        {
          DATALOG_SD_Log_Disable();
          SD_Log_Enabled=0;
        }
        else
        {
          while(SD_Log_Enabled != 1)
          {
            if(DATALOG_SD_Log_Enable())
            {
              SD_Log_Enabled=1;
            }
            else
            {
              DATALOG_SD_Log_Disable();
            }
            HAL_Delay(100);
          }
        }
      }
    }
    
    /* Go to Sleep */
    __WFI();
  }
}



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
  
  /* Inialize the Gas Gauge if the battery is present */
//  if(BSP_GG_Init(&GG_handle) == COMPONENT_ERROR)
//  {
//    no_GG=1;
//  }
//
  /*if(SaveSD)
  {
     //Enable HW Double Tap detection
    BSP_ACCELERO_Enable_Double_Tap_Detection_Ext(LSM6DSM_X_0_handle);
    BSP_ACCELERO_Set_Tap_Threshold_Ext(LSM6DSM_X_0_handle, LSM6DSM_TAP_THRESHOLD_MID);
  }*/
  
  
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
  BSP_PRESSURE_Sensor_Disable( LPS22HB_P_0_handle );
}

void configSensors(void){


	BSP_ACCELERO_Set_FS_Value(LSM6DSM_X_0_handle, 4.0f); // 2.0f, 4.0f, 8.0f, 16.0f
	BSP_ACCELERO_Set_ODR_Value(LSM6DSM_X_0_handle, 6660.0f); // 13.0f, 26.0f, 52.0f, 104.0f, 208.0f,
	  	  	  	  	  	  	  	  	  	  	  	  	  	  	 //	416.0f, 833.0f, 1660.0f, 3330.0f, 6660.0f
	BSP_GYRO_Set_FS_Value(LSM6DSM_G_0_handle, 2000.0f);	// 245.0f, 500.0f, 1000.0f, 2000.0f
	BSP_GYRO_Set_ODR_Value(LSM6DSM_G_0_handle, 6660.0f);	// 13.0f, 26.0f, 52.0f, 104.0f, 208.0f,
	 	  	  	  	  	 	 	 	 	 	 	 	 	//	416.0f, 833.0f, 1660.0f, 3330.0f, 6660.0f

	BSP_ACCELERO_Set_FS_Value(LSM303AGR_X_0_handle, 4.0f);	// 2.0f, 4.0f, 8.0f, 16.0f
	BSP_ACCELERO_Set_ODR_Value(LSM303AGR_X_0_handle, 5376.0f);	// 13.0f, 26.0f, 52.0f, 104.0f, 208.0f,
		 	  	  	  	  	 	 	 	 	 	 	 	 	//	416.0f, 833.0f, 1660.0f, 3330.0f, 6660.0f

	BSP_MAGNETO_Set_FS_Value(LSM303AGR_M_0_handle, 50.0f);
	BSP_MAGNETO_Set_ODR_Value(LSM303AGR_M_0_handle, 50.0f);
}


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



/**
* @brief  EXTI line detection callbacks
* @param  GPIO_Pin: Specifies the pins connected EXTI line
* @retval None
*/
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
  MEMSInterrupt=1;
}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 39;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 199;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  //htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 39;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 199;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  //htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

void pwmStart(void){

	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);

}

void changePWM(uint8_t pwmOp, uint8_t pwmValue){

	switch(pwmOp){
	case 1:
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, pwmValue);
		break;
	case 2:
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, pwmValue);
		break;
	case 3:
		__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, pwmValue);
		break;
	default:
		break;
	}

}

static void InitTimers(void)
{
  uint32_t uwPrescalerValue;

  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;

  /* Compute the prescaler value to have TIM2? counter clock equal to 10 KHz */
    //uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);
    uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2000) - 1);

    /* Set TIM4 instance */
    TimSampleHandle.Instance = TIM4;
    //TimShotHandle.Init.Period = 65535;
    TimSampleHandle.Init.Period = 2*sampleRate - 1;
    TimSampleHandle.Init.Prescaler = uwPrescalerValue;
    TimSampleHandle.Init.ClockDivision = 0;
    TimSampleHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    if(HAL_TIM_Base_Init(&TimSampleHandle) != HAL_OK) {
      /* Initialization Error */
      Error_Handler();
    }

}

void ChangeShotSampleRate(void){
	uint32_t uwPrescalerValue;

    //*********************************************************************************
	/* Compute the prescaler value to have TIM2? counter clock equal to 10 KHz */
	//uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);
	uwPrescalerValue = (uint32_t) ((SystemCoreClock / 2000) - 1);

	/* Set TIM4 instance */
	TimSampleHandle.Instance = TIM4;
	//TimShotHandle.Init.Period = 65535;
	TimSampleHandle.Init.Period = 2*sampleRate - 1;
	TimSampleHandle.Init.Prescaler = uwPrescalerValue;
	TimSampleHandle.Init.ClockDivision = 0;
	TimSampleHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	if(HAL_TIM_Base_Init(&TimSampleHandle) != HAL_OK) {
	  /* Initialization Error */
	  Error_Handler();
	}
	//********************************************************************************

}

///**
//  * @brief  Period elapsed callback in non blocking mode for Environmental timer
//  * @param  htim : TIM handle
//  * @retval None
//  */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//if(htim == (&TimShotHandle)){
//	  shotAlgorithm_simplified();
//	  //ALLMEMS1_PRINTF("T:%d\r\n", HAL_GetTick());
//  }
//}

void collectDataIMU(void){
	uint16_t size;
	static uint16_t countSamples = 0;
	SensorAxes_t acceleration;

	if(timerEnabled == 1){
		if(countSamples >= SAMPLES_PER_BLOCK){
			if(pingPongSampleBlock == 0){
				pingPongSampleBlock = 1;
				lenStrPong = 0;
			}
			else{
				pingPongSampleBlock = 0;
				lenStrPing = 0;
			}
			samplesToSave = countSamples;
			saveSamplesSD = 1;
			countSamples = 0;
		}
			if(pingPongSampleBlock == 0){
				RTC_Handler( &RtcHandle, dataAux);
				size = sprintf(pingDataBlock + lenStrPing, "%s,", dataAux);
				lenStrPing += size;

				Accelero_Sensor_Handler(LSM6DSM_X_0_handle, dataAux);
				//Accelero_Sensor_Handler(LSM303AGR_X_0_handle, dataAux);
				size = sprintf(pingDataBlock + lenStrPing, "%s,", dataAux);
				lenStrPing += size;

//				if ( BSP_ACCELERO_Get_Axes( LSM303AGR_X_0_handle, &acceleration ) == COMPONENT_ERROR )
//				{
//				    acceleration.AXIS_X = 0;
//				    acceleration.AXIS_Y = 0;
//				    acceleration.AXIS_Z = 0;
//				 }
//				size = sprintf(pingDataBlock + lenStrPing, "%s,", dataAux);
//				lenStrPing += size;

				Gyro_Sensor_Handler( LSM6DSM_G_0_handle, dataAux);
				size = sprintf(pingDataBlock + lenStrPing, "%s,\r\n", dataAux);
				lenStrPing += size;
				countSamples++;

//				//********************guardado por bytes*************
//				memcpy(pingDataBlock + lenStrPing , HAL_GetTick(), 4);
//				lenStrPing += 4;
//
//				Accelero_Sensor_Handler_bytes(LSM6DSM_X_0_handle, dataAuxBuff);
//				memcpy(pingDataBlock + lenStrPing, dataAuxBuff, 12);
//				lenStrPing += 12;
//
//				Gyro_Sensor_Handler_bytes( LSM6DSM_G_0_handle, dataAuxBuff);
//				memcpy(pingDataBlock + lenStrPing, dataAuxBuff, 12);
//				lenStrPing += 12;
//
//				countSamples++;
//				//***************************************************

			}
			else{
				RTC_Handler( &RtcHandle, dataAux);
				size = sprintf(pongDataBlock + lenStrPong, "%s,", dataAux);
				lenStrPong += size;

				Accelero_Sensor_Handler(LSM6DSM_X_0_handle, dataAux);
				size = sprintf(pongDataBlock + lenStrPong, "%s,", dataAux);
				lenStrPong += size;

				Gyro_Sensor_Handler( LSM6DSM_G_0_handle, dataAux);
				size = sprintf(pongDataBlock + lenStrPong, "%s,\r\n", dataAux);
				lenStrPong += size;
				countSamples++;

//				//********************guardado por bytes*************
//				memcpy(pongDataBlock + lenStrPong , HAL_GetTick(), 4);
//				lenStrPong += 4;
//
//				Accelero_Sensor_Handler_bytes(LSM6DSM_X_0_handle, dataAuxBuff);
//				memcpy(pongDataBlock + lenStrPong, dataAuxBuff, 12);
//				lenStrPong += 12;
//
//				Gyro_Sensor_Handler_bytes( LSM6DSM_G_0_handle, dataAuxBuff);
//				memcpy(pongDataBlock + lenStrPong, dataAuxBuff, 12);
//				lenStrPong += 12;
//
//				countSamples++;
//				//***************************************************

			}
	}
}

void setParametersFiltersSensorsLsm303agr(void){
	LSM303AGR_ACC_Object_t *pAcc;
	LSM303AGR_MAG_Object_t *pMag;
	uint8_t v1;
	lsm303agr_hpcf_a_t v2;
	lsm303agr_hpm_a_t v3;
	uint8_t v4;
	lsm303agr_hp_a_t v5;
	uint8_t v6;
	lsm303agr_lpf_m_t v7;

	v1 = 0x00;
	v2 = LSM303AGR_AGGRESSIVE; // LSM303AGR_AGGRESSIVE LSM303AGR_AGGRESSIVE LSM303AGR_MEDIUM LSM303AGR_LIGHT
	v3 = LSM303AGR_LIGHT; // LSM303AGR_NORMAL_WITH_RST LSM303AGR_NORMAL_WITH_RST LSM303AGR_NORMAL LSM303AGR_AUTORST_ON_INT
	v4 = 0x00;
	v5 = LSM303AGR_DISC_FROM_INT_GENERATOR;
	v6 = 0x00;
	v7 = LSM303AGR_ODR_DIV_2; // LSM303AGR_ODR_DIV_2 LSM303AGR_ODR_DIV_4

	/*
	lsm303agr_xl_high_pass_on_outputs_set();
	lsm303agr_xl_high_pass_bandwidth_set();
	lsm303agr_xl_high_pass_mode_set();
	lsm303agr_xl_filter_reference_set();
	lsm303agr_xl_high_pass_int_conf_set();
	lsm303agr_xl_high_pass_on_outputs_set();
	lsm303agr_mag_low_pass_bandwidth_set();
	*/

}

void getParametersFiltersSensorsLsm303agr(void){
	char bufferFilters[500];
	LSM303AGR_ACC_Object_t *pAcc;
	LSM303AGR_MAG_Object_t *pMag;
	uint32_t size = 0;
	uint32_t totalSize = 0;

	uint8_t v1;
	lsm303agr_hpcf_a_t v2;
	lsm303agr_hpm_a_t v3;
	uint8_t v4;
	lsm303agr_hpm_a_t v5;
	uint8_t v6;
	lsm303agr_lpf_m_t v7;

	size = sprintf(bufferFilters, "Acc initilized: %d\r\n", pAcc->is_initialized);
	totalSize += size;
	size = sprintf(bufferFilters + totalSize, "Acc initilized: %d\r\n", pAcc->acc_is_enabled);
	totalSize += size;
	size = sprintf(bufferFilters + totalSize, "Acc ODR: %d\r\n", pAcc->acc_odr);
	totalSize += size;

	size = sprintf(bufferFilters + totalSize, "Mag initilized: %d\r\n", pMag->is_initialized);
	totalSize += size;
	size = sprintf(bufferFilters + totalSize, "Acc initilized: %d\r\n", pMag->mag_is_enabled);
	totalSize += size;

	/*
	lsm303agr_xl_high_pass_on_outputs_get(&(pAcc->Ctx), &v1);
	size = sprintf(bufferFilters + totalSize, "Acc high pass bandwidth output: %d\r\n", v1);
	totalSize += size;

	lsm303agr_xl_high_pass_bandwidth_get(&(pAcc->Ctx), &v2);
	size = sprintf(bufferFilters + totalSize, "Acc high pass bandwidth: %d\r\n", v2);
	totalSize += size;

	lsm303agr_xl_high_pass_mode_get(&(pAcc->Ctx), &v3);
	size = sprintf(bufferFilters + totalSize, "Acc high pass mode: %d\r\n", v3);
	totalSize += size;

	lsm303agr_xl_filter_reference_get(&(pAcc->Ctx), &v4);
	size = sprintf(bufferFilters + totalSize, "Acc filter reference: %d\r\n", v4);
	totalSize += size;

	lsm303agr_xl_high_pass_int_conf_get(&(pAcc->Ctx), &v5);
	size = sprintf(bufferFilters + totalSize, "Acc high pass int: %d\r\n", v5);
	totalSize += size;

	lsm303agr_xl_high_pass_on_outputs_get(&(pAcc->Ctx), &v6);
	size = sprintf(bufferFilters + totalSize, "Acc high pass output: %d\r\n", v6);
	totalSize += size;

	lsm303agr_mag_low_pass_bandwidth_get(&(pMag->Ctx), &v7);
	size = sprintf(bufferFilters + totalSize, "Mag low pass bdw: %d\r\n", v7);
	totalSize += size;
	*/

	CDC_Fill_Buffer(( uint8_t * )bufferFilters, totalSize);
}

void setParametersFiltersSensorsLsm6dsm(void){
	char bufferFilters[500];
	LSM6DSM_Object_t *pImu;
	uint8_t settlingMask;
	lsm6dsm_slope_fds_t v1;
	lsm6dsm_bw0_xl_t v2;
	lsm6dsm_lpf1_bw_sel_t v3;
	uint8_t v4;
	lsm6dsm_lpf1_bw_sel_t v5;
	lsm6dsm_input_composite_t v6;
	lsm6dsm_hpcf_xl_t v7;
	lsm6dsm_lpf1_sel_g_t v8;
	int32_t response;
	uint32_t size;
	uint32_t totalSize;

	settlingMask = 0x00;
	v1 = LSM6DSM_USE_SLOPE; // LSM6DSM_USE_SLOPE LSM6DSM_USE_HPF LSM6DSM_HP_PATH_ND
	v2 = LSM6DSM_XL_ANA_BW_400Hz; // LSM6DSM_XL_ANA_BW_1k5Hz LSM6DSM_XL_ANA_BW_400Hz LSM6DSM_XL_ANA_BW_ND
	v3 = LSM6DSM_XL_LP1_ODR_DIV_2; // LSM6DSM_XL_LP1_ODR_DIV_2 LSM6DSM_XL_LP1_ODR_DIV_4 LSM6DSM_XL_LP1_NA
	v4 = 0x00;
	v5 = LSM6DSM_XL_LP1_ODR_DIV_2; // LSM6DSM_XL_LP1_ODR_DIV_2 LS LSM6DSM_XL_LP1_NA
	v6 = LSM6DSM_XL_LOW_NOISE_LP_ODR_DIV_50;
	v7 = LSM6DSM_XL_HP_ODR_DIV_4;
	v8 = LSM6DSM_LP2_ONLY;

	pImu = MotionCompObj[LSM6DSM_0];

	lsm6dsm_filter_settling_mask_set(&(pImu->Ctx), settlingMask);
	//lsm6dsm_xl_hp_path_internal_set(&(pImu->Ctx), v1);
	lsm6dsm_xl_filter_analog_set(&(pImu->Ctx), v2);
	lsm6dsm_xl_lp1_bandwidth_set(&(pImu->Ctx), v3);
	lsm6dsm_xl_reference_mode_set(&(pImu->Ctx), v4);
	lsm6dsm_xl_lp1_bandwidth_set(&(pImu->Ctx), v5);
	lsm6dsm_xl_lp2_bandwidth_set(&(pImu->Ctx), v6);
	//response = lsm6dsm_xl_hp_bandwidth_set(&(pImu->Ctx), v7);
	lsm6dsm_gy_band_pass_set(&(pImu->Ctx), v8);

	memset(bufferFilters, 0, 200);

	totalSize = 0;
	size = sprintf(bufferFilters + totalSize, "Set hp bandwith: %d\r\n", response);
	totalSize += size;

	CDC_Fill_Buffer(( uint8_t * )bufferFilters, totalSize);

}

void getParametersFiltersSensorsLsm6dsm(void){
	char bufferFilters[500];
	LSM6DSM_Object_t *pImu;
	uint32_t size = 0;
	uint32_t totalSize = 0;
	uint8_t settlingMask;
	lsm6dsm_slope_fds_t v1;
	lsm6dsm_bw0_xl_t v2;
	lsm6dsm_lpf1_bw_sel_t v3;
	uint8_t v4;
	lsm6dsm_lpf1_bw_sel_t v5;
	lsm6dsm_input_composite_t v6;
	lsm6dsm_hpcf_xl_t v7;
	lsm6dsm_lpf1_sel_g_t v8;

	memset(bufferFilters, 0, 200);

	pImu = MotionCompObj[LSM6DSM_0];

	size = sprintf(bufferFilters, "Sensor initilized: %d\r\n", pImu->is_initialized);
	totalSize += size;
	size = sprintf(bufferFilters + totalSize, "Acc initilized: %d\r\n", pImu->acc_is_enabled);
	totalSize += size;
	size = sprintf(bufferFilters + totalSize, "Gyr initilized: %d\r\n", pImu->gyro_is_enabled);
	totalSize += size;
	size = sprintf(bufferFilters + totalSize, "Acc ODR: %d\r\n", pImu->acc_odr);
	totalSize += size;
	size = sprintf(bufferFilters + totalSize, "Gyro ODR: %d\r\n", pImu->gyro_odr);
	totalSize += size;

	lsm6dsm_filter_settling_mask_get(&(pImu->Ctx), &settlingMask);
	size = sprintf(bufferFilters + totalSize, "Filter settling mask: %d\r\n", settlingMask);
	totalSize += size;

	lsm6dsm_xl_hp_path_internal_get(&(pImu->Ctx), &v1);
	size = sprintf(bufferFilters + totalSize, "Acc hp path: %d\r\n", v1);
	totalSize += size;

	lsm6dsm_xl_filter_analog_get(&(pImu->Ctx), &v2);
	size = sprintf(bufferFilters + totalSize, "Acc filter analog: %d\r\n", v2);
	totalSize += size;

	lsm6dsm_xl_lp1_bandwidth_get(&(pImu->Ctx), &v3);
	size = sprintf(bufferFilters + totalSize, "Acc LP1 Bdw: %d\r\n", v3);
	totalSize += size;

	lsm6dsm_xl_reference_mode_get(&(pImu->Ctx), &v4);
	size = sprintf(bufferFilters + totalSize, "Acc refrence mode: %d\r\n", v4);
	totalSize += size;

	lsm6dsm_xl_lp1_bandwidth_get(&(pImu->Ctx), &v5);
	size = sprintf(bufferFilters + totalSize, "Acc LP1 Bdw: %d\r\n", v5);
	totalSize += size;

	lsm6dsm_xl_lp2_bandwidth_get(&(pImu->Ctx), &v6);
	size = sprintf(bufferFilters + totalSize, "Acc LP2 Bdw: %d\r\n", v6);
	totalSize += size;

	lsm6dsm_xl_hp_bandwidth_get(&(pImu->Ctx), &v7);
	size = sprintf(bufferFilters + totalSize, "Acc hp bdw: %d\r\n", v7);
	totalSize += size;

	lsm6dsm_gy_band_pass_get(&(pImu->Ctx), &v8);
	size = sprintf(bufferFilters + totalSize, "Gyro band pass: %d\r\n", v8);
	totalSize += size;

	CDC_Fill_Buffer(( uint8_t * )bufferFilters, totalSize);
}

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
