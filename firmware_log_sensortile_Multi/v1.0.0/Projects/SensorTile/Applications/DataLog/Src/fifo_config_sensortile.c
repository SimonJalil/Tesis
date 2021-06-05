#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "fifo_config_sensortile.h"
#include "main.h"
#include <stdio.h>

#include "stm32l4xx_hal.h"
//#include "stm32l4xx_nucleo.h"
#include "SensorTile_motion_sensors.h"
#include "SensorTile_motion_sensors_ex.h"

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief  Handle State Machine
 */
typedef enum
{
  STATUS_IDLE,
  STATUS_SET_FIFO_CONTINUOUS_MODE,
  STATUS_FIFO_RUN,
  STATUS_FIFO_DOWNLOAD,
  STATUS_SET_FIFO_BYPASS_MODE
} DEMO_FIFO_STATUS_t;

/* Private define ------------------------------------------------------------*/
#define MAX_BUF_SIZE 256
#define FIFO_WATERMARK   91 /*!< FIFO size limit */
#define SAMPLE_LIST_MAX  10U /*!< Max. number of values (X,Y,Z) to be printed to UART */

#define LSM6DSM_SAMPLE_ODR      10.0f /*!< Sample Output Data Rate [Hz] */
#define LSM6DSM_FIFO_MAX_ODR  6600.0f /*!< LSM6DSM FIFO maximum ODR */

#define ENABLE  1 /*!< Enable LSM6DSM FIFO functions */

#define INDICATION_DELAY  100 /* LED is ON for this period [ms]. */

#define PATTERN_GYR_X_AXIS  0 /*!< Pattern of gyro X axis */
#define PATTERN_GYR_Y_AXIS  1 /*!< Pattern of gyro Y axis */
#define PATTERN_GYR_Z_AXIS  2 /*!< Pattern of gyro Z axis */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t MemsEventDetected = 0;
static volatile uint8_t PushButtonDetected = 0;
static DEMO_FIFO_STATUS_t DemoFifoStatus = STATUS_SET_FIFO_BYPASS_MODE;
static char dataOut[MAX_BUF_SIZE];
static uint8_t fifo_full_status = 0;
static uint16_t num_samples = 0;
static uint16_t prev_num_samples = 0;
static int32_t PushButtonState = GPIO_PIN_RESET;

/* Private function prototypes -----------------------------------------------*/
static void Error_Handler( void );
static void MX_LSM6DSM_FIFOContinuousMode_Init(void);
static void MX_LSM6DSM_FIFOContinuousMode_Process(void);
static int32_t LSM6DSM_FIFO_Set_Bypass_Mode(void);
static int32_t LSM6DSM_FIFO_Set_Continuous_Mode(void);
static int32_t LSM6DSM_Read_All_FIFO_Data(void);
static int32_t LSM6DSM_Read_Single_FIFO_Pattern_Cycle(uint16_t SampleIndex);
static int32_t LSM6DSM_FIFO_Demo_Config(void);

void MX_MEMS_Init(void)
{
  /* USER CODE BEGIN SV */ 

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Init_PreTreatment */
  
  /* USER CODE END MEMS_Init_PreTreatment */

  /* Initialize the peripherals and the MEMS components */

  MX_LSM6DSM_FIFOContinuousMode_Init();

  /* USER CODE BEGIN MEMS_Init_PostTreatment */
  
  /* USER CODE END MEMS_Init_PostTreatment */
}
/*
 * LM background task
 */
void MX_MEMS_Process(void)
{
  /* USER CODE BEGIN MEMS_Process_PreTreatment */
  
  /* USER CODE END MEMS_Process_PreTreatment */

  MX_LSM6DSM_FIFOContinuousMode_Process();

  /* USER CODE BEGIN MEMS_Process_PostTreatment */
  
  /* USER CODE END MEMS_Process_PostTreatment */
}

/**
  * @brief  Initialize the LSM6DSM FIFO Continuous Mode application
  * @retval None
  */
void MX_LSM6DSM_FIFOContinuousMode_Init(void)
{

  (void)BSP_MOTION_SENSOR_Init(LSM6DSM_0, MOTION_ACCELERO);
  (void)BSP_MOTION_SENSOR_Init(LSM6DSM_0, MOTION_GYRO);
  //(void)BSP_MOTION_SENSOR_Init(USE_MOTION_SENSOR_LSM6DSM_0, MOTION_MAGNETO);

  /* Configure LSM6DSM sensor for the DEMO application */
  if (LSM6DSM_FIFO_Demo_Config() != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

}
/**
  * @brief  Process of the LSM6DSM FIFO Continuous Mode application
  * @retval None
  */
void MX_LSM6DSM_FIFOContinuousMode_Process(void)
{
  if (PushButtonDetected != 0U)
  {
    /* Debouncing */
    //HAL_Delay(50);

    ///* Wait until the button is released */
    //while ((BSP_PB_GetState( BUTTON_KEY ) == PushButtonState));

    ///* Debouncing */
    //HAL_Delay(50);

    ///* Reset Interrupt flag */
    //PushButtonDetected = 0;

    switch (DemoFifoStatus)
    {
      /* If FIFO is in Bypass mode switch to Continuous mode */
      case STATUS_IDLE:
        DemoFifoStatus = STATUS_SET_FIFO_CONTINUOUS_MODE;
        break;
      /* If FIFO is in Continuous mode switch to Bypass mode */
      case STATUS_FIFO_RUN:
        DemoFifoStatus = STATUS_SET_FIFO_BYPASS_MODE;
        break;
      /* Otherwise do nothing */
      case STATUS_SET_FIFO_CONTINUOUS_MODE:
      case STATUS_FIFO_DOWNLOAD:
      case STATUS_SET_FIFO_BYPASS_MODE:
        break;
      default:
        Error_Handler();
        break;
    }
  }

  /* Handle DEMO State Machine */
  switch (DemoFifoStatus)
  {
    case STATUS_IDLE:
      break;

    case STATUS_SET_FIFO_CONTINUOUS_MODE:
      if (LSM6DSM_FIFO_Set_Continuous_Mode() != BSP_ERROR_NONE)
      {
        Error_Handler();
      }
      DemoFifoStatus = STATUS_FIFO_RUN;
      break;

    case STATUS_FIFO_RUN:
      /* Get num of unread FIFO samples before reading data */
      if (BSP_MOTION_SENSOR_FIFO_Get_Num_Samples(LSM6DSM_0, &num_samples) != BSP_ERROR_NONE)
      {
        Error_Handler();
      }

      /* Print dot realtime whenever new data is stored in FIFO */
      if (num_samples != prev_num_samples)
      {
        prev_num_samples = num_samples;
        /*
        (void)snprintf(dataOut, MAX_BUF_SIZE, ".");
        printf("%s", dataOut);
        fflush(stdout);
        */
      }
      if (MemsEventDetected == 1U)
      {
        DemoFifoStatus = STATUS_FIFO_DOWNLOAD;
        MemsEventDetected = 0;
      }
      break;

    case STATUS_FIFO_DOWNLOAD:
      /* Print data if FIFO is full */
      if (BSP_MOTION_SENSOR_FIFO_Get_Full_Status(LSM6DSM_0, &fifo_full_status) != BSP_ERROR_NONE)
      {
        Error_Handler();
      }
      if (fifo_full_status == 1U)
      {
        //BSP_LED_On(LED2);
        if (LSM6DSM_Read_All_FIFO_Data() != BSP_ERROR_NONE)
        {
          Error_Handler();
        }
        //BSP_LED_Off(LED2);
        DemoFifoStatus = STATUS_FIFO_RUN;
      }
      break;

    case STATUS_SET_FIFO_BYPASS_MODE:
      if (LSM6DSM_FIFO_Set_Bypass_Mode() != BSP_ERROR_NONE)
      {
        Error_Handler();
      }
      MemsEventDetected = 0;
      num_samples = 0;
      prev_num_samples = 0;
      DemoFifoStatus = STATUS_IDLE;
      break;

    default:
      Error_Handler();
      break;
  }
}

/**
  * @brief  Configure FIFO
  * @retval BSP status
  */
static int32_t LSM6DSM_FIFO_Demo_Config(void)
{
  int32_t ret;

  if ((ret = BSP_MOTION_SENSOR_SetOutputDataRate(LSM6DSM_0, MOTION_GYRO, LSM6DSM_SAMPLE_ODR)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  if ((ret = BSP_MOTION_SENSOR_SetOutputDataRate(LSM6DSM_0, MOTION_ACCELERO, LSM6DSM_SAMPLE_ODR)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  /* Set accelero FIFO decimation */
  if ((ret = BSP_MOTION_SENSOR_FIFO_Set_Decimation(LSM6DSM_0, MOTION_ACCELERO, (uint8_t)LSM6DSM_FIFO_XL_NO_DEC)) != BSP_ERROR_NONE)
  {
	  return ret;
  }

  /* Set gyro FIFO decimation */
  if ((ret = BSP_MOTION_SENSOR_FIFO_Set_Decimation(LSM6DSM_0, MOTION_GYRO, (uint8_t)LSM6DSM_FIFO_GY_NO_DEC)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  /* Set FIFO ODR to highest value */
  if ((ret = BSP_MOTION_SENSOR_FIFO_Set_ODR_Value(LSM6DSM_0, LSM6DSM_FIFO_MAX_ODR)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  /* Set FIFO_FULL on INT1 */
  if ((ret = BSP_MOTION_SENSOR_FIFO_Set_INT1_FIFO_Full(LSM6DSM_0, ENABLE)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  /* Set FIFO watermark */
  if ((ret = BSP_MOTION_SENSOR_FIFO_Set_Watermark_Level(LSM6DSM_0, FIFO_WATERMARK)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  /* Set FIFO depth to be limited to watermark threshold level  */
  if ((ret = BSP_MOTION_SENSOR_FIFO_Set_Stop_On_Fth(LSM6DSM_0, ENABLE)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  return ret;
}

/**
  * @brief  Set FIFO bypass mode
  * @retval BSP status
  */
static int32_t LSM6DSM_FIFO_Set_Bypass_Mode(void)
{
  int32_t ret;

  if ((ret = BSP_MOTION_SENSOR_FIFO_Set_Mode(LSM6DSM_0, (uint8_t)LSM6DSM_BYPASS_MODE)) != BSP_ERROR_NONE)
  {
    return ret;
  }

/*
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nFIFO is stopped in Bypass mode.\r\n");
  printf("%s", dataOut);

  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nPress USER button to start the DEMO...\r\n");
  printf("%s", dataOut);
  */

  return ret;
}

/**
  * @brief  Set FIFO to Continuous mode
  * @retval BSP status
  */
static int32_t LSM6DSM_FIFO_Set_Continuous_Mode(void)
{
  int32_t ret;

  /*
  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nLSM6DSM starts to store the data into FIFO...\r\n\r\n");
  printf("%s", dataOut);

  */

  HAL_Delay(1000);

  /* Set FIFO mode to Continuous */
  if ((ret = BSP_MOTION_SENSOR_FIFO_Set_Mode(LSM6DSM_0, (uint8_t)LSM6DSM_STREAM_MODE)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  return ret;
}

/**
  * @brief  Read all unread FIFO data in cycle
  * @retval BSP status
  */
static int32_t LSM6DSM_Read_All_FIFO_Data(void)
{
  uint16_t samples_to_read = 0;
  uint16_t i;
  int32_t ret;

  /* Get num of unread FIFO samples before reading data */
  if ((ret = BSP_MOTION_SENSOR_FIFO_Get_Num_Samples(LSM6DSM_0, &samples_to_read)) != BSP_ERROR_NONE)
  {
    return ret;
  }

  /* 'samples_to_read' actually contains number of words in FIFO but each FIFO sample (data set) consists of 3 words
  so the 'samples_to_read' has to be divided by 3 */
  samples_to_read /= 3U;

  /*

  (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\n\r\n%d samples in FIFO.\r\n\r\nStarted downloading data from FIFO...\r\n\r\n", samples_to_read);
  printf("%s", dataOut);

  (void)snprintf(dataOut, MAX_BUF_SIZE, "[DATA ##]     GYR_X     GYR_Y     GYR_Z\r\n");
  printf("%s", dataOut);

  */

  for (i = 0; i < samples_to_read; i++)
  {
    if ((ret = LSM6DSM_Read_Single_FIFO_Pattern_Cycle(i)) != BSP_ERROR_NONE)
    {
      return ret;
    }
  }

  if (samples_to_read > SAMPLE_LIST_MAX)
  {
    /*
    (void)snprintf(dataOut, MAX_BUF_SIZE, "\r\nSample list limited to: %d\r\n\r\n", SAMPLE_LIST_MAX);
    printf("%s", dataOut);
    */
  }

  return ret;
}

/**
  * @brief  Read single FIFO pattern cycle
  * @param  SampleIndex Current sample index.
  * @retval BSP status
  */
static int32_t LSM6DSM_Read_Single_FIFO_Pattern_Cycle(uint16_t SampleIndex)
{
  uint16_t pattern = 0;
  int32_t angular_velocity = 0;
  int32_t gyr_x = 0, gyr_y = 0, gyr_z = 0;
  int32_t ret = BSP_ERROR_NONE;
  int i;

  /* Read one whole FIFO pattern cycle. Pattern: Gx, Gy, Gz */
  for (i = 0; i <= 2; i++)
  {
    /* Read FIFO pattern number */
    if ((ret = BSP_MOTION_SENSOR_FIFO_Get_Pattern(LSM6DSM_0, &pattern)) != BSP_ERROR_NONE)
    {
      return ret;
    }

    /* Read single FIFO data (angular velocity in one axis) */
    if ((ret = BSP_MOTION_SENSOR_FIFO_Get_Axis(LSM6DSM_0, MOTION_GYRO, &angular_velocity)) != BSP_ERROR_NONE)
    {
      return ret;
    }

    /* Decide which axis has been read from FIFO based on pattern number */
    switch (pattern)
    {
      case PATTERN_GYR_X_AXIS:
        gyr_x = angular_velocity;
        break;

      case PATTERN_GYR_Y_AXIS:
        gyr_y = angular_velocity;
        break;

      case PATTERN_GYR_Z_AXIS:
        gyr_z = angular_velocity;
        break;

      default:
        ret = BSP_ERROR_UNKNOWN_FAILURE;
        break;
    }
  }

  if (ret != BSP_ERROR_NONE)
  {
    return ret;
  }

  if (SampleIndex < SAMPLE_LIST_MAX)
  {
    /*
    (void)snprintf(dataOut, MAX_BUF_SIZE, "[DATA %02d]  %8ld  %8ld  %8ld\r\n", SampleIndex + 1U, gyr_x, gyr_y, gyr_z);
    printf("%s", dataOut);
    */
  }


  return ret;
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

#ifdef __cplusplus
}
#endif
