/**
  ******************************************************************************
  * @file    DataLog/Src/datalog_application.c
  * @author  Central Labs
  * @version V1.1.0
  * @date    27-Sept-2016
  * @brief   This file provides a set of functions to handle the datalog
  *          application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "datalog_application.h"
#include "main.h"
#include "string.h"
#include "SensorTile.h"
#include "SensorTile2.h"
#include "SensorTile_bus.h"
#include "SensorTile_motion_sensors.h"
#include "SensorTile_motion_sensors_ex.h"
#include <math.h>

static uint8_t verbose = 0;  /* Verbose output to UART terminal ON/OFF. */

static char dataOut[256];
char newLine[] = "\r\n";

SensorAxes_t acceleration;
SensorAxes_t angular_velocity;
SensorAxes_t magnetic_field;
SensorAxesRaw_t acceleration_raw;
/**
* @brief  Handles the time+date getting/sending
* @param  None
* @retval None
*/
void RTC_Handler( RTC_HandleTypeDef *RtcHandle, char *str)
{
  uint16_t subSec = 0;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;
  uint8_t size;
  
  HAL_RTC_GetTime( RtcHandle, &stimestructure, FORMAT_BIN );
  //HAL_RTC_GetDate( RtcHandle, &sdatestructureget, FORMAT_BIN );
  //subSec = (((((( int )RTC_SYNCH_PREDIV) - (( int )stimestructure.SubSeconds)) * 100) / ( RTC_SYNCH_PREDIV + 1 )) & 0xff );
  
  size = sprintf( str, "%d", HAL_GetTick());
  //CDC_Fill_Buffer(( uint8_t * )str, strlen( str ));
}

void Accelero_Sensor_Handler( void *handle)
{

    if ( BSP_ACCELERO_Get_Axes( handle, &acceleration ) == COMPONENT_ERROR )
    {
          acceleration.AXIS_X = 0;
          acceleration.AXIS_Y = 0;
          acceleration.AXIS_Z = 0;
    }

    if ( BSP_ACCELERO_Get_AxesRaw( handle, &acceleration_raw ) == COMPONENT_ERROR )
        {
              acceleration_raw.AXIS_X = 0;
              acceleration_raw.AXIS_Y = 0;
              acceleration_raw.AXIS_Z = 0;
        }

}

void Accelero_Sensor_Handler_str( void *handle , char *str)
{
  uint8_t who_am_i;
  float odr;
  float fullScale;
  float r, theta, phi;
  float x, y, z;
  uint8_t id;
  uint8_t status;
  int32_t d1, d2, d3, d4, d5, d6;
  const float RADIAN = 57.2957795;
  uint8_t size;


    if ( BSP_ACCELERO_Get_Axes( handle, &acceleration ) == COMPONENT_ERROR )
    {
          acceleration.AXIS_X = 0;
          acceleration.AXIS_Y = 0;
          acceleration.AXIS_Z = 0;
    }

    size = sprintf(str, "%d,%d,%d", (int)acceleration.AXIS_X, (int)acceleration.AXIS_Y, (int)acceleration.AXIS_Z);

}

void Accelero_Sensor_Handler_bytes( void *handle , uint8_t *buff)
{
  uint8_t who_am_i;
  float odr;
  float fullScale;
  float r, theta, phi;
  float x, y, z;
  uint8_t id;
  uint8_t status;
  int32_t d1, d2, d3, d4, d5, d6;
  const float RADIAN = 57.2957795;
  uint8_t size;

    if ( BSP_ACCELERO_Get_Axes( handle, &acceleration ) == COMPONENT_ERROR )
    {
          acceleration.AXIS_X = 0;
          acceleration.AXIS_Y = 0;
          acceleration.AXIS_Z = 0;
    }

    memcpy(buff, acceleration.AXIS_X, 4);
    memcpy(buff + 4, acceleration.AXIS_Y, 4);
    memcpy(buff + 8, acceleration.AXIS_Z, 4);

}

void Gyro_Sensor_Handler( void *handle)
{

    if ( BSP_GYRO_Get_Axes( handle, &angular_velocity ) == COMPONENT_ERROR )
    {
      angular_velocity.AXIS_X = 0;
      angular_velocity.AXIS_Y = 0;
      angular_velocity.AXIS_Z = 0;
    }

}

/**
* @brief  Handles the gyroscope axes data getting/sending
* @param  handle the device handle
* @retval None
*/
void Gyro_Sensor_Handler_str( void *handle, char * str)
{
  
  uint8_t who_am_i;
  float odr;
  float fullScale;
  uint8_t id;
  uint8_t status;
  int32_t d1, d2;
  uint8_t size;

    if ( BSP_GYRO_Get_Axes( handle, &angular_velocity ) == COMPONENT_ERROR )
    {
      angular_velocity.AXIS_X = 0;
      angular_velocity.AXIS_Y = 0;
      angular_velocity.AXIS_Z = 0;
    }
    
    size = sprintf(str, "%d,%d,%d", (int)angular_velocity.AXIS_X, (int)angular_velocity.AXIS_Y, (int)angular_velocity.AXIS_Z);

}


void Gyro_Sensor_Handler_bytes( void *handle, uint8_t * buff)
{

  uint8_t who_am_i;
  float odr;
  float fullScale;
  uint8_t id;
  uint8_t status;
  int32_t d1, d2;
  uint8_t size;

  status = 1;

  if ( status == 1 )
  {
    if ( BSP_GYRO_Get_Axes( handle, &angular_velocity ) == COMPONENT_ERROR )
    {
      angular_velocity.AXIS_X = 0;
      angular_velocity.AXIS_Y = 0;
      angular_velocity.AXIS_Z = 0;
    }

    memcpy(buff, angular_velocity.AXIS_X, 4);
    memcpy(buff + 4, angular_velocity.AXIS_Y, 4);
    memcpy(buff + 8, angular_velocity.AXIS_Z, 4);
  }
}

void Magneto_Sensor_Handler( void *handle)
{

  uint8_t id;
  uint8_t status;

  BSP_MAGNETO_Get_Instance( handle, &id );

  BSP_MAGNETO_IsInitialized( handle, &status );


    if ( BSP_MAGNETO_Get_Axes( handle, &magnetic_field ) == COMPONENT_ERROR )
    {
      magnetic_field.AXIS_X = 0;
      magnetic_field.AXIS_Y = 0;
      magnetic_field.AXIS_Z = 0;
    }


}

/**
* @brief  Handles the magneto axes data getting/sending
* @param  handle the device handle
* @retval None
*/
void Magneto_Sensor_Handler_str( void *handle, char *str )
{
  uint8_t who_am_i;
  float odr;
  float fullScale;
  uint8_t id;
  uint8_t status;
  int32_t d1, d2;
  uint8_t size;
  
  BSP_MAGNETO_Get_Instance( handle, &id );
  
  BSP_MAGNETO_IsInitialized( handle, &status );
  

    if ( BSP_MAGNETO_Get_Axes( handle, &magnetic_field ) == COMPONENT_ERROR )
    {
      magnetic_field.AXIS_X = 0;
      magnetic_field.AXIS_Y = 0;
      magnetic_field.AXIS_Z = 0;
    }
    
    size = sprintf(str, "%d,%d,%d", (int)magnetic_field.AXIS_X, (int)magnetic_field.AXIS_Y, (int)magnetic_field.AXIS_Z);

}


void Magneto_Sensor_Handler_bytes( void *handle, uint8_t * buff )
{
  uint8_t who_am_i;
  float odr;
  float fullScale;
  uint8_t id;
  uint8_t status;
  int32_t d1, d2;
  uint8_t size;

  status = 1;

  if ( status == 1 )
  {
    if ( BSP_MAGNETO_Get_Axes( handle, &magnetic_field ) == COMPONENT_ERROR )
    {
      magnetic_field.AXIS_X = 0;
      magnetic_field.AXIS_Y = 0;
      magnetic_field.AXIS_Z = 0;
    }

    memcpy(buff, magnetic_field.AXIS_X, 4);
    memcpy(buff + 4, magnetic_field.AXIS_Y, 4);
    memcpy(buff + 8, magnetic_field.AXIS_Z, 4);
  }
}

/**
* @brief  Splits a float into two integer values.
* @param  in the float value as input
* @param  out_int the pointer to the integer part as output
* @param  out_dec the pointer to the decimal part as output
* @param  dec_prec the decimal precision to be used
* @retval None
*/
void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec )
{
  *out_int = (int32_t)in;
  if(in >= 0.0f)
  {
    in = in - (float)(*out_int);
  }
  else
  {
    in = (float)(*out_int) - in;
  }
  *out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}


/**
* @brief  Splits a double into two integer values.
* @param  in the double value as input
* @param  out_int the pointer to the integer part as output
* @param  out_dec the pointer to the decimal part as output
* @param  dec_prec the decimal precision to be used
* @retval None
*/
void doubleToInt( double in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec )
{
  *out_int = (int32_t)in;
  if(in >= 0.0f)
  {
    in = in - (double)(*out_int);
  }
  else
  {
    in = (double)(*out_int) - in;
  }
  *out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
