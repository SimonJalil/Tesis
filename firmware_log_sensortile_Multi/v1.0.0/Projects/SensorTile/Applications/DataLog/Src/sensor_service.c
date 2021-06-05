/**
  ******************************************************************************
  * @file    sensor_service.c
  * @author  Central LAB
  * @version V3.1.0
  * @date    14-July-2017
  * @brief   Add 4 bluetooth services using vendor specific profiles.
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
#include <stdio.h>
//#include "TargetFeatures.h"
#include "main.h"
#include "MetaDataManager.h"
#include "sensor_service.h"
//#include "console.h"
//#include "HWAdvanceFeatures.h"
#include "bluenrg_utils.h"
#include "bluenrg_l2cap_aci.h"
#include "uuid_ble_service.h"
#include "OTA.h"
//#include "main.c"

//#define PRINT_DEBUG_SHOT

/* Exported variables ---------------------------------------------------------*/
int connected = FALSE;
uint8_t set_connectable = TRUE;

volatile uint32_t FeatureMask;

/* Imported Variables -------------------------------------------------------------*/
extern uint16_t amountStepsB;
extern uint16_t amountShotsB;
extern uint8_t batteryB;
extern uint8_t gunReleased;

extern volatile uint8_t sampleRate;  //milliseconds
extern volatile uint8_t shotCount;
extern volatile uint8_t quatRevTime; //tiempo previo al detectado el disparo,
									 //para capturar el valor de quaternion

volatile uint8_t audioInitialized = 0;

extern uint32_t ConnectionBleStatus;
  
extern uint32_t FirstConnectionConfig;

extern TIM_HandleTypeDef    TimCCHandle;
extern TIM_HandleTypeDef    TimEnvHandle;
extern TIM_HandleTypeDef    TimAudioDataHandle;
extern TIM_HandleTypeDef    TimShotHandle;

extern volatile char Mydata[256];

extern int32_t BF_dir;

extern uint8_t bdaddr[6];

extern uint32_t uhCCR4_Val;

/* Private variables ------------------------------------------------------------*/

static uint16_t HWServW2STHandle;
static uint16_t EnvironmentalCharHandle;
static uint16_t AccGyroMagCharHandle;
static uint16_t AccEventCharHandle;

static uint16_t ConfigServW2STHandle;
static uint16_t ConfigCharHandle;

static uint16_t ConsoleW2STHandle;
static uint16_t TermCharHandle;
static uint16_t StdErrCharHandle;

static uint8_t LastStderrBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastStderrLen;
static uint8_t LastTermBuffer[W2ST_CONSOLE_MAX_CHAR_LEN];
static uint8_t LastTermLen;

static uint8_t  EnvironmentalCharSize = 2; /* Size for Environmental BLE characteristic */


static uint16_t connection_handle = 0;
uint32_t ConnectionBleStatus  =0;

/* Private functions ------------------------------------------------------------*/
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
static void GAP_DisconnectionComplete_CB(void);

void refreshGAP();

extern void floatToInt( float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec );

/* Private define ------------------------------------------------------------*/

#ifdef ACC_BLUENRG_CONGESTION
#define ACI_GATT_UPDATE_CHAR_VALUE safe_aci_gatt_update_char_value
static int32_t breath;


/* @brief  Update the value of a characteristic avoiding (for a short time) to
 *         send the next updates if an error in the previous sending has
 *         occurred.
 * @param  servHandle The handle of the service
 * @param  charHandle The handle of the characteristic
 * @param  charValOffset The offset of the characteristic
 * @param  charValueLen The length of the characteristic
 * @param  charValue The pointer to the characteristic
 * @retval tBleStatus Status
 */
tBleStatus safe_aci_gatt_update_char_value(uint16_t servHandle, 
				      uint16_t charHandle,
				      uint8_t charValOffset,
				      uint8_t charValueLen,   
				      const uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
  
  if (breath > 0) {
    breath--;
  } else {
    ret = aci_gatt_update_char_value(servHandle,charHandle,charValOffset,charValueLen,charValue);
    
    if (ret != BLE_STATUS_SUCCESS){
      breath = ACC_BLUENRG_CONGESTION_SKIP;
    }
  }
  
  return (ret);
}

#else /* ACC_BLUENRG_CONGESTION */
#define ACI_GATT_UPDATE_CHAR_VALUE aci_gatt_update_char_value
#endif /* ACC_BLUENRG_CONGESTION */


/**
 * @brief  Add the Config service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConfigW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_CONFIG_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3,&ConfigServW2STHandle);

  if (ret != BLE_STATUS_SUCCESS)
    goto fail;

  COPY_CONFIG_W2ST_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConfigServW2STHandle, UUID_TYPE_128, uuid, 20 /* Max Dimension */,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &ConfigCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }
  
  return BLE_STATUS_SUCCESS;

fail:
  //ALLMEMS1_PRINTF("Error while adding Configuration service.\n");
  return BLE_STATUS_ERROR;
}


/**
 * @brief  Add the Console service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_ConsoleW2ST_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];

  COPY_CONSOLE_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 1+3*2,&ConsoleW2STHandle);
  
  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_TERM_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, uuid, W2ST_CONSOLE_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY| CHAR_PROP_WRITE_WITHOUT_RESP | CHAR_PROP_WRITE | CHAR_PROP_READ ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_ATTRIBUTE_WRITE | GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &TermCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
    goto fail;
  }

  COPY_STDERR_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(ConsoleW2STHandle, UUID_TYPE_128, uuid, W2ST_CONSOLE_MAX_CHAR_LEN,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 1, &StdErrCharHandle);

  if (ret != BLE_STATUS_SUCCESS) {
     goto fail;
  }

  return BLE_STATUS_SUCCESS;

fail:
  //ALLMEMS1_PRINTF("Error while adding Console service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update Stderr characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Stderr_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages*/
  for(Offset =0; Offset<length; Offset +=W2ST_CONSOLE_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastStderrBuffer,data+Offset,DataToSend);
    LastStderrLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, StdErrCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
      return BLE_STATUS_ERROR;
    }
    HAL_Delay(10);
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Terminal characteristic value
 * @param  uint8_t *data string to write
 * @param  uint8_t lenght lengt of string to write
 * @retval tBleStatus      Status
 */
tBleStatus Term_Update(uint8_t *data,uint8_t length)
{
  tBleStatus ret;
  uint8_t Offset;
  uint8_t DataToSend;

  /* Split the code in packages */
  for(Offset =0; Offset<length; Offset +=W2ST_CONSOLE_MAX_CHAR_LEN){
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>W2ST_CONSOLE_MAX_CHAR_LEN) ?  W2ST_CONSOLE_MAX_CHAR_LEN : DataToSend;

    /* keep a copy */
    memcpy(LastTermBuffer,data+Offset,DataToSend);
    LastTermLen = DataToSend;

    ret = aci_gatt_update_char_value(ConsoleW2STHandle, TermCharHandle, 0, DataToSend , data+Offset);
    if (ret != BLE_STATUS_SUCCESS) {
        //ALLMEMS1_PRINTF("Error Updating Stdout Char\r\n");
      return BLE_STATUS_ERROR;
    }
    HAL_Delay(20);
  }

  return BLE_STATUS_SUCCESS;
}



/* @brief  Send a notification for answering to a configuration command for Accelerometer events
 * @param  uint32_t Feature Feature calibrated
 * @param  uint8_t Command Replay to this Command
 * @param  uint8_t data result to send back
 * @retval tBleStatus Status
 */
tBleStatus Config_Notify(uint32_t Feature,uint8_t Command,uint8_t data)
{
  tBleStatus ret;
  uint8_t buff[2+4+1+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_BE_32(buff+2,Feature);
  buff[6] = Command;
  buff[7] = data;

  ret = aci_gatt_update_char_value (ConfigServW2STHandle, ConfigCharHandle, 0, 8,buff);
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
      //BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Configuration Char\r\n");
      //Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      //ALLMEMS1_PRINTF("Error Updating Configuration Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Send a notification When the DS3 detects one Acceleration event
 * @param  Command to Send
 * @retval tBleStatus Status
 */
tBleStatus AccEvent_Notify(uint16_t Command)
{
  tBleStatus ret;
  uint8_t buff[2+2];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_LE_16(buff+2,Command);

  ret = aci_gatt_update_char_value(HWServW2STHandle, AccEventCharHandle, 0, 2+2,buff);
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
    //  BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating AccEvent_Notify Char\r\n");
    //  Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
    //  ALLMEMS1_PRINTF("Error Updating AccEvent_Notify Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Add the HW Features service using a vendor specific profile
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_HW_SW_ServW2ST_Service(void)
{
  tBleStatus ret;

  /*
  The following 128bits UUIDs have been generated from the random UUID
  generator:
  D973F2E0-B19E-11E2-9E96-0800200C9A66 --> Service 128bits UUID
  D973F2E1-B19E-11E2-9E96-0800200C9A66 --> Characteristic_1 128bits UUID
  D973F2E2-B19E-11E2-9E96-0800200C9A66 --> Characteristic_2 128bits UUID
  */
  /* Service 128bits UUID */
  const uint8_t service_uuid[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9};
  /* Characteristic_1 128bits UUID */
  const uint8_t charUuid_1[16] =   {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
  /* Characteristic_2 128bits UUID */
  const uint8_t charUuid_2[16] =   {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};
  
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid, PRIMARY_SERVICE, 7, &HWServW2STHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  ret = aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, charUuid_1, 20,
		  CHAR_PROP_NOTIFY|CHAR_PROP_READ,
		  ATTR_PERMISSION_NONE,
		  GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
		  16, 0, &EnvironmentalCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  ret = aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, charUuid_2, 20, CHAR_PROP_WRITE|CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,16, 1, &AccGyroMagCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  return BLE_STATUS_SUCCESS;

fail:
  return BLE_STATUS_ERROR;
}

/**
 * @brief  Update acceleration/Gryoscope and Magneto characteristics value
 * @param  SensorAxes_t Acc Structure containing acceleration value in mg
 * @param  SensorAxes_t Gyro Structure containing Gyroscope value
 * @param  SensorAxes_t Mag Structure containing magneto value
 * @retval tBleStatus      Status
 */
tBleStatus AccGyroMag_Update(SensorAxes_t *Acc,SensorAxes_t *Gyro,SensorAxes_t *Mag)
{  
  tBleStatus ret;
  int32_t AXIS_X;
  int32_t AXIS_Y;
  int32_t AXIS_Z;

  uint8_t buff[2+3*3*2];

  STORE_LE_16(buff   ,(HAL_GetTick()>>3));
  
  STORE_LE_16(buff+2 ,Acc->AXIS_X);
  STORE_LE_16(buff+4 ,Acc->AXIS_Y);
  STORE_LE_16(buff+6 ,Acc->AXIS_Z);
  
  Gyro->AXIS_X/=100;
  Gyro->AXIS_Y/=100;
  Gyro->AXIS_Z/=100;

  STORE_LE_16(buff+8 ,Gyro->AXIS_X);
  STORE_LE_16(buff+10,Gyro->AXIS_Y);
  STORE_LE_16(buff+12,Gyro->AXIS_Z);
    
  /* Apply Magneto calibration */
  //AXIS_X = Mag->AXIS_X - MAG_Offset.AXIS_X;
  //AXIS_Y = Mag->AXIS_Y - MAG_Offset.AXIS_Y;
  //AXIS_Z = Mag->AXIS_Z - MAG_Offset.AXIS_Z;

  STORE_LE_16(buff+14,AXIS_X);
  STORE_LE_16(buff+16,AXIS_Y);
  STORE_LE_16(buff+18,AXIS_Z);
  
  ret = ACI_GATT_UPDATE_CHAR_VALUE(HWServW2STHandle, AccGyroMagCharHandle, 0, 2+3*3*2, buff);
	
  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
    //  BytesToWrite =sprintf((char *)BufferToWrite, "Error Updating Acc/Gyro/Mag Char\r\n");
    //  Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
    //  ALLMEMS1_PRINTF("Error Updating Acc/Gyro/Mag Char\r\n");
    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;	
}


/**
 * @brief  Update Environmental characteristic value
 * @param  int32_t Press Pressure in mbar
 * @param  uint16_t Hum humidity RH (Relative Humidity) in thenths of %
 * @param  int16_t Temp2 Temperature in tenths of degree second sensor
 * @param  int16_t Temp1 Temperature in tenths of degree first sensor
 * @retval tBleStatus   Status
 */
tBleStatus Environmental_Update(uint8_t* Buf, uint32_t TotalLen)
{
  tBleStatus ret;
  uint8_t BuffPos = 0;
  static uint16_t num_ent=1000;
  
  uint8_t buff[2+4/*Press*/+2/*Hum*/+2/*Temp2*/+2/*Temp1*/];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  BuffPos=2;

  num_ent++;
  STORE_LE_16(buff+BuffPos,num_ent);
  BuffPos+=2;

 // ret = aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle, 0, BuffPos,buff);
  ret = aci_gatt_update_char_value(HWServW2STHandle, EnvironmentalCharHandle, 0, TotalLen, Buf);

  if (ret != BLE_STATUS_SUCCESS){
    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_ERR)){
    } else {

    }
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}



void refreshGAP(){
	//static uint8_t varC = 0;
	//varC++;
	  uint8_t manuf_data[26] = {
	    2,0x0A,0xF8 /* -8 dBm */, // Trasmission Power
	    8,0x09,NAME_STLBLE, // Complete Name
		13,
					0x16,
					0x9A,
					0xFE,
					0x7E,
					(amountShotsB>>8) & 0xFF,
					amountShotsB & 0xFF,
					(amountStepsB>>8) & 0xFF,
					amountStepsB & 0xFF,
					batteryB,
					gunReleased,
					0x10,
					0x10,
					0x10,
	  };
	  aci_gap_update_adv_data(26, manuf_data);
}

/**
 * @brief  Puts the device in connectable mode.
 * @param  None 
 * @retval None
 */
void setConnectable(void)
{
 #define NAME_STLBLE 'S','T','L','B',STLBLE_VERSION_MAJOR,STLBLE_VERSION_MINOR,STLBLE_VERSION_PATCH
  char local_name[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,NAME_STLBLE};
  uint8_t manuf_data[26] = {
    2,0x0A,0xF8 /* -8 dBm */, // Trasmission Power
    8,0x09,NAME_STLBLE, // Complete Name
	13,0xFF,0x01/*SKD version */,
    0x02,
    0x00, /* */
    0xE0, /* ACC+Gyro+Mag*/
    0x00, /*  */
    0x00, /*  */
    0x00, /* BLE MAC start */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, /* BLE MAC stop */
  };

  /* BLE MAC */
  //manuf_data[20] = bdaddr[5];
  //manuf_data[21] = bdaddr[4];
  //manuf_data[22] = bdaddr[3];
  //manuf_data[23] = bdaddr[2];
  //manuf_data[24] = bdaddr[1];
  //manuf_data[25] = bdaddr[0];


  /* disable scan response */
  hci_le_set_scan_resp_data(0,NULL);
  aci_gap_set_discoverable(ADV_IND, 0x0640, 0x0640,
  //aci_gap_set_discoverable(ADV_IND, 0, 0,
		  // 0x800 default value - 1.28 s
		  // 0x640  - 1.00 s
#ifndef MAC_BLUEMS
  #ifdef MAC_STM32UID_BLUEMS
                           STATIC_RANDOM_ADDR,
  #else /* MAC_STM32UID_BLUEMS */
                           RANDOM_ADDR,
  #endif /* MAC_STM32UID_BLUEMS */
#else /* MAC_BLUEMS */
                           PUBLIC_ADDR,
#endif /* MAC_BLUEMS */
                           NO_WHITE_LIST_USE,
                           sizeof(local_name), local_name, 0, NULL, 0, 0);

  /* Send Advertising data */
  aci_gap_update_adv_data(26, manuf_data);
}


/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t addr[6] Address of peer device
 * @param  uint16_t handle Connection handle
 * @retval None
 */
static void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
  connected = TRUE;
  connection_handle = handle;

  ConnectionBleStatus=0;
  
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
static void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;


  set_connectable = TRUE;

  ConnectionBleStatus=0;


}

/**
 * @brief  This function is called when there is a Bluetooth Read request
 * @param  uint16_t handle Handle of the attribute
 * @retval None
 */
void Read_Request_CB(uint16_t handle)
{
    if(handle == EnvironmentalCharHandle + 1){
    	Environmental_Update(( uint8_t * )Mydata, strlen( Mydata ));
  }
  //EXIT:
  if(connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}

/**
 * @brief  This function is called when there is a change on the gatt attribute
 * With this function it's possible to understand if one application 
 * is subscribed or not to the one service
 * @param uint16_t att_handle Handle of the attribute
 * @param uint8_t *att_data attribute data
 * @param uint8_t data_length length of the data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t attr_handle, uint8_t * att_data, uint8_t data_length) {
  
  
}


/**
 * @brief  This function is called whenever there is an ACI event to be processed.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void *pckt Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  if(hci_pckt->type != HCI_EVENT_PKT) {
    return;
  }
  
  switch(event_pckt->evt){
    
  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;
  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      
      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){
      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *pr = (void*)blue_evt->data; 
          Read_Request_CB(pr->attr_handle);                    
        }
        break;
      case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
        if(1) {
              evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
              Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
            } else {
              evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
              Attribute_Modified_CB(evt->attr_handle, evt->att_data,evt->data_length);
            }
        break;
      }
    }
    break;
  }
}



/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/
