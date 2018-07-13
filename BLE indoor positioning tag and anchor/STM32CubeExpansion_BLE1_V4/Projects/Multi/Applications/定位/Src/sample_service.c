/**
  ******************************************************************************
  * @file    sample_service.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    04-July-2014
  * @brief   Add a sample service using a vendor specific profile.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "sample_service.h"
#include "connection_config.h"
#include "osal.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* Private variables ---------------------------------------------------------*/
// address
uint8_t bdaddr_anchor1[6] = {0xa1, 0x00, 0x00, 0xE1, 0x80, 0x02};
uint8_t bdaddr_anchor2[6] = {0xa2, 0x00, 0x00, 0xE1, 0x80, 0x02};
uint8_t bdaddr_anchor3[6] = {0xa3, 0x00, 0x00, 0xE1, 0x80, 0x02};
uint8_t bdaddr_tag[6] = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02};
uint8_t bdaddr_server[6] = {0xbb, 0x00, 0x00, 0xE1, 0x80, 0x02};

int t_event = 0;
int t_connection = 0;
int t_disconnection = 0;
int t_modified  = 0;
int t_readChar = 0;
int t_endpro_other = 0;
int t_endpro_1 = 0;
int t_endpro_2 = 0;
int t_recvadv = 0;
int count1=0;
int count2=0;
int count3=0;
// device names
const char local_name_anchor[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','L','E','_','A','D','V'};
//const char local_name_server[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,'A','L','E','_','S','E','R'};


// uuid of service and characteristics
const uint8_t service_uuid[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd7};
const uint8_t charUuidPosition[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd8};
// reserved uuid
const uint8_t charUuidRoomSize[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};

// status 
#ifdef TAG
enum TagState tagstate = TAG_DISC; 
#endif
#ifdef SERVER
enum ServerState serverstate = SER_DISC;
#endif
// variables 
uint8_t c1IsReceving = 0;
uint8_t c2IsReceving = 0;

float DIS = 5;;
float d1=90;
float d2=90;
float d3=90;
float x;
float y;
int count=0;
//uint8_t room_length = 2;  
//uint8_t room_wide = 1;

// service & characteristics handles ,uesd both by server and tag
uint16_t sampleServHandle;
uint16_t positionCharHandle; 
uint16_t roomSizeCharHandle;

uint16_t connection_handle;

#define RSSI_SIZE 10

/**
 * @}
 */

/** @defgroup SAMPLE_SERVICE_Private_Macros
 * @{
 */
/* Private macros ------------------------------------------------------------*/
void dataPack(float position_x, float position_y, char * stringData )
{
	//stringDATA must be larger than 22.
    char buffer_x[10];
	char buffer_y[10];
    sprintf( buffer_x, "%f", position_x);
	sprintf( buffer_y, "%f", position_y);
	Osal_MemCpy(stringData, buffer_x, sizeof(buffer_x));
	Osal_MemCpy(stringData + sizeof(buffer_x), buffer_y, sizeof(buffer_y));
}
void dataUnpack(float *position_x, float *position_y, uint8_t * stringData)
{
    char buffer_x[10];
	char buffer_y[10];
	Osal_MemCpy(buffer_x, stringData, sizeof(buffer_x));
	Osal_MemCpy(buffer_y, stringData + sizeof(buffer_x), sizeof(buffer_y));
	*position_x = atof(buffer_x);
	*position_y = atof(buffer_y);	
}

/**
 * @brief  Add a sample service using a vendor specific profile
 * @param  None
 * @retval Status
 */
tBleStatus Add_Sample_Service(void) // only used with SERVER
{ 
  tBleStatus ret;
	
  ret = aci_gatt_add_serv(UUID_TYPE_128, service_uuid, PRIMARY_SERVICE, 7, &sampleServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
    
  ret =  aci_gatt_add_char(sampleServHandle, 
						   UUID_TYPE_128, charUuidPosition, 20, 
						   CHAR_PROP_NOTIFY|CHAR_PROP_WRITE|CHAR_PROP_WRITE_WITHOUT_RESP, 
						   ATTR_PERMISSION_NONE, 
						   GATT_NOTIFY_ATTRIBUTE_WRITE,
                           16, 1, &positionCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
	
  ret =  aci_gatt_add_char(sampleServHandle, 
						   UUID_TYPE_128, charUuidRoomSize, 20, 
						   CHAR_PROP_NOTIFY|CHAR_PROP_WRITE|CHAR_PROP_WRITE_WITHOUT_RESP, 
						   ATTR_PERMISSION_NONE, 
						   GATT_NOTIFY_ATTRIBUTE_WRITE,
                           16, 1, &roomSizeCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
	
  return BLE_STATUS_SUCCESS; 
  
fail:
  PRINTF("Error while adding Sample Service.\n");
  return BLE_STATUS_ERROR ;
}

/**
 * @brief  This function is called when an attribute gets modified
 * @param  handle : handle of the attribute
 * @param  data_length : size of the modified attribute data
 * @param  att_data : pointer to the modified attribute data
 * @retval None
 */
void Evt_Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data)
{
	t_modified++;
	#ifdef SERVER
	if(handle == positionCharHandle + 1) // value handle position
	{
		dataUnpack(&x, &y, att_data);
	}

	if(handle == positionCharHandle + 2) // descriptor handle position
	{        
		if((att_data[0] == 0x01) && (serverstate == SER_WAIT))
			serverstate = SER_UPDATE1;
	}
	
	if(handle == positionCharHandle + 1) // value handle room size
	{
		room_length = att_data[0];
		room_wide = att_data[1];
	}

	if(handle == positionCharHandle + 2) // descriptor handle room size
	{        
		if((att_data[0] == 0x01) && (serverstate == SER_WAIT))
			serverstate = SER_UPDATE2;
	}
	#endif
}
//假设房间宽2m，高1m，1锚节点放在(0,0),2锚节点放在(2,0)，3锚节点放在(1,1)
//d1^2=x^2+y^2
//d2^2=(x-7.83)^2+y^2
//d3^2=(0.5*7.83-x)^2+(9-y)^2

void RssiToDis(int8_t rssi)
{
	int AttenuOm = 60;
	double AttenuFa = 2.957;
	// Can be determined by caculation.  
	//algorithm: Pr(d) = Pr(d0) - 10*k*log(d/d0), and d0 is the distance betweent Rx and Tx	                            
    int8_t PRSSI = abs(rssi);
	double power2 = (PRSSI-AttenuOm)/(10*AttenuFa);
	DIS = pow(10, power2);
}

void Evt_AD_REPORT_GET_DATA( uint8_t att_info_evt_type[1], 
						     uint8_t att_info_addr_type[1], 
   						     uint8_t att_info_addr[6], 
						     uint8_t att_info_data_length[1], 
						     uint8_t *att_info_data_rssi )
{ t_recvadv++;
  #ifdef TAG
  int8_t RSSI = -(att_info_data_rssi[14]- att_info_data_rssi[15]);
  if(RSSI < 10)
  {
	  RssiToDis(RSSI);
	  
		  if (att_info_addr[0] == bdaddr_anchor1[0]&&att_info_addr[1] == bdaddr_anchor1[1] 
				&& att_info_addr[2] == bdaddr_anchor1[2] && att_info_addr[3] == bdaddr_anchor1[3] 
			&& att_info_addr[4] == bdaddr_anchor1[4] && att_info_addr[5] == bdaddr_anchor1[5])
		  {
			  d1 = -RSSI;
			 
		  }
		  
		  if (att_info_addr[0] == bdaddr_anchor2[0]&&att_info_addr[1] == bdaddr_anchor2[1] 
				&& att_info_addr[2] == bdaddr_anchor2[2] && att_info_addr[3] == bdaddr_anchor2[3] 
			&& att_info_addr[4] == bdaddr_anchor2[4] && att_info_addr[5] == bdaddr_anchor2[5])
		  {
			  d2 = -RSSI;
			
		  }	  
		  if (att_info_addr[0] == bdaddr_anchor3[0]&&att_info_addr[1] == bdaddr_anchor3[1] 
				&& att_info_addr[2] == bdaddr_anchor3[2] && att_info_addr[3] == bdaddr_anchor3[3] 
			&& att_info_addr[4] == bdaddr_anchor3[4] && att_info_addr[5] == bdaddr_anchor3[5])
		  {
			  d3 = -RSSI;				
		  }  
	 if(d1<d2&&d1<d3)
	 {
		 x=3.91;
		 y=4.50;
	 }
	 else if(d2<d1&&d2<d3)
	 {
		 x=9.92;
		 y=6.50;
	 }
	 else if(d3<d1&&d3<d2)
	 {
		 x=0.00;
		 y=10.47;
	 }
	 else
	 {
		 x=7.83;
		 y=9.00;
	 }
	 count++;
//	  	x = 3.915 - 0.0639 * pow(d2, 2) + 0.0639 * pow(d1, 2);
//      y = 3.65+0.0278 * pow(d2, 2) + 0.0278 * pow(d1, 2) - 0.556 * pow(d3, 2)+3;
			//&&x<7.83&&y<9)
//				if(count1!=100&&count2!=100&&count3!=100)
//				{
//					//if(x>3.9&&y>4.5)
//					PRINTF("%f,%f\n",x,y);
//				}
//				else if(count1==19)
//					{
//					 x=6.623870;
//						y=7.169348;
//						PRINTF("%f,%f\n",x,y);
//						count1=0;
//					}
//						else if(count2==13)
//					{
//					 x=5.915342;
//						y=7.882889;
//						PRINTF("%f,%f\n",x,y);
//						count2=0;
//					}
//						else if(count3==10)
//					{
//					 x=5.619321;
//						y=6.411214;
//						PRINTF("%f,%f\n",x,y);
//						count3=0;
//					}
//			count1++;
//if(x>0&&y>0)
if(count==3)
{
	PRINTF("%lf,%lf\n",x,y);
  count=0;
}
//PRINTF("%f,%f\n",x,y);
			if(att_info_addr[0] == bdaddr_server[0])
							tagstate = TAG_CONN;
   }
	#endif
}
	
/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  addr : Address of peer device
 * @param  handle : Connection handle
 * @retval None
 */
//完成此函数后如果tag的state为TAG_CONN则变为TAG_CHAR1, 如果server的state为SER_DISC则变为SER_WAIT
void Evt_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{ 
	t_connection++;
	
	#ifdef TAG
	connection_handle = handle;
	#ifndef TEST
	if(tagstate == TAG_CONN)
	#endif
		tagstate = TAG_CHAR1;
	#endif
	
	#ifdef SERVER
	#ifndef TEST
	if(serverstate == SER_DISC)
	#endif
		serverstate = SER_WAIT;
	#endif
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
// 完成此函数后tag的state变为TAG_DISC, server的state变为SER_DISC
void Evt_DisconnectionComplete_CB(void)
{
	t_disconnection++;
	#ifdef TAG
		tagstate = TAG_DISC;
	#endif
	
	#ifdef SERVER
		serverstate = SER_DISC;
	#endif
}

/**
 * @brief  This function is called when there is a notification from the sever.
 * @param  attr_handle Handle of the attribute
 * @param  attr_len    Length of attribute value in the notification
 * @param  attr_value  Attribute value in the notification
 * @retval None
 */
void Evt_Notification_CB(uint16_t attr_handle, uint8_t attr_len, uint8_t *attr_value)
{
//    if(attr_handle == postionhandle + 1){
//      receiveData(attr_value, attr_len);
//    }
}
void Evt_Read_Char_By_Uuid_Resp_CB(uint16_t handle)
{
	t_readChar++;
	#ifdef TAG
	if(c1IsReceving == 1)
		positionCharHandle = handle;
	if(c2IsReceving == 1)
		roomSizeCharHandle = handle;
	#endif
}

void Evt_ProcetureComplete()
{
	
	#ifdef TAG
	if(c2IsReceving == 1)
	{
		tagstate = TAG_WRITE;
		c2IsReceving = 0;
		t_endpro_1++;
	}
	else
	if(c1IsReceving == 1)
	{
		tagstate = TAG_CHAR2;
		c1IsReceving = 0;
		t_endpro_2++;
	}
	else
	{
		t_endpro_other++;
	}
	#endif
}
/**
 * @brief  This function is called whenever there is an ACI event to be processed.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  pckt  Pointer to the ACI packet
 * @retval None
 */
//提供了一整个应对各种事件发生时的回调措施
void HCI_Event_CB(void *pckt)
{
	hci_uart_pckt *hci_pckt = pckt;
	hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
	if(hci_pckt->type != HCI_EVENT_PKT)
		return;
	t_event++;
	switch(event_pckt->evt)
	{
		
		case EVT_DISCONN_COMPLETE:
		{
			Evt_DisconnectionComplete_CB();   //Tag ->discover  Server->discover
		}
		break;
    
		case EVT_LE_META_EVENT:
		{
			evt_le_meta_event *evt = (void *)event_pckt->data;
      
			switch(evt->subevent)
			{
				case EVT_LE_CONN_COMPLETE:       
				{
					evt_le_connection_complete *cc = (void *)evt->data;
					Evt_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);   //tagstate from connection -> char1; serverstate from discover -> wait
				}
				break;
				
				case EVT_LE_ADVERTISING_REPORT: 
				{
					le_advertising_info *pr; 
					/* evt->data[0] is number of reports (On BlueNRG-MS is always 1) */
					pr = (void *)(evt -> data + 1);
					Evt_AD_REPORT_GET_DATA( &(pr->evt_type), 
										  &(pr->bdaddr_type), 
								            pr->bdaddr, 
								          &(pr->data_length), 
								            pr->data_RSSI  );
				}
				break;
				
				default:
				break;
			}
		}
		break;
    
		case EVT_VENDOR:
        {
			evt_blue_aci *blue_evt = (void*)event_pckt->data;
			
			switch(blue_evt->ecode)
			{
        
				case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
				{   
					evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
					Evt_Attribute_Modified_CB(evt->attr_handle, evt->data_length, (evt->att_data));  //2 bytes  offests   Server Unpack data
				}
				break;
				
				case EVT_BLUE_GATT_NOTIFICATION:
				{
					//evt_gatt_attr_notification *evt = (evt_gatt_attr_notification*)blue_evt->data;
					//GATT_Notification_CB(evt->attr_handle, evt->event_data_length - 2, evt->attr_value);
				}
				break;
				
				case EVT_BLUE_GATT_DISC_READ_CHAR_BY_UUID_RESP:
				{
					evt_gatt_disc_read_char_by_uuid_resp *resp = (void*)blue_evt->data;
					Evt_Read_Char_By_Uuid_Resp_CB(resp->attr_handle);
				}
				break;
        
				case EVT_BLUE_GATT_PROCEDURE_COMPLETE:
				{
					Evt_ProcetureComplete();
				}
				break;
				
				default:
				break;
			}
		}
		break;
		
		default:
	    break;
	}    
}

 /**
 * @brief  Caculation. Transfer RSSI (dBm) to Distance (meter)
 * @param  rssi: the received rssi
 * @param  dis: distance from receiver 
 * @retval None
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
