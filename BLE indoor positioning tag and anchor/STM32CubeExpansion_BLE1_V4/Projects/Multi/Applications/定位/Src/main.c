/**
******************************************************************************
* @file    main.c 
* @author  CL
* @version V1.0.0
* @date    04-July-2014
* @brief   This sample code shows how to use the BlueNRG STM32 expansion board
*          to exchange data between two devices.
*          It provides also the feature for the throughput estimation.
*          The communication is done using two STM32 Nucleo boards.
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
#include "cube_hal.h"
#include "osal.h"
#include "sample_service.h"
#include "role_type.h"
#include "debug.h"
#include "stm32_bluenrg_ble.h"
#include "bluenrg_utils.h"
#include <stdio.h>
#include "bsp.h"
#include "vcom.h"

/* Private macros ------------------------------------------------------------*/
//device defines 
//int time = 0;
struct timer t;
tBleStatus ret = 0;
tBleStatus resp;
/* Private variables ---------------------------------------------------------*/  
uint8_t bdaddr[6];
UART_HandleTypeDef UartHandle;

/* Private function prototypes -----------------------------------------------*/
void User_Process(void);
void Init(void);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void vcom_init(void);
int main(void)
{ 
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
	


  Init();
  /* assign address */
  #ifdef ANCHOR_1 // a1
  Osal_MemCpy(bdaddr, bdaddr_anchor1, sizeof(bdaddr_anchor1));
  #endif
  #ifdef ANCHOR_2 //a2
  Osal_MemCpy(bdaddr, bdaddr_anchor2, sizeof(bdaddr_anchor2));
  #endif
  #ifdef ANCHOR_3 //a3
  Osal_MemCpy(bdaddr, bdaddr_anchor3, sizeof(bdaddr_anchor3));
  #endif 
  #ifdef TAG //aa
  Osal_MemCpy(bdaddr, bdaddr_tag, sizeof(bdaddr_tag));
  #endif
  #ifdef SERVER //bb
  Osal_MemCpy(bdaddr, bdaddr_server, sizeof(bdaddr_server));
  #endif
  
  BlueNRG_RST();
	
  /* set address as public sddress*/
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
  if(ret)
  {
    PRINTF("Setting BD_ADDR failed 0x%02x.\n", ret);
	Error_Handler();
  }
  /* gatt init */
  ret = aci_gatt_init();    
  if(ret)
  {
    PRINTF("GATT_Init failed.\n");
	Error_Handler();
  }
  
  /*set GAP role and basic services and characteristics*/
  #if defined(TAG) 
  ret = aci_gap_init_IDB05A1(GAP_CENTRAL_ROLE_IDB05A1, 0, 
			                 0x07, &service_handle, 
                             &dev_name_char_handle, 
			                 &appearance_char_handle);
  #else 
  ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 
			                 0x07, &service_handle, 
                             &dev_name_char_handle, 
			                 &appearance_char_handle);
  #endif

  
  if(ret != BLE_STATUS_SUCCESS)
  {
     PRINTF("Server GAP_Init failed.\n");
	 Error_Handler();
  }
  
  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);

  #ifdef SERVER
  ret = Add_Sample_Service();
  if(ret != BLE_STATUS_SUCCESS)
  {
     PRINTF("Server Add services failed.\n");
	 Error_Handler();
  }  
  #endif
  

  
  while(1)
  {
    HCI_Process();
    User_Process();
		
		HAL_Delay(500);
//		Timer_Set(&t,10);
//			if(Timer_Expired(&t))
//			{
//				Timer_Reset(&t);
//				HAL_NVIC_SystemReset();
//			}
//		time++;
//		//PRINTF("%d\n",time);
//		if(time == 300)
//		{
//			HAL_NVIC_SystemReset();
//		}
  }
}

/**
 * @brief  Process user input (i.e. pressing the USER button on Nucleo board)
 *         and send a LED toggle command to the remote board.
 * @param  None
 * @retval None
 */
#ifdef TEST
void User_Process(void)
{
	#ifdef TAG
	if(tagstate == TAG_CHAR1)
	{
		ret++;
		if(ret == 100)
		{
			ret = 0;
		}
		char a = 1;
		resp = aci_gatt_write_without_response(connection_handle, 0x0011, 1, (const uint8_t*)&a);

		HAL_Delay(50);
		
		aci_gap_terminate(connection_handle, 0x00);
		HAL_Delay(1000);
	}
	else
	{
		aci_gap_create_connection(0x4000, 0x4000, PUBLIC_ADDR, bdaddr_server, PUBLIC_ADDR, CONN_P1, CONN_P2, 0,
                                  SUPERV_TIMEOUT, CONN_L1, CONN_L2);	
	}
	#endif
	 
	#ifdef SERVER
	if(serverstate == SER_WAIT)
	{
		
	}
	else
	{
			hci_le_set_scan_resp_data(0, NULL); //disable scan response
			//need locacl name and name length
			resp = aci_gap_set_discoverable(ADV_IND,  
		                     0,
		                     0, 
		                     PUBLIC_ADDR, 
		                     NO_WHITE_LIST_USE,
                             sizeof(local_name_anchor), 
						     local_name_anchor, 
		                     0, NULL, 0, 0);
		
	}
	#endif
}
#endif
	
void User_Process(void)
{
	#if defined(ANCHOR_1) || defined(ANCHOR_2) ||defined(ANCHOR_3) // anchor - one action : set discoverable
	
    hci_le_set_scan_resp_data(0, NULL); //disable scan response
	//need locacl name and name length
	aci_gap_set_discoverable(ADV_NONCONN_IND,  
		                     0x0020,
		                     0x0020, 
		                     PUBLIC_ADDR, 
		                     NO_WHITE_LIST_USE,
                             sizeof(local_name_anchor), 
						     local_name_anchor, 
		                     0, NULL, 0, 0);
    #endif
							
    #ifdef TAG	// TAG	判断TAG在不同state下要做的事情							
	switch(tagstate)
	{
		case TAG_IDLE:
			//PRINTF("IDLE\n");
		break;
		
		case TAG_DISC:
		{//PRINTF("DISC\n");
			aci_gap_start_general_discovery_proc(SCAN_INTERVAL,  
										         SCAN_WINDOW,
									             PUBLIC_ADDR, 0);
			Timer_Set(&t,10);
			if(Timer_Expired(&t))
			{
				Timer_Reset(&t);
				HAL_NVIC_SystemReset();
			}
		}
		break;
		
		case TAG_CONN: 
		{//PRINTF("CONN\n");
			aci_gap_terminate_gap_procedure(0x02); // GENERAL_DISCOVERY_PROC
			// need address
			aci_gap_create_connection(SCAN_P, SCAN_L, PUBLIC_ADDR, bdaddr_server, PUBLIC_ADDR, CONN_P1, CONN_P2, 0,
                                      SUPERV_TIMEOUT, CONN_L1, CONN_L2);
		}
		break;
		
		case TAG_CHAR1:
		{//PRINTF("CHAR1\n");
			if(c1IsReceving == 0) // to make the CHAR_proc only be carried once 
			{
				// need uuid + connection handle
				aci_gatt_disc_charac_by_uuid(connection_handle, 0x0001, 0xFFFF, UUID_TYPE_128, charUuidPosition);
				c1IsReceving = 1;
			}
			Timer_Set(&t,10);
			if(Timer_Expired(&t))
			{
				Timer_Reset(&t);
				HAL_NVIC_SystemReset();
			}
			
		}
		break;
		
		case TAG_CHAR2:
		{
			if(c2IsReceving == 0) // to make the CHAR_proc only be carried once
			{
				// need uuid + connection handle
				aci_gatt_disc_charac_by_uuid(connection_handle, 0x0001, 0xFFFF, UUID_TYPE_128, charUuidRoomSize);
				c2IsReceving = 1;
			}

		}
		break;
		
		case TAG_WRITE:
		{//PRINTF("WRITE\n");
			char data_position[20];
			//char data_roomSize[2];
			
//			data_roomSize[0] = room_length;
//			data_roomSize[1] = room_wide;
//			resp = aci_gatt_write_without_response(connection_handle, roomSizeCharHandle + 1, 
//			                                       2, (const uint8_t*)data_roomSize);
															
			dataPack(x, y, data_position);
			resp = aci_gatt_write_without_response(connection_handle, positionCharHandle + 1, 
		                                           20, (const uint8_t*)data_position);
//		char a[] = "12345678901234567890";
//		resp = aci_gatt_write_without_response(connection_handle, 0x0011, 
//			                                       20, (const uint8_t*)&a);	
//		HAL_Delay(100);
//			
//		resp = aci_gatt_write_without_response(connection_handle, 0x0011, 
//			                                       20, (const uint8_t*)&a);	
		HAL_Delay(100);
			
		aci_gap_terminate(connection_handle, 0x00);
		
		}
		break;
		
		default:
		break;
	
	}							 
	#endif	
							 
    #ifdef SERVER      //判断server在不同state下要做的事情
	switch(serverstate)
	{
		case SER_IDLE:
		break;

		case SER_DISC:
		{
			hci_le_set_scan_resp_data(0, NULL); //disable scan response
			//need locacl name and name length
			resp = aci_gap_set_discoverable(ADV_IND,  
		                     0x20,
		                     0x20, 
		                     PUBLIC_ADDR, 
		                     NO_WHITE_LIST_USE,
                             sizeof(local_name_anchor), 
						     local_name_anchor, 
		                     0, NULL, 0, 0);
			
		}
		break;
		
		case SER_WAIT:
		{
			PRINTF("%f,%f\n",x,y);
			// do nothing, just wait EVENTS
		}
		break;
		
		case SER_UPDATE1: // update position
		{
			char data[22];
			dataPack(x, y, data);
			aci_gatt_update_char_value(sampleServHandle, positionCharHandle, 0, 20, data);
			serverstate = SER_WAIT;
		}
		break;
		
		case SER_UPDATE2: //// update room size
		{
			char data[2];
			data[0] = room_length;
			data[1] = room_wide;
			aci_gatt_update_char_value(sampleServHandle, roomSizeCharHandle, 0, 2, data);
			serverstate = SER_WAIT;
		}
		break;
		
		default:
		break;
	
	}							 

	#endif
}

void Init(void)
{
  HAL_Init();

  /* Configure LED2 */
  BSP_LED_Init(LED2);
  
  /* Configure the User Button in GPIO Mode */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initialize the VCOM interface */
  vcom_init();
  
  /* Initialize the BlueNRG SPI driver */
  BNRG_SPI_Init();

  /* Initialize the BlueNRG HCI */
  HCI_Init();
}

/** 
* @brief  Init the VCOM.
* @param  None
* @return None
*/
void vcom_init(void)
{
  /*## Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = ODD parity
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance        = USARTx;
  
  UartHandle.Init.BaudRate   = 9600;
  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits   = UART_STOPBITS_1;
  UartHandle.Init.Parity     = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode       = UART_MODE_TX_RX;
  
  if(HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
}

/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - NVIC configuration for UART interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  USARTx_TX_GPIO_CLK_ENABLE();
  USARTx_RX_GPIO_CLK_ENABLE();
  /* Enable USART1 clock */
  USARTx_CLK_ENABLE(); 
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* UART TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = USARTx_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = USARTx_TX_AF;
  
  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
    
  /* UART RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = USARTx_RX_PIN;
  GPIO_InitStruct.Alternate = USARTx_RX_AF;
    
  HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
}
#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  //printf("Error_Handler\n");
  /* Blink LED2 */
  while(1)
  {
    BSP_LED_Toggle(LED2);
    HAL_Delay(100);
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
