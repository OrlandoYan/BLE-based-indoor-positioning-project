/**
  ******************************************************************************
  * @file    sample_service.h 
  * @author  CL
  * @version V1.0.0
  * @date    04-July-2014
  * @brief   
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _SAMPLE_SERVICE_H_
#define _SAMPLE_SERVICE_H_
#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#include "hal_types.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "string.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "hci_const.h"
#include "gp_timer.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"
#include "hci.h"
#include "hci_le.h"
#include "hal.h"
#include "sm.h"
#include "debug.h"
#include "connection_config.h"
#include "role_type.h"

#include "uart_support.h"

/* defines */
//#define ANCHOR_1 // server
//#define ANCHOR_2 //server
//#define ANCHOR_3 //server
#define TAG      //client
//#define SERVER //server
//#define TEST
#define  ADV_INTERVAL_MIN_MS   0x4000 //0x80
#define  ADV_INTERVAL_MAX_MS   0x4000 //0x80
#define  SCAN_INTERVAL         0x80
#define  SCAN_WINDOW           0x80

/* state define */
#ifdef TAG
enum TagState
{
	TAG_IDLE = 0U, // reserved
	TAG_DISC,
	TAG_CONN,
	TAG_CHAR1,
	TAG_CHAR2,
	TAG_WRITE,
};
extern enum TagState tagstate;
extern uint8_t c1IsReceving;
extern uint8_t c2IsReceving;
#endif

#ifdef SERVER
enum ServerState
{
	SER_IDLE = 0U, // reserved
	SER_DISC,
	SER_WAIT,
	SER_UPDATE1,
	SER_UPDATE2,
};
extern enum ServerState serverstate;
#endif

/* extern variables*/
extern const uint8_t service_uuid[];
extern const uint8_t charUuidPosition[];
extern const uint8_t charUuidRoomSize[];

extern uint8_t bdaddr_anchor1[6];
extern uint8_t bdaddr_anchor2[6];
extern uint8_t bdaddr_anchor3[6];
extern uint8_t bdaddr_tag[6];
extern uint8_t bdaddr_server[6];

extern const char local_name_anchor[8];
//extern const char local_name_server[8];
	
extern uint16_t sampleServHandle;
extern uint16_t positionCharHandle;
extern uint16_t roomSizeCharHandle;

extern uint16_t connection_handle;

extern float x;
extern float y;
extern uint8_t room_length;  
extern uint8_t room_wide;

tBleStatus Add_Sample_Service(void);

void dataPack(float position_x, float position_y, char * stringData );
  /* User Code*/

/* User Code End */


 
#ifdef __cplusplus
}
#endif

#endif /* _SAMPLE_SERVICE_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

