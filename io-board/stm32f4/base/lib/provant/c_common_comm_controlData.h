/**
 ******************************************************************************
 * @file    modules/common/c_common_comm_controlData.h
 * @author  Fernando Silvano Gonçalves
 * @version V1.0.0
 * @date    05-April-2016
 * @brief   Funções gerais para manipulação de controle.
 *
 *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_COMMON_COMM_CONTROLDATA_H
#define C_COMMON_COMM_CONTROLDATA_H

/* Includes ------------------------------------------------------------------*/

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"


/* proVANT includes */
#include "pv_typedefs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void set_vantBehaviorData(pv_type_vantBehavior message);
void set_actuationData(pv_type_actuation message);
void set_controlData(pv_msg_controlData message);
pv_type_vantBehavior get_vantBehaviorData();
void set_heartBeat(unsigned int message);
void set_cicleTime(unsigned int message);
pv_type_actuation get_actuationData();
pv_msg_controlData get_controlData();
unsigned int get_cicleTime();
unsigned int get_heartBeat();
void init_controlData();

/* Header-defined wrapper functions ----------------------------------------- */

#endif //C_COMMON_COM_CONTROLDATA_H
