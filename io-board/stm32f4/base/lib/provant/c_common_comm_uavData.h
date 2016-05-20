/**
 ******************************************************************************
 * @file    modules/common/c_common_comm_uavData.h
 * @author  Fernando Silvano Gonçalves
 * @version V1.0.0
 * @date    05-April-2016
 * @brief   Funções para troca de mensagens entre as threads dos dados do VANT.
 *
 *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_COMMON_COMM_UAVDATA_H
#define C_COMMON_COMM_UAVDATA_H

/* Includes ------------------------------------------------------------------*/

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* proVANT includes */
#include "pv_typedefs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void set_attitudeReferenceData(pv_type_datapr_attitude message);
void set_positionReferenceData(pv_type_datapr_position message);
void set_positionData(pv_type_datapr_position message);
void set_attitudeData(pv_type_datapr_attitude message);
void set_receiverData(pv_type_receiverOutput message);
pv_type_datapr_position get_positionReferenceData();
pv_type_datapr_attitude get_attitudeReferenceData();
void set_sonarData(pv_type_sonarOutput message);
void set_servoData(pv_type_servoOutput message);
void set_imuData(pv_type_imuOutput message);
void set_escData(pv_type_escOutput message);
pv_type_datapr_position get_positionData();
pv_type_datapr_attitude get_attitudeData();
pv_type_receiverOutput get_receiverData();
void set_uavData(pv_msg_uavData message);
void set_enableintegration(bool data);
pv_type_sonarOutput get_sonarData();
pv_type_servoOutput get_servoData();
void set_securityStop(bool data);
pv_type_imuOutput get_imuData();
pv_type_escOutput get_escData();
void set_flightmode(bool data);
pv_msg_uavData get_uavData();
bool get_enableintegration();
void set_init(bool data);
bool get_securityStop();
bool get_flightmode();
void init_uavData();
bool get_init();

/* Header-defined wrapper functions ----------------------------------------- */

#endif //C_COMMON_COMM_UAVDATA_H

