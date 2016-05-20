/**
 ******************************************************************************
 * @file    modules/common/c_common_comm_controlData.c
 * @author  Fernando Silvano Gonçalves
 * @version V1.0.0
 * @date    05-April-2016
 * @brief   Funções para troca de mensagens entre as threads dos dados do VANT.
 *
 *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_common_comm_controlData.h"

/** @addtogroup Common_Components
 * @{
 */

/** @addtogroup Common_Components_Utils
 *
 * \brief Implementa funções utilitárias para o projeto.
 *
 *
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
pv_msg_controlData controlData;
//static const xSemaphoreHandle xSemaphore = xSemaphoreCreateMutex();
xSemaphoreHandle xSemaphoreControl;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de controle + output.
 *
 * Instancia as Queues de comunicação inter-thread, inicializa a pinagem necessária para
 * os perifericos e aloca o que for necessário para as equações de controle.
 * @param  None
 * @retval None
 */
void init_controlData() {
	xSemaphoreControl = xSemaphoreCreateMutex();

	/*Initialize every data structures*/
	/* Initialize actuation data */
	controlData.actuation.escLeftNewtons = 0;
	controlData.actuation.escLeftSpeed = 0;
	controlData.actuation.escRightNewtons = 0;
	controlData.actuation.escRightSpeed = 0;
	controlData.actuation.servoLeft = 0;
	controlData.actuation.servoRight = 0;
	controlData.actuation.servoTorqueControlEnable = 0;

	/* Initialize vant behavior data */
	controlData.vantBehavior.drpy[0] = 0;
	controlData.vantBehavior.drpy[1] = 0;
	controlData.vantBehavior.drpy[2] = 0;
	controlData.vantBehavior.dxyz[0] = 0;
	controlData.vantBehavior.dxyz[1] = 0;
	controlData.vantBehavior.dxyz[2] = 0;
	controlData.vantBehavior.rpy[0] = 0;
	controlData.vantBehavior.rpy[1] = 0;
	controlData.vantBehavior.rpy[2] = 0;
	controlData.vantBehavior.xyz[0] = 0;
	controlData.vantBehavior.xyz[1] = 0;
	controlData.vantBehavior.xyz[2] = 0;

	controlData.cicleTime = 0;

	controlData.heartBeat = 0;
}

void set_controlData(pv_msg_controlData message) {
	xSemaphoreTake(xSemaphoreControl, 3/ portTICK_RATE_MS);
	controlData = message;
	xSemaphoreGive(xSemaphoreControl);
}

pv_msg_controlData get_controlData() {
	pv_msg_controlData tempData;
	xSemaphoreTake(xSemaphoreControl, 3/ portTICK_RATE_MS);
	tempData = controlData;
	xSemaphoreGive(xSemaphoreControl);
	return tempData;
}

void set_vantBehaviorData(pv_type_vantBehavior message) {
	xSemaphoreTake(xSemaphoreControl, 3/ portTICK_RATE_MS);
	controlData.vantBehavior = message;
	xSemaphoreGive(xSemaphoreControl);
}

pv_type_vantBehavior get_vantBehaviorData() {
	pv_type_vantBehavior tempData;
	xSemaphoreTake(xSemaphoreControl, 3/ portTICK_RATE_MS);
	tempData = controlData.vantBehavior;
	xSemaphoreGive(xSemaphoreControl);
	return tempData;
}

void set_actuationData(pv_type_actuation message) {
	xSemaphoreTake(xSemaphoreControl, 3/ portTICK_RATE_MS);
	controlData.actuation = message;
	xSemaphoreGive(xSemaphoreControl);
}

pv_type_actuation get_actuationData() {
	pv_type_actuation tempData;
	xSemaphoreTake(xSemaphoreControl, 3/ portTICK_RATE_MS);
	tempData = controlData.actuation;
	xSemaphoreGive(xSemaphoreControl);
	return tempData;
}

void set_heartBeat(unsigned int message) {
	xSemaphoreTake(xSemaphoreControl, 3/ portTICK_RATE_MS);
	controlData.heartBeat = message;
	xSemaphoreGive(xSemaphoreControl);
}

unsigned int get_heartBeat() {
	unsigned int tempData;
	xSemaphoreTake(xSemaphoreControl, 3/ portTICK_RATE_MS);
	tempData = controlData.heartBeat;
	xSemaphoreGive(xSemaphoreControl);
	return tempData;
}

void set_cicleTime(unsigned int message) {
	xSemaphoreTake(xSemaphoreControl, 3/ portTICK_RATE_MS);
	controlData.cicleTime = message;
	xSemaphoreGive(xSemaphoreControl);
}

unsigned int get_cicleTime() {
	unsigned int tempData;
	xSemaphoreTake(xSemaphoreControl, 3/ portTICK_RATE_MS);
	tempData = controlData.cicleTime;
	xSemaphoreGive(xSemaphoreControl);
	return tempData;
}

