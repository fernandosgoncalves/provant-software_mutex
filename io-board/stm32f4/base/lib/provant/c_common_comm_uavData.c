/**
 ******************************************************************************
 * @file    modules/common/c_common_comm_uavData.c
 * @author  Fernando Silvano Gonçalves
 * @version V1.0.0
 * @date    05-April-2016
 * @brief   Funções para troca de mensagens entre as threads dos dados do VANT.
 *
 *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_common_comm_uavData.h"

/** @addtogroup Common_Components
 * @{
 */

/** @addtogroup Common_Components_Utils
 *
 * \brief Implementa funções utilitárias para o projeto.
 *
 * Aqui são adicionadas funções genéricas (matemáticas, etc.) passíveis de utilização
 * em todos os módulos.
 *
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
pv_msg_uavData uavData;
//static const xSemaphoreHandle xSemaphore = xSemaphoreCreateMutex();
xSemaphoreHandle xSemaphore;

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
void init_uavData() {
	xSemaphore = xSemaphoreCreateMutex();

	/*Initialize every data structures*/
	/* Initialize IMU data */
	uavData.imuOutput.accRaw[0] = 0;
	uavData.imuOutput.accRaw[1] = 0;
	uavData.imuOutput.accRaw[2] = 0;
	uavData.imuOutput.gyrRaw[0] = 0;
	uavData.imuOutput.gyrRaw[1] = 0;
	uavData.imuOutput.gyrRaw[2] = 0;
	uavData.imuOutput.magRaw[0] = 0;
	uavData.imuOutput.magRaw[1] = 0;
	uavData.imuOutput.magRaw[2] = 0;
	uavData.imuOutput.pressure = 0;
	uavData.imuOutput.sampleTime = 0;
	uavData.imuOutput.temperature = 0;

	/* Initialize receiver data */
	uavData.receiverOutput.aButton = false;
	uavData.receiverOutput.bButton = false;
	uavData.receiverOutput.joystick[0] = 0;
	uavData.receiverOutput.joystick[1] = 0;
	uavData.receiverOutput.joystick[2] = 0;
	uavData.receiverOutput.joystick[3] = 0;
	uavData.receiverOutput.sampleTime = 0;
	uavData.receiverOutput.vrPot = 0;

	/* Initialize sonar data */
	uavData.sonarOutput.altitude = 0;
	uavData.sonarOutput.sampleTime = 0;

	/* Initialize esc data */
	uavData.escOutput.ID[0] = 0;
	uavData.escOutput.ID[1] = 0;
	uavData.escOutput.angularSpeed[0] = 0;
	uavData.escOutput.angularSpeed[1] = 0;
	uavData.escOutput.current[0] = 0;
	uavData.escOutput.current[1] = 0;
	uavData.escOutput.rpm[0] = 0;
	uavData.escOutput.rpm[1] = 0;
	uavData.escOutput.sampleTime = 0;
	uavData.escOutput.voltage[0] = 0;
	uavData.escOutput.voltage[1] = 0;

	/* Initialize servo data */
	uavData.servosOutput.idLeft = 0;
	uavData.servosOutput.idRight = 0;
	uavData.servosOutput.servo.alphal = 0;
	uavData.servosOutput.servo.alphar = 0;
	uavData.servosOutput.servo.dotAlphal = 0;
	uavData.servosOutput.servo.dotAlphar = 0;
	uavData.servosOutput.servo_refrence.alphal = 0;
	uavData.servosOutput.servo_refrence.alphar = 0;
	uavData.servosOutput.servo_refrence.dotAlphal = 0;
	uavData.servosOutput.servo_refrence.dotAlphar = 0;

	/* Initialize attitude data */
	uavData.attitude.dotPitch = 0;
	uavData.attitude.dotRoll = 0;
	uavData.attitude.dotYaw = 0;
	uavData.attitude.pitch = 0;
	uavData.attitude.roll = 0;
	uavData.attitude.yaw = 0;

	/* Initialize attitude data */
	uavData.position.dotX = 0;
	uavData.position.dotY = 0;
	uavData.position.dotZ = 0;
	uavData.position.x = 0;
	uavData.position.y = 0;
	uavData.position.z = 0;

	/* Initialize attitude reference data */
	uavData.attitude_reference.dotPitch = 0;
	uavData.attitude_reference.dotRoll = 0;
	uavData.attitude_reference.dotYaw = 0;
	uavData.attitude_reference.pitch = 0;
	uavData.attitude_reference.roll = 0;
	uavData.attitude_reference.yaw = 0;

	/* Initialize attitude reference data */
	uavData.position_refrence.dotX = 0;
	uavData.position_refrence.dotY = 0;
	uavData.position_refrence.dotZ = 0;
	uavData.position_refrence.x = 0;
	uavData.position_refrence.y = 0;
	uavData.position_refrence.z = 0;

	uavData.cicleTime = 0;
	uavData.heartBeat = 0;
	uavData.init = 0;
	uavData.securityStop = 0;
	uavData.flightmode = 0;
	uavData.enableintegration = 0;
}

void set_uavData(pv_msg_uavData message) {
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	uavData = message;
	xSemaphoreGive(xSemaphore);
}

pv_msg_uavData get_uavData() {
	pv_msg_uavData tempData;
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	tempData = uavData;
	xSemaphoreGive(xSemaphore);
	return tempData;
}

void set_imuData(pv_type_imuOutput message) {
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	uavData.imuOutput = message;
	xSemaphoreGive(xSemaphore);
}

pv_type_imuOutput get_imuData() {
	pv_type_imuOutput tempData;
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	tempData = uavData.imuOutput;
	xSemaphoreGive(xSemaphore);
	return tempData;
}

void set_receiverData(pv_type_receiverOutput message) {
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	uavData.receiverOutput = message;
	xSemaphoreGive(xSemaphore);
}

pv_type_receiverOutput get_receiverData() {
	pv_type_receiverOutput tempData;
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	tempData = uavData.receiverOutput;
	xSemaphoreGive(xSemaphore);
	return tempData;
}

void set_sonarData(pv_type_sonarOutput message) {
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	uavData.sonarOutput = message;
	xSemaphoreGive(xSemaphore);
}

pv_type_sonarOutput get_sonarData() {
	pv_type_sonarOutput tempData;
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	tempData = uavData.sonarOutput;
	xSemaphoreGive(xSemaphore);
	return tempData;
}

void set_escData(pv_type_escOutput message) {
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	uavData.escOutput = message;
	xSemaphoreGive(xSemaphore);
}

pv_type_escOutput get_escData() {
	pv_type_escOutput tempData;
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	tempData = uavData.escOutput;
	xSemaphoreGive(xSemaphore);
	return tempData;
}

void set_servoData(pv_type_servoOutput message) {
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	uavData.servosOutput = message;
	xSemaphoreGive(xSemaphore);
}

pv_type_servoOutput get_servoData() {
	pv_type_servoOutput tempData;
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	tempData = uavData.servosOutput;
	xSemaphoreGive(xSemaphore);
	return tempData;
}

void set_attitudeData(pv_type_datapr_attitude message) {
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	uavData.attitude = message;
	xSemaphoreGive(xSemaphore);
}

pv_type_datapr_attitude get_attitudeData() {
	pv_type_datapr_attitude tempData;
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	tempData = uavData.attitude;
	xSemaphoreGive(xSemaphore);
	return tempData;
}

void set_positionData(pv_type_datapr_position message) {
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	uavData.position = message;
	xSemaphoreGive(xSemaphore);
}

pv_type_datapr_position get_positionData() {
	pv_type_datapr_position tempData;
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	tempData = uavData.position;
	xSemaphoreGive(xSemaphore);
	return tempData;
}

void set_attitudeReferenceData(pv_type_datapr_attitude message) {
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	uavData.attitude_reference = message;
	xSemaphoreGive(xSemaphore);
}

pv_type_datapr_attitude get_attitudeReferenceData() {
	pv_type_datapr_attitude tempData;
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	tempData = uavData.attitude_reference;
	xSemaphoreGive(xSemaphore);
	return tempData;
}

void set_positionReferenceData(pv_type_datapr_position message) {
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	uavData.position_refrence = message;
	xSemaphoreGive(xSemaphore);
}

pv_type_datapr_position get_positionReferenceData() {
	pv_type_datapr_position tempData;
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	tempData = uavData.position_refrence;
	xSemaphoreGive(xSemaphore);
	return tempData;
}

void set_init(bool data) {
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	uavData.init = data;
	xSemaphoreGive(xSemaphore);
}

bool get_init() {
	bool tempData;
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	tempData = uavData.init;
	xSemaphoreGive(xSemaphore);
	return tempData;
}

void set_securityStop(bool data) {
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	uavData.securityStop = data;
	xSemaphoreGive(xSemaphore);
}

bool get_securityStop() {
	bool tempData;
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	tempData = uavData.securityStop;
	xSemaphoreGive(xSemaphore);
	return tempData;
}

void set_flightmode(bool data) {
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	uavData.flightmode = data;
	xSemaphoreGive(xSemaphore);
}

bool get_flightmode() {
	bool tempData;
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	tempData = uavData.flightmode;
	xSemaphoreGive(xSemaphore);
	return tempData;
}

void set_enableintegration(bool data) {
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	uavData.enableintegration = data;
	xSemaphoreGive(xSemaphore);
}

bool get_enableintegration() {
	bool tempData;
	xSemaphoreTake(xSemaphore, 3/ portTICK_RATE_MS);
	tempData = uavData.enableintegration;
	xSemaphoreGive(xSemaphore);
	return tempData;
}

