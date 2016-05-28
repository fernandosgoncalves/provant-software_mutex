/*
 *-----------------------------------------------------------------------------
 *  Filename:    CommLowLevelInterface.cpp
 *  Implementação da interface para algum modulo especifico.
 *-----------------------------------------------------------------------------
 *     ___                           _
 *    / _ \_ __ ___/\   /\__ _ _ __ | |_
 *   / /_)/ '__/ _ \ \ / / _` | '_ \| __|
 *  / ___/| | | (_) \ V / (_| | | | | |_
 *  \/    |_|  \___/ \_/ \__,_|_| |_|\__|
 *
 *-----------------------------------------------------------------------------
 *
 */

#include "UavStatesDataInterface.h"

UavStatesDataInterface::~UavStatesDataInterface() {

}

void UavStatesDataInterface::set_servo_states(proVant::servos_state data) {
	q_mutex.lock();
	this->servos_state = data;
	q_mutex.unlock();
}

void UavStatesDataInterface::set_position(proVant::position data) {
	q_mutex.lock();
	this->position = data;
	q_mutex.unlock();
}

void UavStatesDataInterface::set_altitude(proVant::altitude data) {
	q_mutex.lock();
	this->altitude = data;
	q_mutex.unlock();
}

void UavStatesDataInterface::set_attitude(proVant::attitude data) {
	q_mutex.lock();
	this->atitude = data;
	q_mutex.unlock();
}

void UavStatesDataInterface::set_escData(proVant::escData data) {
	q_mutex.lock();
	this->escData = data;
	q_mutex.unlock();
}

proVant::servos_state UavStatesDataInterface::get_servo_states() {
	proVant::servos_state temp_data;
	q_mutex.lock();
	temp_data = this->servos_state;
	q_mutex.unlock();
	return temp_data;
}

proVant::position UavStatesDataInterface::get_position() {
	proVant::position temp_data;
	q_mutex.lock();
	temp_data = this->position;
	q_mutex.unlock();
	return temp_data;
}

proVant::altitude UavStatesDataInterface::get_altitude() {
	proVant::altitude temp_data;
	q_mutex.lock();
	temp_data = this->altitude;
	q_mutex.unlock();
	return temp_data;
}

proVant::attitude UavStatesDataInterface::get_attitude() {
	proVant::attitude temp_data;
	q_mutex.lock();
	temp_data = this->atitude;
	q_mutex.unlock();
	return temp_data;
}

proVant::escData UavStatesDataInterface::get_escData() {
	proVant::escData temp_data;
	q_mutex.lock();
	temp_data = this->escData;
	q_mutex.unlock();
	return temp_data;
}
