/*
 *-----------------------------------------------------------------------------
 *  Filename:    SystemDataInterface.cpp
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

#include "SystemDataInterface.h"

SystemDataInterface::~SystemDataInterface() {

}

void SystemDataInterface::set_analog(proVant::analog data) {
	q_mutex.lock();
	this->analog = data;
	q_mutex.unlock();
}

proVant::analog SystemDataInterface::get_analog() {
	proVant::analog temp_data;
	q_mutex.lock();
	temp_data = this->analog;
	q_mutex.unlock();
	return temp_data;
}

void SystemDataInterface::set_status(proVant::status data) {
	q_mutex.lock();
	this->status = data;
	q_mutex.unlock();
}

proVant::status SystemDataInterface::get_status() {
	proVant::status temp_data;
	q_mutex.lock();
	temp_data = this->status;
	q_mutex.unlock();
	return temp_data;
}

void SystemDataInterface::set_rcNormalize(proVant::rcNormalize data) {
	q_mutex.lock();
	this->rcNormalize = data;
	q_mutex.unlock();
}

proVant::rcNormalize SystemDataInterface::get_rcNormalize() {
	proVant::rcNormalize temp_data;
	q_mutex.lock();
	temp_data = this->rcNormalize;
	q_mutex.unlock();
	return temp_data;
}

void SystemDataInterface::set_ident(proVant::ident data) {
	q_mutex.lock();
	this->ident = data;
	q_mutex.unlock();
}

proVant::ident SystemDataInterface::get_ident(){
	proVant::ident temp_data;
	q_mutex.lock();
	temp_data = this->ident;
	q_mutex.unlock();
	return temp_data;
}

void SystemDataInterface::set_debug(proVant::debug data){
	q_mutex.lock();
	this->debug = data;
	q_mutex.unlock();
}

proVant::debug SystemDataInterface::get_debug(){
	proVant::debug temp_data;
	q_mutex.lock();
	temp_data = this->debug;
	q_mutex.unlock();
	return temp_data;
}
