/*
 *-----------------------------------------------------------------------------
 *  Filename:    ESC.cpp
 *
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

#include "ESC.h"

ESC::ESC() {
	escData.current[0] = 0;
	escData.current[1] = 0;
	escData.voltage[0] = 0;
	escData.voltage[1] = 0;
	escData.rpm[0] = 0;
	escData.rpm[1] = 0;
}

ESC::~ESC() {

}

void ESC::setEscData(proVant::escData data) {
	esc_mutex.lock();
	this->escData = data;
	esc_mutex.unlock();
}

proVant::escData ESC::getEscData() {
	return this->escData;
}
