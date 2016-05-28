/*
 *-----------------------------------------------------------------------------
 *  Filename:    Analog.cpp
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

#include "Analog.h"

Analog::Analog() {
	analog.intPowerMeterSum = 0;
	analog.amperage = 0;
	analog.rssi = 0;
	analog.vbat = 0;
}

Analog::~Analog() {

}

void Analog::setAnalog(proVant::analog data) {
	analog_mutex.lock();
	this->analog = data;
	analog_mutex.unlock();
}

proVant::analog Analog::getAnalog() {
	return this->analog;
}
