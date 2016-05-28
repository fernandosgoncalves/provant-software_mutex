/*
 *-----------------------------------------------------------------------------
 *  Filename:    Altitude.cpp
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

#include "Altitude.h"

Altitude::Altitude() {
	altitude.estAlt = 0;
	altitude.vario = 0;
}

Altitude::~Altitude() {

}

void Altitude::setAltitude(proVant::altitude data) {
	altitude_mutex.lock();
	this->altitude = data;
	altitude_mutex.unlock();
}

proVant::altitude Altitude::getAltitude() {
	return this->altitude;
}
