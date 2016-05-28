/*
 *-----------------------------------------------------------------------------
 *  Filename:    Attitude.cpp
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

#include "Attitude.h"

Attitude::Attitude() {
	attitude.dotPitch = 0;
	attitude.dotRoll = 0;
	attitude.dotYaw = 0;
	attitude.pitch = 0;
	attitude.roll = 0;
	attitude.yaw = 0;
}

Attitude::~Attitude() {

}

void Attitude::setAttitude(proVant::attitude data) {
	attitude_mutex.lock();
	this->attitude = data;
	attitude_mutex.unlock();
}

proVant::attitude Attitude::getAttitude() {
	return this->attitude;
}
