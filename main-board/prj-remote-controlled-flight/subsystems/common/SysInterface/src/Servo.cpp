/*
 *-----------------------------------------------------------------------------
 *  Filename: Servo.cpp
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

#include "Servo.h"

Servo::Servo(){
	servos_state.dotAlphal = 0;
	servos_state.dotAlphar = 0;
	servos_state.alphal = 0;
	servos_state.alphar = 0;
}

Servo::~Servo() {

}

void Servo::setServoStates(proVant::servos_state data) {
	servo_mutex.lock();
	this->servos_state = data;
	servo_mutex.unlock();
}

proVant::servos_state Servo::getServoStates() {
	return this->servos_state;
}
