/*
 *-----------------------------------------------------------------------------
 *  Filename:    Actuation.cpp
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

#include "Actuation.h"

Actuation::Actuation(){
	actuation.escRightNewtons = 0;
	actuation.escLeftNewtons = 0;
	actuation.escRightSpeed = 0;
	actuation.escLeftSpeed = 0;
	actuation.servoRight = 0;
	actuation.servoLeft = 0;
}

Actuation::~Actuation(){

}

void Actuation::setActuation(proVant::controlOutput data){
	actuation_mutex.lock();
	this->actuation = data;
	actuation_mutex.unlock();
}

proVant::controlOutput Actuation::getActuation(){
	return this->actuation;
}
