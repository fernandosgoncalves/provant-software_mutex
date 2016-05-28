/*
 *-----------------------------------------------------------------------------
 *  Filename:    Position.cpp
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

#include "Position.h"

Position::Position() {
	position.dotX = 0;
	position.dotY = 0;
	position.dotZ = 0;
	position.x = 0;
	position.y = 0;
	position.z = 0;
}

Position::~Position() {

}

void Position::setPosition(proVant::position data) {
	position_mutex.lock();
	this->position = data;
	position_mutex.unlock();
}

proVant::position Position::getPosition() {
	return this->position;
}
