/*
 *-----------------------------------------------------------------------------
 *  Filename:    Debug.cpp
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

#include "Debug.h"

Debug::Debug() {
	debug.debug[0] = 0;
	debug.debug[1] = 0;
	debug.debug[2] = 0;
	debug.debug[3] = 0;
}

Debug::~Debug() {

}

void Debug::setDebug(proVant::debug data){
	debug_mutex.lock();
	this->debug = data;
	debug_mutex.unlock();
}

proVant::debug Debug::getDebug(){
	return this->debug;
}
