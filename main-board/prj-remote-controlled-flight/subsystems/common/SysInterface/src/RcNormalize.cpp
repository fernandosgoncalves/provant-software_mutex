/*
 *-----------------------------------------------------------------------------
 *  Filename:    RcNormalize.cpp
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

#include "RcNormalize.h"

RcNormalize::RcNormalize() {
	rcNormalize.normChannels[0] = 0;
	rcNormalize.normChannels[1] = 0;
	rcNormalize.normChannels[2] = 0;
	rcNormalize.normChannels[3] = 0;
	rcNormalize.normChannels[4] = 0;
	rcNormalize.normChannels[5] = 0;
	rcNormalize.normChannels[6] = 0;
}

RcNormalize::~RcNormalize() {

}

void RcNormalize::setRcNormalize(proVant::rcNormalize data) {
	rcNormalize_mutex.lock();
	this->rcNormalize = data;
	rcNormalize_mutex.unlock();
}

proVant::rcNormalize RcNormalize::getRcNormalize() {
	return this->rcNormalize;
}
