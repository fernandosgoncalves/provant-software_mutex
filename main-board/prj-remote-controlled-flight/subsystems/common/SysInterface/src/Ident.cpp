/*
 *-----------------------------------------------------------------------------
 *  Filename:    Ident.cpp
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

#include "Ident.h"

Ident::Ident() {
	ident.capability = 0;
	ident.mspVersion = 0;
	ident.multitype = 0;
	ident.version = 0;
}

Ident::~Ident() {

}

void Ident::setIdent(proVant::ident data) {
	ident_mutex.lock();
	this->ident = data;
	ident_mutex.unlock();
}

proVant::ident Ident::getIdent(){
	return this->ident;
}
