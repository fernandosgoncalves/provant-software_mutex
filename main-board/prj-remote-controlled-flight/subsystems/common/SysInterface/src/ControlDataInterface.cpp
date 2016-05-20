/*
 *-----------------------------------------------------------------------------
 *  Filename:    ControlDataInterface.cpp
 *  Implementação da interface para algum modulo especifico.
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

#include "ControlDataInterface.h"

ControlDataInterface::~ControlDataInterface()
{

}

void ControlDataInterface::set_controlOutput(proVant::controlOutput data){
	q_mutex.lock();
	this->controlOutput = data;
	q_mutex.unlock();
}

proVant::controlOutput ControlDataInterface::get_controlOutput(){
	proVant::controlOutput temp_data;
	q_mutex.lock();
	temp_data = this->controlOutput;
	q_mutex.unlock();
	return temp_data;
}

