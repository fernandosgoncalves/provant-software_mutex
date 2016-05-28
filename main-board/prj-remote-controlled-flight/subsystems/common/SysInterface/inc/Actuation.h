/*
 *-----------------------------------------------------------------------------
 *  Filename:    Actuation.h
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

#ifndef CONTROL_ACTUATION_INTERFACE_H
#define CONTROL_ACTUATION_INTERFACE_H

#include <boost/thread/mutex.hpp>
#include "proVantTypes.h"

/*! \brief Implementação da interface entre os modulos.
 *
 *  A interface contem um MessageQueue para cada inbox que possuir (q_in, no caso),
 *  e ponteiros para o inbox da interface com o qual ela se conecta (q_out).
 */
class Actuation{
public:
	Actuation();
	~Actuation();
	void setActuation(proVant::controlOutput data);
	proVant::controlOutput getActuation();

private:
	boost::mutex actuation_mutex;
	proVant::controlOutput actuation;

};

#endif // CONTROL_ACTUATION_INTERFACE_H
