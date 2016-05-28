/*
 *-----------------------------------------------------------------------------
 *  Filename:    Status.h
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

#ifndef SYSTEM_STATUS_INTERFACE_H
#define SYSTEM_STATUS_INTERFACE_H

#include <boost/thread/mutex.hpp>
#include "proVantTypes.h"

/*! \brief Implementação da interface entre os modulos.
 *
 *  A interface contem um MessageQueue para cada inbox que possuir (q_in, no caso),
 *  e ponteiros para o inbox da interface com o qual ela se conecta (q_out).
 */
class Status {
public:
	Status();
	~Status();
	void setStatus(proVant::status data);
	proVant::status getStatus();
private:
	boost::mutex status_mutex;
	proVant::status status;
};

#endif // SYSTEM_STATUS_INTERFACE_H
