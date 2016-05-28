/*
 *-----------------------------------------------------------------------------
 *  Filename:    Attitude.h
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

#ifndef UAV_ATTITUDE_INTERFACE_H
#define UAV_ATTITUDE_INTERFACE_H

#include <boost/thread/mutex.hpp>
#include "proVantTypes.h"

/*! \brief Implementação da interface entre os modulos.
 *
 *  A interface contem um MessageQueue para cada inbox que possuir (q_in, no caso),
 *  e ponteiros para o inbox da interface com o qual ela se conecta (q_out).
 */
class Attitude{
public:
	Attitude();
	~Attitude();

	void setAttitude(proVant::attitude data);
	proVant::attitude getAttitude();
private:
	boost::mutex attitude_mutex;
	proVant::attitude attitude;
};

#endif // UAV_ATTITUDE_INTERFACE_H
