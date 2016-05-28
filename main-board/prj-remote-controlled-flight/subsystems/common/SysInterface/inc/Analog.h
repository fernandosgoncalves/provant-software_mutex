/*
 *-----------------------------------------------------------------------------
 *  Filename:    Analog.h
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

#ifndef SYSTEM_ANALOG_INTERFACE_H
#define SYSTEM_ANALOG_INTERFACE_H

#include <boost/thread/mutex.hpp>
#include "proVantTypes.h"

/*! \brief Implementação da interface entre os modulos.
 *
 *  A interface contem um MessageQueue para cada inbox que possuir (q_in, no caso),
 *  e ponteiros para o inbox da interface com o qual ela se conecta (q_out).
 */
class Analog {
public:
	Analog;
	~Analog();
	void setAnalog(proVant::analog data);
	proVant::analog getAnalog();
private:
	boost::mutex analog_mutex;
	proVant::analog analog;
};

#endif //SYSTEM_ANALOG_INTERFACE_H
