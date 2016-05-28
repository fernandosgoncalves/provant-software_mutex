/*
 *-----------------------------------------------------------------------------
 *  Filename:    ESC.h
 *  Implementação da interface dos estados do VANT.
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

#ifndef UAV_ESC_INTERFACE_H
#define UAV_ESC_INTERFACE_H

#include <boost/thread/mutex.hpp>
#include "proVantTypes.h"

/*! \brief Implementação da interface entre os modulos.
 *
 *  A interface contem um MessageQueue para cada inbox que possuir (q_in, no caso),
 *  e ponteiros para o inbox da interface com o qual ela se conecta (q_out).
 */
class ESC {
public:
	ESC();
	~ESC();

	void setEscData(proVant::escData data);
	proVant::escData getEscData();

private:
	boost::mutex esc_mutex;
	proVant::escData escData;
};

#endif // UAV_STATES_DATA_INTERFACE_H
