/*
 *-----------------------------------------------------------------------------
 *  Filename:    Servo.h
 *  Implementação da classe de dados do servomotor.
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

#ifndef UAV_SERVO_INTERFACE_H
#define UAV_SERVO_INTERFACE_H

#include <boost/thread/mutex.hpp>
#include "proVantTypes.h"

/*! \brief Implementação da interface entre os modulos.
 *
 *  A interface contem um MessageQueue para cada inbox que possuir (q_in, no caso),
 *  e ponteiros para o inbox da interface com o qual ela se conecta (q_out).
 */
class Servo {
public:
	Servo()
	~Servo();

	void setServoStates(proVant::servos_state data);
	proVant::servos_state getServoStates();
private:
	boost::mutex servo_mutex;
	proVant::servos_state servos_state;
};

#endif // UAV_SERVO_INTERFACE_H
