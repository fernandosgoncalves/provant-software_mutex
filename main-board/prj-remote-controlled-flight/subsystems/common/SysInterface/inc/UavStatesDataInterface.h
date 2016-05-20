/*
 *-----------------------------------------------------------------------------
 *  Filename:    UavSatateslInterface.h
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

/*** Renomear .cpp e o .h com o nome do módulo + "Interface" (ex. DataProcessingInterface) ***/

#ifndef UAV_STATES_DATA_INTERFACE_H
#define UAV_STATES_DATA_INTERFACE_H

//Father
#include "AbstractDataInterface.h"

#include "proVantTypes.h"

/*! \brief Implementação da interface entre os modulos.
 *
 *  A interface contem um MessageQueue para cada inbox que possuir (q_in, no caso),
 *  e ponteiros para o inbox da interface com o qual ela se conecta (q_out).
 */
class UavStatesDataInterface: public AbstractDataInterface {
public:
	UavStatesDataInterface(std::string name) :
			//servos_state(0),
			//position(0),
		    //altitude(0),
		    //atitude(0),
		    //escData(0),
		    name_(name) {
	}

	~UavStatesDataInterface();

	void set_servo_states(proVant::servos_state data);
	void set_position(proVant::position data);
	void set_altitude(proVant::altitude data);
	void set_attitude(proVant::atitude data);
	proVant::servos_state get_servo_states();
	void set_escData(proVant::escData data);
	proVant::position get_position();
	proVant::altitude get_altitude();
	proVant::atitude get_attitude();
	proVant::escData get_escData();

private:
	proVant::servos_state servos_state;
	proVant::position position;
	proVant::altitude altitude;
	std::string name_;
	proVant::atitude atitude;
	proVant::escData escData;
};

#endif // UAV_STATES_DATA_INTERFACE_H
