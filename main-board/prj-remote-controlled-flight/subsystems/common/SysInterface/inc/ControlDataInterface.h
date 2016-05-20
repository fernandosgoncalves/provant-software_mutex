/*
 *-----------------------------------------------------------------------------
 *  Filename:    ControlDataInterface.h
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

/*** Renomear .cpp e o .h com o nome do módulo + "Interface" (ex. DataProcessingInterface) ***/

#ifndef CONTROL_DATA_INTERFACE_H
#define CONTROL_DATA_INTERFACE_H

//Father
#include "AbstractDataInterface.h"

#include "../../Provant/inc/proVantTypes.h"

/*! \brief Implementação da interface entre os modulos.
 *
 *  A interface contem um MessageQueue para cada inbox que possuir (q_in, no caso),
 *  e ponteiros para o inbox da interface com o qual ela se conecta (q_out).
 */
class ControlDataInterface: public AbstractDataInterface {
public:
	ControlDataInterface(std::string name) :
	//controlOutput(0),
	name_(name) {
	}

	~ControlDataInterface();
	void set_controlOutput(proVant::controlOutput data);
	proVant::controlOutput get_controlOutput();

private:
	std::string name_;
	proVant::controlOutput controlOutput;

};

#endif // DATA_PROCESSING_INTERFACE_H
