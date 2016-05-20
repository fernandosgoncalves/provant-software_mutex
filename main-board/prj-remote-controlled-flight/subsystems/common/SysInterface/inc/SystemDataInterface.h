/*
 *-----------------------------------------------------------------------------
 *  Filename:    SystemDataInterface.h
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

#ifndef SYSTEM_DATA_INTERFACE_H
#define SYSTEM_DATA_INTERFACE_H

//Father
#include "AbstractDataInterface.h"

#include "../../Provant/inc/proVantTypes.h"

/*! \brief Implementação da interface entre os modulos.
 *
 *  A interface contem um MessageQueue para cada inbox que possuir (q_in, no caso),
 *  e ponteiros para o inbox da interface com o qual ela se conecta (q_out).
 */
class SystemDataInterface: public AbstractDataInterface {
public:
	SystemDataInterface(std::string name) :
			//analog(0),
			//status(0),
			//rcNormalize(0),
			//ident(0),
			//debug(0),
			name_(name) {
	}

	~SystemDataInterface();
	void set_rcNormalize(proVant::rcNormalize data);
	proVant::rcNormalize get_rcNormalize();
	void set_analog(proVant::analog data);
	void set_status(proVant::status data);
	void set_ident(proVant::ident data);
	void set_debug(proVant::debug data);
	proVant::analog get_analog();
	proVant::status get_status();
	proVant::debug get_debug();
	proVant::ident get_ident();

private:
	proVant::rcNormalize rcNormalize;
	std::string name_;
	proVant::analog analog;
	proVant::status status;
	proVant::ident ident;
	proVant::debug debug;

};

#endif // DATA_PROCESSING_INTERFACE_H
