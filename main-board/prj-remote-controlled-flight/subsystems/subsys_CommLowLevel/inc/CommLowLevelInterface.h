/*
 *-----------------------------------------------------------------------------
 *  Filename:    CommLowLevelInterface.h
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

#ifndef COMMUNICATION_LOW_LEVEL_INTERFACE_H
#define COMMUNICATION_LOW_LEVEL_INTERFACE_H

//Father
#include "AbstractMessageInterface.h"

#include "proVantTypes.h"
#include "RcNormalize.h"
#include "Attitude.h"
#include "Position.h"
#include "Status.h"
#include "Servo.h"
#include "Debug.h"

/*! \brief Implementação da interface entre os modulos.
 *
 *  A interface contem um MessageQueue para cada inbox que possuir (q_in, no caso),
 *  e ponteiros para o inbox da interface com o qual ela se conecta (q_out).
 */
class CommLowLevelInterface : public AbstractMessageInterface
{
public:
    CommLowLevelInterface(std::string name) :
    //q_position_out_(NULL),
	//q_atitude_out_(NULL),
	//q_servos_out_(NULL),
	//q_debug_out_(NULL),
	//q_rc_out_(NULL),
	//q_position2_out_(NULL),
	//q_atitude2_out_(NULL),
	//q_servos2_out_(NULL),
	//q_debug2_out_(NULL),
	//q_rc2_out_(NULL),
    name_(name) { }

    ~CommLowLevelInterface();

    // Outboxes (ponteiros para inboxes alheios)
    //ControlOutput Producing
    Attitude q_attitude_out_;
    //MsgQueue<proVant::attitude>* q_atitude_out_;
    Position q_position_out_;
    //MsgQueue<proVant::position>* q_position_out_;
    Servo q_servos_out_;
    //MsgQueue<proVant::servos_state>* q_servos_out_;
    Debug q_debug_out_;
    //MsgQueue<proVant::debug>* q_debug_out_;
    RcNormalize q_rc_out_;
    //MsgQueue<proVant::rcNormalize>* q_rc_out_;
    Status q_status_out_;
    //MsgQueue<proVant::status>* q_status_out_;

    // CommLowLevelManager consuming
    MsgQueue<proVant::controlOutput> q_actuation_in;

    //Dataprocesing Producing
    //MsgQueue<proVant::attitude>* q_atitude2_out_;
    //MsgQueue<proVant::position>* q_position2_out_;
    //MsgQueue<proVant::servos_state>* q_servos2_out_;
    //MsgQueue<proVant::debug>* q_debug2_out_;
    //MsgQueue<proVant::rcNormalize>* q_rc2_out_;
    //MsgQueue<proVant::status>* q_status2_out_;
    //MsgQueue<proVant::controlOutput>* q_actuation2_out_;

private:
    std::string name_;

};

#endif // DATA_PROCESSING_INTERFACE_H
