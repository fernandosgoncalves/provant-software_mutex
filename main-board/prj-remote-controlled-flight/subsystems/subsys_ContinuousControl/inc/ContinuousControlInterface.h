/*
 *-----------------------------------------------------------------------------
 *  Filename:    ContinuousControlInterface.h
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

#ifndef CONTINUOUS_CONTROL_INTERFACE_H
#define CONTINUOUS_CONTROL_INTERFACE_H

//Father
#include "AbstractMessageInterface.h"
#include "RcNormalize.h"
#include "Actuation.h"
#include "Attitude.h"
#include "Position.h"
#include "Status.h"
#include "Servo.h"
#include "Debug.h"

#include "proVantTypes.h"

/*! \brief Implementação da interface entre os modulos.
 *
 *  A interface contem um MessageQueue para cada inbox que possuir (q_in, no caso),
 *  e ponteiros para o inbox da interface com o qual ela se conecta (q_out).
 */
class ContinuousControlInterface : public AbstractMessageInterface
{
public:
    ContinuousControlInterface(std::string name) :
        //q_actuation_out_(NULL),
		//q_actuation2_out_(NULL),
        name_(name) { }

    ~ContinuousControlInterface();

    // Inboxes
    Attitude q_attitude_in;
    //MsgQueue<proVant::attitude> q_atitude_in;
    Position q_position_in;
    //MsgQueue<proVant::position> q_position_in;
    Servo q_servos_in;
    //MsgQueue<proVant::servos_state> q_servos_in;
    Debug q_debug_in;
    //MsgQueue<proVant::debug> q_debug_in;
    RcNormalize q_rc_in;
    //MsgQueue<proVant::rcNormalize> q_rc_in;
    Status q_status_in;
    //MsgQueue<proVant::status> q_status_in;

    // Outboxes (ponteiros para inboxes alheios)
    Actuation q_actuation_out_;
    //MsgQueue<proVant::controlOutput>* q_actuation_out_;
    //MsgQueue<proVant::controlOutput>* q_actuation2_out_;

private:
    std::string name_;

};

#endif // DATA_PROCESSING_INTERFACE_H
