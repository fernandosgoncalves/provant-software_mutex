/*
 *-----------------------------------------------------------------------------
 *  Filename:    CommLowLevelataManager.cpp
 *  Implementação do gerenciador e instanciador de submodulos.
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

#include "ContinuousControlManager.h"
#include "proVantTypes.h"

//Internal
#include "debug.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

using namespace Eigen;
using namespace loadmodel;
using namespace std;

ContinuousControlManager::ContinuousControlManager(std::string name) :
    interface(new ContinuousControlInterface("ContinuousControl:Interface")),
    // sm1(new SubModule1), // talvez fosse mais interessante construir os submodulos no init
    ms_sample_time(12),
    name_(name)
{
	//mpc=new MPC::MpcControler();
	//mpcload=new MPCLOAD::MpcLoad();
	//mpcbirotor=new MPCBirotor::MpcBirotor();
	//mpc=new MPC::MpcControler();
	lqr=new LQR::LQRControler();
	//test= new TEST::TESTActuator();
}

ContinuousControlManager::~ContinuousControlManager()
{

}

void ContinuousControlManager::Init()
{
	Log();
    //PROVANT.init("/dev/ttyUSB0", 460800);
    //DEBUG(LEVEL_INFO, "Connected");

    // DEBUG(DEBUG_INFO, "Initializing: ") << __func__ ;

    /// Linkando os submodulos à interface do modulo
    // sm1->interface = interface;

    /// Conectar os submodulos, (caso eles se comuniquem por mensagem)
    /// ...

    /// Neste caso, conectar a notificacao da interface com o callback da classe
    // interface->q_in.notification.connect( boost::bind(&DataProcessingManager::inboxCallback, this) );

}

void ContinuousControlManager::Log() {
	char file[20];
	int fcounter;
	for (fcounter = 0; true; fcounter++) {
		sprintf(file, "log/cc%d.txt\0", fcounter);
		CONTCOM = fopen(file, "r");
		if (CONTCOM != NULL)
			fclose(CONTCOM);
		else
			break;
	}
	strcpy(this->file, file);
	CONTCOM = fopen(file, "w");
	time_t rawtime;
	struct tm * timeinfo;

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	// log file header
	fprintf(CONTCOM, "%% Start date/time : %s", asctime(timeinfo));
	//fprintf(SIMfp,
	//		"\n%%X	Y	Z	dotX	dotY	dotZ	Phi	Theta	Psi	dotPhi	dotTheta	dotPsi");
	fclose(CONTCOM);

	printf("\nLog configuration sucessfull! Using file %s.\n", file);
}

void ContinuousControlManager::Run()
{
    Init();
    int count = 0;
    int times[1000];
    // Algumas variaveis... 
    proVant::controlOutput actuation;
    proVant::rcNormalize rcNormalize;
    proVant::servos_state servos;
    proVant::position position;
    proVant::attitude attitude;
    proVant::status status;
    proVant::debug debug;
    int i=0;

    // Matrix class
    MatrixXf xs(16,1);
    Matrix'Xf ref(3,1);
    Matrix'Xf channels(4,1);
    Matrix'Xf u(4,1);

    start = boost::chrono::system_clock::now();
    last_start = boost::chrono::system_clock::now();

    for (int j=0;j<7;j++)
    	rcNormalize.normChannels[j]=0;
    // Loop principal!

    while(1) {
    	start= boost::chrono::system_clock::now();
    	auto sample_time = boost::chrono::duration_cast<boost::chrono::microseconds>(start-last_start);
    	last_start=start;

    	attitude = interface->q_attitude_in.getAttitude();
    	//atitude = uavStatesDataInterface->get_attitude();
    	position = interface->q_position_in.getPosition();
    	//position = uavStatesDataInterface->get_position();
    	servos = interface->q_servos_in.getServoStates();
    	//servos = uavStatesDataInterface->get_servo_states();
    	debug = interface->q_debug_in.getDebug();
    	//debug = systemDataInterface->get_debug();
    	rcNormalize = interface->q_rc_in.getRcNormalize();
    	//rcNormalize = systemDataInterface->get_rcNormalize();
    	status = interface->q_status_in.getStatus();
    	//status = systemDataInterface->get_status();

    	channels.setZero();
    	channels<<rcNormalize.normChannels[0],rcNormalize.normChannels[1],rcNormalize.normChannels[2],rcNormalize.normChannels[3];
    	xs.setZero();
    	xs<<position.x,position.y,position.z,atitude.roll,atitude.pitch,atitude.yaw,servos.alphar,servos.alphal
    			,position.dotX,position.dotY,position.dotZ,atitude.dotRoll,atitude.dotPitch,atitude.dotYaw,servos.dotAlphar,servos.dotAlphal;
    	ref<<(float)debug.debug[0]/100.0,(float)debug.debug[1]/100.0,(float)debug.debug[2]/100.0;

    	//std::cout<<ref<<std::endl;
    	//u=mpc->Controler(xs);

    	u=lqr->Controler(xs,ref,status.stop);
    	//u=mpcload->Controler(xs);
    	//u=mpcbirotor->Controler(xs,status.stop);
    	//u=test->Controler(channels);

    	/////////////////////////////////
    	//std::cout<<u<<std::endl;
    	actuation.escRightNewtons=u(0,0);
    	actuation.escLeftNewtons=u(1,0);
    	actuation.servoRight=u(2,0);
    	actuation.servoLeft=u(3,0);
    	actuation.escLeftSpeed=0;
    	actuation.escRightSpeed=0;

    	interface->q_actuation_out_.setActuation(actuation);
    	//controlDataInterface->set_controlOutput(actuation);

    	//std::cout << "sp_thread2: " << sample_time.count()<< " microseconds." << std::endl;
    	i++;
    	auto elapsed = boost::chrono::duration_cast<boost::chrono::microseconds>(boost::chrono::system_clock::now()-start);

    	if(count < 1000){
    		//std::cout << "counting " << count << std::endl;
    		//times[count] = sample_time.count();
    		times[count] = elapsed.count();
    		count++;
    	}else{
    		std::cout << "save th2" << std::endl;
    		this->CONTCOM = fopen(this->file,"w");
    		for(int x=0;x<1000;x++){
    			fprintf(this->CONTCOM, "%d\n",times[x]);
    		}
    		fclose(this->CONTCOM);
    		count = 0;
    	}

    	boost::this_thread::sleep_until(boost::chrono::system_clock::now() + boost::chrono::microseconds((ms_sample_time*1000)-elapsed.count()));
    }
}

void ContinuousControlManager::inboxCallback()
{
    DEBUG(LEVEL_INFO, "Got message! ") << name_;
}
