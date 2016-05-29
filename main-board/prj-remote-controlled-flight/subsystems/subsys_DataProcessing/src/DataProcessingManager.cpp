/*
 *-----------------------------------------------------------------------------
 *  Filename:    DataProcessingManager.cpp
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

#include "DataProcessingManager.h"

//Internal
#include "debug.h"
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <chrono>

using namespace std;

DataProcessingManager::DataProcessingManager(std::string name) :
    interface(new DataProcessingInterface("DataProcessing:Interface")),
    // sm1(new SubModule1), // talvez fosse mais interessante construir os submodulos no init
    ms_sample_time(12),
    name_(name)
{

}

DataProcessingManager::~DataProcessingManager()
{

}

void DataProcessingManager::Init()
{
	PROVANT2.init("/dev/ttyO2", 460800);
	DEBUG(LEVEL_INFO, "Connected 0");
	Log();
    /// Linkando os submodulos à interface do modulo
    // sm1->interface = interface;

    /// Conectar os submodulos, (caso eles se comuniquem por mensagem)
    /// ...


    /// Neste caso, conectar a notificacao da interface com o callback da classe
    // interface->q_in.notification.connect( boost::bind(&DataProcessingManager::inboxCallback, this) );

}

void DataProcessingManager::Log() {
	char file[20];
	int fcounter;
	for (fcounter = 0; true; fcounter++) {
		sprintf(file, "log/dp%d.txt\0", fcounter);
		DATAPROSS = fopen(file, "r");
		if (DATAPROSS != NULL)
			fclose(DATAPROSS);
		else
			break;
	}
	strcpy(this->file, file);
	DATAPROSS = fopen(file, "w");
	time_t rawtime;
	struct tm * timeinfo;

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	// log file header
	fprintf(DATAPROSS, "%% Start date/time : %s", asctime(timeinfo));
	//fprintf(SIMfp,
	//		"\n%%X	Y	Z	dotX	dotY	dotZ	Phi	Theta	Psi	dotPhi	dotTheta	dotPsi");
	fclose(DATAPROSS);

	printf("\nLog configuration sucessfull! Using file %s.\n", file);
}

void DataProcessingManager::Run()
{
    Init();
    int count = 0;
    int times[1000];
    // Algumas variaveis...
    proVant::controlOutput actuation;
    proVant::servos_state servos;
    proVant::position position;
    proVant::attitude atitude;
    proVant::rcNormalize rc;
    proVant::debug debug;

    int16_t normChannels[7]={};
    float servoTorque[2]={};
    float trajectory[3]={};
    float escForce[2]={};
    float velocity[3]={};
    float dalpha[2]={};
    float debugv[4]={};
    float alpha[2]={};
    float drpy[3]={};
    float aux2[3]={};
    float rpy[3]={};
    int aux[2]={};
    int i = 0;

    // Loop principal!
    while(1) {
    	start= boost::chrono::system_clock::now();
    	auto sample_time = boost::chrono::duration_cast<boost::chrono::microseconds>(start-last_start);
    	last_start=start;

    	atitude = interface->q_attitude_in.getAttitude();
    	//atitude = uavStatesDataInterface->get_attitude();
    	/*Atitude*/
    	drpy[1]=atitude.dotPitch;
    	drpy[0]=atitude.dotRoll;
   		drpy[2]=atitude.dotYaw;
    	rpy[1]=atitude.pitch;
    	rpy[0]=atitude.roll;
    	rpy[2]=atitude.yaw;

    	position = interface->q_position_in.getPosition();
    	//position = uavStatesDataInterface->get_position();
    	/*Position*/
    	velocity[0]=position.dotX;
        velocity[1]=position.dotY;
        velocity[2]=position.dotZ;
    	trajectory[0]=position.x;
    	trajectory[1]=position.y;
    	trajectory[2]=position.z;

    	servos = interface->q_servos_in.getServoStates();
    	//servos = uavStatesDataInterface->get_servo_states();
    	/*Servos*/
    	dalpha[0]=servos.dotAlphal;
        dalpha[1]=servos.dotAlphar;
    	alpha[0]=servos.alphal;
    	alpha[1]=servos.alphar;

    	actuation = interface->q_actuation_in.getActuation();
    	//actuation = controlDataInterface->get_controlOutput();
   		/*Control*/
   		escForce[1]=actuation.escRightNewtons;
   		escForce[0]=actuation.escLeftNewtons;
    	servoTorque[0]=-actuation.servoLeft;
   		servoTorque[1]=actuation.servoRight;
   		aux2[1]=actuation.escRightSpeed;
   		aux2[0]=actuation.escLeftSpeed;

   		debug = interface->q_debug_in.getDebug();
    	//debug = systemDataInterface->get_debug();
    	/*Debug*/
    	debugv[0]=debug.debug[0];
    	debugv[1]=debug.debug[1];
    	debugv[2]=debug.debug[2];
    	debugv[3]=debug.debug[3];

    	rc = interface->q_rc_in.getRcNormalize();
    	//rc = systemDataInterface->get_rcNormalize();
   		/*Debug*/
   		normChannels[0]=rc.normChannels[0];
   		normChannels[1]=rc.normChannels[1];
   		normChannels[2]=rc.normChannels[2];
   		normChannels[3]=rc.normChannels[3];
   		normChannels[4]=rc.normChannels[4];
   		normChannels[5]=rc.normChannels[5];
   		normChannels[6]=rc.normChannels[6];

    	PROVANT2.multwii_attitude(rpy[0]*RAD_TO_DEG,rpy[1]*RAD_TO_DEG,rpy[2]*RAD_TO_DEG);
    	PROVANT2.multwii2_sendControldatain(rpy,drpy,trajectory,velocity);
    	PROVANT2.multwii_sendstack();
    	PROVANT2.multwii2_sendEscdata(aux,alpha,dalpha);
    	PROVANT2.multwii2_sendControldataout(servoTorque,escForce,aux2);
    	PROVANT2.multwii_sendstack();
    	//PROVANT2.multwii2_rcNormalize(normChannels);
    	PROVANT2.multwii_debug((float)debugv[0],(float)debugv[1],(float)debugv[2],(float)debugv[3]);
    	PROVANT2.multwii_sendstack();

    	//std::cout << "sp_thread3: " << sample_time.count()<< " microseconds." << std::endl;
    	i++;
    	auto elapsed = boost::chrono::duration_cast<boost::chrono::microseconds>(boost::chrono::system_clock::now()-start);

    	if(count < 1000){
    		//std::cout << "counting " << count << std::endl;
    		//times[count] = sample_time.count();
    		times[count] = elapsed.count();
    		count++;
    	}else{
    		std::cout << "save th3" << std::endl;
    		this->DATAPROSS = fopen(this->file,"w");
    		for(int x=0;x<1000;x++){
    			fprintf(this->DATAPROSS, "%d\n",times[x]);
    		}
    		fclose(this->DATAPROSS);
    		count = 0;
    	}

    	boost::this_thread::sleep_until(boost::chrono::system_clock::now() + boost::chrono::microseconds((ms_sample_time*1000)-elapsed.count()));
    }
}

void DataProcessingManager::inboxCallback()
{
    DEBUG(LEVEL_INFO, "Got message! ") << name_;
}
