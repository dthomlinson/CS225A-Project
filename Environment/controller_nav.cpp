// This controller moves the two arms and the body using posori controllers

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <fstream>
#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/Divebot_Hybrid.urdf";

#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1

int state = POSORI_CONTROLLER;

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;

//read
// std::string OBJ_POS_KEY = "sai2::cs225a::project::rob_pos";
 


// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;
bool printJ = true;

//Max torque limits for all oceanone joints
VectorXd maxTorque(VectorXd torque) {
	VectorXd maxjointtorques(20);
	maxjointtorques << 100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0,100.0;
	// maxjointtorques << 10.0,1.0,1.0,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1;
	for (int i=0;i<20;i++) {
		if(abs(torque[i])>maxjointtorques[i]){
			if (torque[i]>0.0)
			torque[i]=maxjointtorques[i];
			else
			torque[i]=-maxjointtorques[i];
		}
	}
	return torque;
}

int main() {

	JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
	JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
	JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();


	// prepare controller
	int dof = robot->dof();
	// std::cout << "degrees of freedom:" << dof; // 20 for oceanone
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);
	VectorXd sat_pos = VectorXd::Zero(3);
	sat_pos << 0,0.5,0; //1,0.5,0
	// redis_client.setEigenMatrixJSON(OBJ_POS_KEY, sat_pos);

	// pose task

	VectorXd docking_position = VectorXd::Zero(dof);
	docking_position=initial_q;

	const string control_link_body = "Body";
	const Vector3d control_point_body = Vector3d(0,0,0.0);
	auto posori_task_body = new Sai2Primitives::PosOriTask(robot, control_link_body, control_point_body);

#ifdef USING_OTG
	posori_task_body->_use_interpolation_flag = true;
#else
	posori_task_body->_use_velocity_saturation_flag = true;
#endif
	

	VectorXd posori_task_torques_body = VectorXd::Zero(dof);
	posori_task_body->_kp_pos = 200.0;
	posori_task_body->_kv_pos = 20.0;
	posori_task_body->_kp_ori = 200.0;
	posori_task_body->_kv_ori = 20.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
	joint_task->_use_interpolation_flag = true;
#else
	joint_task->_use_velocity_saturation_flag = true;
#endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 250.0;
	joint_task->_kv = 15.0;

	VectorXd q_init_desired = initial_q;
	// q_init_desired << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	//q_init_desired.setZero();
	//q_init_desired *= M_PI/180.0;
	joint_task->_desired_position = q_init_desired; // may need to chage

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	//setDynamicDecouplingNone();


	ofstream myfile ("data.txt");
	if ( ! myfile.is_open())
    	{
        	cerr << "error opening file";
        	exit (1);
    	}
	N_prec.setIdentity();
	posori_task_body->updateTaskModel(N_prec);

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update model
		robot->updateModel();
	
		if(state == JOINT_CONTROLLER)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);

			command_torques = joint_task_torques;

			if( (robot->_q - q_init_desired).norm() < 0.15 )
			{
				posori_task_body->reInitializeTask(); //
				posori_task_body->_desired_position += Vector3d(-0.1,0.1,0.1); //
				posori_task_body->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task_body->_desired_orientation; //

				joint_task->reInitializeTask();
				joint_task->_kp = 0;

				state = POSORI_CONTROLLER;
			}
		}

		else if(state == POSORI_CONTROLLER)
		// if(state == POSORI_CONTROLLER)
		{
			// update task model and set hierarchy
			MatrixXd J_tasks(6, dof);
			J_tasks = posori_task_body->_jacobian;
			MatrixXd N(dof,dof);
			robot->nullspaceMatrix(N, J_tasks);

			// left arm trajectory
			//posori_task_left->_desired_position = Vector3d(0.02, -0.03, -0.5);
			//posori_task_left->_desired_position = 0.001*Vector3d(sin(M_PI*time), cos(M_PI*time), 0);
			//posori_task_left->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task_left->_desired_orientation;


			// right arm trajectory
			//posori_task_right->_desired_position = Vector3d(0.02, -0.03, -0.5);
			//posori_task_right->_desired_position = -0.001*Vector3d(sin(M_PI*time), cos(M_PI*time), 0);
			//posori_task_right->_desired_orientation = AngleAxisd(M_PI/6, Vector3d::UnitX()).toRotationMatrix() * posori_task_right->_desired_orientation;
			
			//Desired ocean one joint orientations for navigation
			// initial_q(6)=-22.530476674*M_PI/180.0;
			// initial_q(7)=-38.938842011*M_PI/180.0;
			// initial_q(8)=-9.5874173775*M_PI/180.0;
			// initial_q(9)=86.55788003*M_PI/180.0;
			// initial_q(10)=35.068683992*M_PI/180.0;
			// initial_q(11)=-32.703571509*M_PI/180.0;
			// initial_q(12)=-22.68013325*M_PI/180.0;
			// initial_q(13)=-22.451580385*M_PI/180.0;
			// initial_q(14)=38.858341441*M_PI/180.0;
			// initial_q(15)=9.6591516935*M_PI/180.0;
			// initial_q(16)=86.49771946*M_PI/180.0;
			// initial_q(17)=-35.222695047*M_PI/180.0;
			// initial_q(18)=-32.634014433*M_PI/180.0;
			// initial_q(19)=22.559812113*M_PI/180.0;
			

			joint_task->_desired_position = docking_position;
			// joint_task->_desired_position(0) += 0.001;
			posori_task_body->_desired_position = sat_pos;
			// posori_task_body->_desired_position();

						std::cout << "Docking_position: " << docking_position << "\n";

			N_prec.setIdentity();
			posori_task_body->updateTaskModel(N_prec); //Operate in Nullspace 
			N_prec = posori_task_body->_N;
			joint_task->updateTaskModel(N_prec);

			// compute torques
			posori_task_body->computeTorques(posori_task_torques_body);
			joint_task->computeTorques(joint_task_torques);

			// std::cout << "Posori torques: " << posori_task_torques_body << "\n";
			//remove body commands for joint_task_torques
			// joint_task_torques(0)=0;
			// joint_task_torques(1)=0;
			// joint_task_torques(2)=0;
			// joint_task_torques(3)=0;
			// joint_task_torques(4)=0;
			// joint_task_torques(5)=0;
			posori_task_torques_body(6)=0;
			posori_task_torques_body(7)=0;
			posori_task_torques_body(8)=0;
			posori_task_torques_body(9)=0;
			posori_task_torques_body(10)=0;
			posori_task_torques_body(11)=0;
			posori_task_torques_body(12)=0;
			posori_task_torques_body(13)=0;
			posori_task_torques_body(14)=0;
			posori_task_torques_body(15)=0;
			posori_task_torques_body(16)=0;
			posori_task_torques_body(17)=0;
			posori_task_torques_body(18)=0;
			posori_task_torques_body(19)=0;

			// command_torques = posori_task_torques_body + joint_task_torques;
			command_torques = posori_task_torques_body;
			// command_torques = joint_task_torques;
			std::cout << "command_torques: " << command_torques << "\n";
			// if(controller_counter % 100 == 0) {
			// 	//cout << J_tasks << endl;
			// 	//cout << posori_task_left->_current_position << endl << endl;
			// 	//printJ = false;
			// }
			//for (int i = 0; i < 3; i++) {
			//	myfile << posori_task_torques_left(i) << ", ";
			//}			

		}

		// send to redis
		//command_torques.setZero();
		// command_torques(2)=0.5;
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
	}

	myfile.close();

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
