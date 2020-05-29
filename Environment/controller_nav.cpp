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

int main() {

	JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
	JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
	JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::object::obj_pos";

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
	sat_pos << 0,0,0;
	// redis_client.setEigenMatrixJSON(OBJ_POS_KEY, sat_pos);

	// pose task

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

			//joint_task->_desired_position = initial_q;
			joint_task->_desired_position(0) += 0.001;
			posori_task_body->_desired_position = sat_pos;
			// posori_task_body->_desired_position();

			N_prec.setIdentity();
			posori_task_body->updateTaskModel(N_prec); //Operate in Nullspace 
			N_prec = posori_task_body->_N;
			joint_task->updateTaskModel(N);

			// compute torques
			posori_task_body->computeTorques(posori_task_torques_body);
			joint_task->computeTorques(joint_task_torques);

			std::cout << "Posori torques: " << posori_task_torques_body << "\n";
			// command_torques = joint_task_torques;
			command_torques = posori_task_torques_body;
			
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
