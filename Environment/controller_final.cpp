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

#define NAVIGATOR     0
#define STRETCH       1
#define HUG           2
#define GRASP	      3

int state = NAVIGATOR;

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
std::string OBJ_POS_KEY;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;
bool printJ = true;

int main() {

	JOINT_ANGLES_KEY = "sai2::cs225a::project::sensors::q";
	JOINT_VELOCITIES_KEY = "sai2::cs225a::project::sensors::dq";
	JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::project::actuators::fgc";
    OBJ_POS_KEY = "sai2::cs225a::camera::obj_pos";

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
    VectorXd v = VectorXd::Zero(3);

	sat_pos << 0,0.5,0; //1,0.5,0
    VectorXd Zero = VectorXd::Zero(dof);

	// pose task

	VectorXd docking_position = VectorXd::Zero(dof);
	docking_position=initial_q;


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

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
        v = redis_client.getEigenMatrixJSON(OBJ_POS_KEY);
		// update model
		robot->updateModel();
	
		if(state == NAVIGATOR)
		{
			// update task model and set hierarchy
            // std::cout << "v: " << v << "\n";
			joint_task->_desired_position = docking_position;
			joint_task->_desired_position(0) = -v(1)+2.5; //x
			joint_task->_desired_position(1) = v(0)+0.2; //y
			joint_task->_desired_position(2) = v(2); //z
			joint_task->_desired_position(3) = 0.0; //yaw
			joint_task->_desired_position(4) = 0.0; //pitch
			joint_task->_desired_position(5) = 0.0; //roll weird


			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			

			// compute torques
			joint_task->computeTorques(joint_task_torques);
            // std::cout << "command_torques: " << command_torques << "\n";
			command_torques = joint_task_torques;
            if(command_torques == Zero)
            {
                state =  STRETCH;
                std::cout << "stretch"  << "\n";
            }
			
        }
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
