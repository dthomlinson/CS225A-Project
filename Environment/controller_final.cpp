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
bool left_stop = false;
bool right_stop = false;

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

	// sat_pos << 0,0.5,0; //1,0.5,0
    VectorXd Zero = VectorXd::Zero(6);
    VectorXd Zero14 = VectorXd::Zero(14);
    Vector3d sat_vel, sat_pos1, sat_pos2, sat_vel_frame;

   

	// pose task

	VectorXd docking_position = VectorXd::Zero(dof);
	docking_position=initial_q;

    // pose task
	const string control_link_left = "endEffector_left";
	const Vector3d control_point_left = Vector3d(0,0,0);
	auto posori_task_left = new Sai2Primitives::PosOriTask(robot, control_link_left, control_point_left);

	const string control_link_right = "endEffector_right";
	const Vector3d control_point_right = Vector3d(0,0,0);
	auto posori_task_right = new Sai2Primitives::PosOriTask(robot, control_link_right, control_point_right);



	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
	joint_task->_use_interpolation_flag = true;
#else
	joint_task->_use_velocity_saturation_flag = true;
#endif

#ifdef USING_OTG
	posori_task_left->_use_interpolation_flag = true;
	posori_task_right->_use_interpolation_flag = true;
#else
	posori_task_left->_use_velocity_saturation_flag = true;
	posori_task_right->_use_velocity_saturation_flag = true;
#endif
	
	VectorXd posori_task_torques_left = VectorXd::Zero(dof);
	VectorXd posori_task_torques_right = VectorXd::Zero(dof);
	posori_task_left->_kp_pos = 200.0;
	posori_task_left->_kv_pos = 20.0;
	posori_task_left->_kp_ori = 200.0;
	posori_task_left->_kv_ori = 20.0;
	posori_task_right->_kp_pos = 200.0;
	posori_task_right->_kv_pos = 20.0;
	posori_task_right->_kp_ori = 200.0;
	posori_task_right->_kv_ori = 20.0;

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 250.0;
	joint_task->_kv = 15.0;

	VectorXd q_init_desired = initial_q;
	// q_init_desired << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	//q_init_desired.setZero();
	//q_init_desired *= M_PI/180.0;
	joint_task->_desired_position = q_init_desired; // may need to chage

    Vector3d pos_init;
	Matrix3d ori_init;
    double timefly;

    sat_pos1 = redis_client.getEigenMatrixJSON(OBJ_POS_KEY);


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
	posori_task_left->updateTaskModel(N_prec);
	posori_task_right->updateTaskModel(N_prec);
	double hug_start;
	Vector3d hug_pos_left_init, hug_pos_right_init, grasp_pos_left_init, grasp_pos_right_init;
	Matrix3d hug_ori_init;
	double r = 0.6;
	double freq = M_PI/20;
	double eps = 0.1;
	double t_stop = 16;
	double grasp_des_pos = 0.03;

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

            N_prec.setIdentity();
            // update task model and set hierarchy
		    MatrixXd J_tasks(12, dof);
		    J_tasks.block(0, 0, 6, dof) = posori_task_left->_jacobian;
		    J_tasks.block(6, 0, 6, dof) = posori_task_right->_jacobian;
		    MatrixXd N(dof,dof);
		    robot->nullspaceMatrix(N, J_tasks);
            // pos_init = joint_task->_desired_position;
            std::cout << "pos_init:" << pos_init  << "\n";
            std::cout << "posori desired pos:" << posori_task_left->_desired_position  << "\n";


            
		if(state == NAVIGATOR)
		{
			// update task model and set hierarchy
            // std::cout << "v: " << v << "\n";
            joint_task->_desired_position = docking_position;
			joint_task->_desired_position(0) = -v(1)+2.5; //x
			joint_task->_desired_position(1) = v(0)+0.2; //y
			joint_task->_desired_position(2) = v(2); //z
			// joint_task->_desired_position(3) = 0.0; //yaw
			// joint_task->_desired_position(4) = 0.0; //pitch
			// joint_task->_desired_position(5) = 0.0; //roll weird

            joint_task->updateTaskModel(N_prec);
			// compute torques
			joint_task->computeTorques(joint_task_torques);
            // std::cout << "command_torques: " << command_torques << "\n";
			command_torques = joint_task_torques;
            if((joint_task->_desired_position - joint_task->_current_position).norm() < eps)
            {
                state =  STRETCH;
                std::cout << "stretch"  << "\n";
                // pos_init = (posori_task_left->_current_position + posori_task_right->_current_position)/2;
	            pos_init = joint_task->_desired_position;
                sat_pos2 = redis_client.getEigenMatrixJSON(OBJ_POS_KEY);
                sat_vel = (-sat_pos1 + sat_pos2)/time;
                ori_init = posori_task_left->_current_orientation;
                timefly = time;
                sat_vel_frame(0)=-sat_vel(1);
                sat_vel_frame(1)=sat_vel(0);
                sat_vel_frame(2)=sat_vel(2);
                posori_task_left->updateTaskModel(N_prec);
	            posori_task_right->updateTaskModel(N_prec);
            }
			
        }
        else if(state == STRETCH) {
            // joint_task->_desired_position = docking_position;
            //+ sat_vel*(time-timefly)
            
			posori_task_left->_desired_position = pos_init  + ori_init*Vector3d(0, r, 0);
			posori_task_right->_desired_position = pos_init - ori_init*Vector3d(0, r, 0);
			posori_task_left->_desired_orientation = ori_init;
            posori_task_right->_desired_orientation = ori_init;

            if( (posori_task_left->_desired_position - posori_task_left->_current_position).norm() < eps && (posori_task_right->_desired_position - posori_task_right->_current_position).norm() < eps) {
				state = HUG;
                std::cout << "Hug"  << "\n";
				hug_start = time;
				hug_pos_left_init = posori_task_left->_current_position;
				hug_pos_right_init = posori_task_right->_current_position;
			}
            N_prec.setIdentity();
		    posori_task_left->updateTaskModel(N_prec); //Operate in Nullspace 
		    N_prec = posori_task_left->_N;
		    joint_task->updateTaskModel(N);

		    // compute torques
		    posori_task_left->computeTorques(posori_task_torques_left);
		    posori_task_right->computeTorques(posori_task_torques_right);
		    joint_task->computeTorques(joint_task_torques);
            // posori_task_torques_left.block(0,0,1,6)=Zero;
            // posori_task_torques_right.block(0,0,1,6)=Zero;
            // joint_task_torques.block(0,6,1,14)=Zero14;
	
		    command_torques = posori_task_torques_left + posori_task_torques_right + joint_task_torques;
            // command_torques = joint_task_torques;
        }
            else if(state == HUG){
			if(time > t_stop) {
				left_stop = true;
				posori_task_left->_desired_position = posori_task_left->_current_position;
				grasp_pos_left_init = posori_task_left->_current_position;
			}
			if(time > t_stop) {
				right_stop = true;
				posori_task_right->_desired_position = posori_task_right->_current_position;
				grasp_pos_right_init = posori_task_right->_current_position;
			}
			if(!left_stop) {
				posori_task_left->_desired_position = hug_pos_left_init + hug_ori_init*(r*Vector3d(cos(freq*(time - hug_start)), -sin(freq*(time - hug_start)), 0));
			}
			if(!right_stop) {
				posori_task_right->_desired_position = hug_pos_right_init + hug_ori_init*(r*Vector3d(cos(freq*(time - hug_start)), sin(freq*(time - hug_start)), 0));
			}
			if(left_stop && right_stop) {
				state = GRASP;
                std::cout << "grasp"  << "\n";
			}
            N_prec.setIdentity();
		    posori_task_left->updateTaskModel(N_prec); //Operate in Nullspace 
		    N_prec = posori_task_left->_N;
		    joint_task->updateTaskModel(N);

		    // compute torques
		    posori_task_left->computeTorques(posori_task_torques_left);
		    posori_task_right->computeTorques(posori_task_torques_right);
		    joint_task->computeTorques(joint_task_torques);
	
		    command_torques = posori_task_torques_left + posori_task_torques_right + joint_task_torques;

		//joint_task->_desired_position = initial_q;
		//joint_task->_desired_position(0) += 0.001;
		}
		else {
			posori_task_left->_desired_position = grasp_pos_left_init + hug_ori_init*(Vector3d(0,-grasp_des_pos,0));
			posori_task_right->_desired_position = grasp_pos_right_init + hug_ori_init*(Vector3d(0,grasp_des_pos,0));
            N_prec.setIdentity();
		    posori_task_left->updateTaskModel(N_prec); //Operate in Nullspace 
            posori_task_right->updateTaskModel(N_prec);
		    N_prec = posori_task_left->_N;
		    joint_task->updateTaskModel(N);

		    // compute torques
		    posori_task_left->computeTorques(posori_task_torques_left);
		    posori_task_right->computeTorques(posori_task_torques_right);
		    joint_task->computeTorques(joint_task_torques);
	
		    command_torques = posori_task_torques_left + posori_task_torques_right + joint_task_torques;
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
