#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <fstream>
#include <cmath>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

int main() {

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
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.1);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	robot->Jv(Jv, link_name, pos_in_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);

	// ************** CODE ADDED *************** //

	// Q1
	VectorXd q_desired = initial_q;
	q_desired(dof - 1) = 0.1;
	VectorXd b = VectorXd::Zero(dof);
	VectorXd g = VectorXd::Zero(dof);
	// Q2
	Vector3d x = Vector3d::Zero();
	Vector3d x_dot = Vector3d::Zero();
	Vector3d x_desired = Vector3d(0.3, 0.1, 0.5);
	// Q3
	Vector3d p = Vector3d::Zero();
	// Q4
	Vector3d xd = Vector3d::Zero();
	Vector3d xd_dot = Vector3d::Zero();
	double Amp = 0.1;
	double w = M_PI;
	// writing trajectories to .txt file
	ofstream file;
	file.open("../../hw2/data_files/q4-iv.txt"); // change this according to the question

	// ************** END ********************* //

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNING_KEY, "1");
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = QUESTION_4; // change to the controller of the question you want : QUESTION_1, QUESTION_2, QUESTION_3, QUESTION_4

		robot->Jv(Jv, link_name, pos_in_link);
		robot->taskInertiaMatrix(Lambda, Jv);
		robot->nullspaceMatrix(N, Jv);
		robot->dynConsistentInverseJacobian(J_bar, Jv);
		robot->gravityVector(g);
		robot->coriolisForce(b);
		robot->position(x, link_name, pos_in_link);
		robot->linearVelocity(x_dot, link_name, pos_in_link);

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			MatrixXd Kp = MatrixXd::Zero(dof, dof);
			MatrixXd Kv = MatrixXd::Zero(dof, dof);
			for (int i = 0; i < dof - 1; i++){
				Kp(i, i) = 400.0;
				Kv(i, i) = 50.0; 
			}
			Kp(dof - 1, dof - 1) = 50.0;
			Kv(dof - 1, dof - 1) = -0.31486499; // tune this

			command_torques = -Kp * (robot->_q - q_desired) - Kv * robot->_dq + b + g;
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
			double kp = 200.0;
			double kv = 24.0; //20*sqrt(2);

			MatrixXd Kv = MatrixXd::Zero(dof, dof);
			for (int i = 0; i < dof; i++){
				Kv(i, i) = 15.0; 
			}

			auto F = Lambda * (-kp * (x - x_desired) - kv * x_dot);
			// command_torques = Jv.transpose() * F + g; // for part (a)
			// command_torques = Jv.transpose() * F + g - Kv * robot->_dq; // for part (c)
			command_torques = Jv.transpose() * F + g - N.transpose() * robot->_M * (Kv * robot->_dq); // for part (d)
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
			double kp = 200.0;
			double kv = 24.0; //20*sqrt(2);

			MatrixXd Kv = MatrixXd::Zero(dof, dof);
			for (int i = 0; i < dof; i++){
				Kv(i, i) = 15.0; 
			}

			p = J_bar.transpose() * g;
			auto F = Lambda * (-kp * (x - x_desired) - kv * x_dot) + p;
			command_torques = Jv.transpose() * F - N.transpose() * robot->_M * (Kv * robot->_dq);
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
			double kp = 200.0;
			double kv = 24.0; //20*sqrt(2);

			MatrixXd Kp = MatrixXd::Zero(dof, dof);
			for (int i = 0; i < dof - 1; i++){
				Kp(i, i) = 400.0;
			}
			Kp(dof - 1, dof - 1) = 50.0;

			MatrixXd Kv = MatrixXd::Zero(dof, dof);
			for (int i = 0; i < dof; i++){
				Kv(i, i) = 15.0;
			}

			xd = Vector3d(0.3, 0.1, 0.5) + Amp * Vector3d(sin(w * time), cos(w * time), 0.0);
			xd_dot = Amp * w * Vector3d(cos(w * time), -sin(w * time), 0.0);

			p = J_bar.transpose() * g;
			// Lambda = MatrixXd::Identity(3, 3); // for part (ii)
			auto F = Lambda * (-kp * (x - xd) - kv * (x_dot - xd_dot)) + p;
			// command_torques = Jv.transpose() * F + N.transpose() * robot->_M * (-Kv * robot->_dq); // for part (i) and (ii)
			// command_torques = Jv.transpose() * F + N.transpose() * robot->_M * (-Kp * robot->_q - Kv * robot->_dq); // for part (iii)
			command_torques = Jv.transpose() * F + N.transpose() * (robot->_M * (-Kp * robot->_q - Kv * robot->_dq) + g); // for part (iv)
		}

		// writing trajectory to txt file
		file << time << "\t" << robot->_q.transpose() << "\t" << x.transpose() << "\n";

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	file.close();
	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}