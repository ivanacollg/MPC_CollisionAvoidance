/*
* The MIT License (MIT)
*
* Copyright 2020 Barbara Barros Carlos, Tommaso Sartor
*
* This file is part of crazyflie_nmpc.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/


#include <ros/ros.h>
#include <std_srvs/Empty.h>


// msgs
//#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
/*// crazyflie
#include <crazyflie_controller/CrazyflieState.h>
#include <crazyflie_controller/PropellerSpeeds.h>
#include <crazyflie_controller/CrazyflieStateStamped.h>
#include <crazyflie_controller/PropellerSpeedsStamped.h>
#include <crazyflie_controller/CrazyflieOpenloopTraj.h>
#include <crazyflie_controller/GenericLogData.h>*/

// Dynamic reconfirgure
//#include <dynamic_reconfigure/server.h>
//#include <boost/thread.hpp>
//#include "boost/thread/mutex.hpp"
// crazyflie
//#include <crazyflie_controller/crazyflie_paramsConfig.h>

// Matrices and vectors
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// standard
#include <iostream>
#include <sstream>
#include <fstream>
#include <ios>

// acados
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

// blasfeo
#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

// usv specific
#include "usv_model_model/usv_model_model.h"
#include "acados_solver_usv_model.h"

// global data
ocp_nlp_in * nlp_in;
ocp_nlp_out * nlp_out;
ocp_nlp_solver * nlp_solver;
void * nlp_opts;
ocp_nlp_plan * nlp_solver_plan;
ocp_nlp_config * nlp_config;
ocp_nlp_dims * nlp_dims;

external_function_param_casadi * forw_vde_casadi;

using namespace Eigen;
using std::ofstream;
using std::cout;
using std::endl;
using std::fixed;
using std::showpos;

// acados dims

// Number of intervals in the horizon
#define N 20
// Number of differential state variables
#define NX 5
// Number of control inputs
#define NU 2
// Number of measurements/references on nodes 0..N-1
#define NY 7
// Number of measurements/references on node N
#define NYN 5
// Constants
//#define pi  3.14159265358979323846
//#define g0 9.80665

#define WEIGHT_MATRICES 0
#define SET_WEIGHTS 0
#define FIXED_U0 0
#define CONTROLLER 1
#define PUB_OPENLOOP_TRAJ 0

class NMPC
	{
	enum systemStates{
		u = 0,
		v = 1,
		r = 2,
		Tport = 3,
	  	Tstbd = 4
	};

	enum controlInputs{
		UTportdot = 0,
		UTstbddot = 1,
	};

	/*enum reference_mode{
		Regulation = 0,
		Tracking = 1,
		Position_Hold = 2
	};*/

	/*struct euler{
		double phi;
		double theta;
		double psi;
	};*/

	struct solver_output{
		double status, KKT_res, cpu_time;
		double u0[NU];
		//double u1[NU];
		double x1[NX];
		//double x2[NX];
		//double x4[NX];
		double xi[NU];
		double ui[NU];
	};

	struct solver_input{
		double x0[NX];
		//double yref[(NY*N)];
		double yref[NY];
		double yref_e[NYN];
		double W[NY*NY];
		double WN[NX*NX];
	};

	//ros::Publisher p_motvel;
	//ros::Publisher p_bodytwist;
	// trajectory
	//ros::Publisher p_ol_traj;

	//ros::Subscriber s_estimator;

	//ros::Subscriber s_imu_sub;
	//ros::Subscriber s_eRaptor_sub;
	//ros::Subscriber s_euler_sub;
	//ros::Subscriber s_motors;


	ros::Publisher right_thruster_pub;
	ros::Publisher left_thruster_pub;

	//ros::Subscriber ins_pose_sub;
	ros::Subscriber local_vel_sub;

	// Variables for joy callback
	/*double joy_roll,joy_pitch,joy_yaw;
	double joy_thrust;*/

	unsigned int k,i,j,ii;

	//float uss,Ct,mq;

	// Variables of the nmpc control process
	double x0_sign[NX];
	double yref_sign[(NY*N)+NY];

	// Variables for dynamic reconfigure
	double u_des, v_des, r_des;
	double u_callback, v_callback, r_callback, past_Tport, past_Tstbd;
	std_msgs::Float64 right_thruster;
	std_msgs::Float64 left_thruster;

	// Variables for dynamic reconfigure
	double Wdiag_u,Wdiag_v,Wdiag_r;
	double Wdiag_Tport,Wdiag_Tstbd;
	double Wdiag_UTportdot,Wdiag_UTstbddot;
	double WN_factor;

	// acados struct
	solver_input acados_in;
	solver_output acados_out;
	int acados_status;

	//reference_mode policy;

	// Variable for storing the optimal trajectory
	/*std::vector<std::vector<double> > precomputed_traj;
	int N_STEPS,iter;*/

public:

	NMPC(ros::NodeHandle& n)
		{

		int status = 0;
		//double WN_factor = 5;

		status = acados_create();

		if (status != 0){
			ROS_INFO_STREAM("acados_create() returned status " << status << ". Exiting." << endl);
			exit(1);
		}

		// publisher for the real robot inputs (thrust, roll, pitch, yawrate)
		//p_bodytwist = n.advertise<geometry_msgs::Twist>("/crazyflie/cmd_vel", 1);

		// publisher for the control inputs of acados (motor speeds to be applied)
		//p_motvel = n.advertise<crazyflie_controller::PropellerSpeedsStamped>("/crazyflie/acados_motvel", 1);

		// solution
		//p_ol_traj = n.advertise<crazyflie_controller::CrazyflieOpenloopTraj>("/cf_mpc/openloop_traj", 2);

		// subscriber of estimator state
		//s_estimator = n.subscribe("/cf_estimator/state_estimate", 5, &NMPC::iteration, this);

		//ROS Publishers 
		right_thruster_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/right_thruster", 1);
		left_thruster_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/left_thruster", 1);

		//ins_pose_sub = n.subscribe("/vectornav/ins_2d/ins_pose", 1000, &AdaptiveSlidingModeControl::insCallback, this);
		local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 5, &NMPC::velocityCallback, this);

		// Initializing control inputs
		for(unsigned int i=0; i < NU; i++) acados_out.u0[i] = 0.0;

		/*// Steady-state control input value
		// (Kg)
		mq = 33e-3;
		// (N/kRPM^2)
		Ct = 3.25e-4;
		// steady state prop speed (kRPM)
		uss = sqrt((mq*g0)/(4*Ct));*/

		//const char * c = ref_traj.c_str();

		/*// Pre-load the trajectory
		N_STEPS = readDataFromFile(c, precomputed_traj);
		if (N_STEPS == 0){
			ROS_WARN("Cannot load CasADi optimal trajectory!");
		}
		else{
			ROS_INFO_STREAM("Number of steps of selected trajectory: " << N_STEPS << endl);
		}*/
		// Initialize dynamic reconfigure options
		u_des = 1.0;
		v_des = 0;
		r_des = 0;

		past_Tport = 0.0;
		past_Tstbd = 0.0;
		u_callback = 0.001;

		acados_in.x0[u] = u_callback;
		acados_in.x0[v] = 0.0;
		acados_in.x0[r] = 0.0;
		acados_in.x0[Tport] = past_Tport;
		acados_in.x0[Tstbd] = past_Tstbd;


		// Set number of trajectory iterations to zero initially
		//iter = 0;

		// Set weight matrix values
		/*Wdiag_u	= 1e0 ;
		Wdiag_v = 0.0 ;
		Wdiag_r	= 1e0 ;
		Wdiag_Tport	= 0.0;
		Wdiag_Tstbd	= 0.0;
		Wdiag_UTportdot	= 0.0;
		Wdiag_UTstbddot	= 0.0;*/
		}

	/*void run()
		{
		ROS_DEBUG("Setting up the dynamic reconfigure panel and server");

			dynamic_reconfigure::Server<crazyflie_controller::crazyflie_paramsConfig> server;
			dynamic_reconfigure::Server<crazyflie_controller::crazyflie_paramsConfig>::CallbackType f;
			f = boost::bind(&NMPC::callback_dynamic_reconfigure, this, _1, _2);
			server.setCallback(f);

		ros::spin();
		}*/

	/*void callback_dynamic_reconfigure(crazyflie_controller::crazyflie_paramsConfig &config, uint32_t level)
		{
		if (level && CONTROLLER)
			{
			if(config.enable_traj_tracking)
				{
				ROS_INFO_STREAM("Tracking trajectory");
				config.enable_regulation = false;
				//policy = Tracking;
				}
			if(config.enable_regulation)
				{
				config.enable_traj_tracking = false;
				u_des = config.u_des;
				v_des = config.v_des;
				r_des = config.r_des;
				//policy = Regulation;
				}
				ROS_INFO_STREAM(
					fixed << showpos << "Quad status" << endl
					<< "NMPC for regulation: " <<  (config.enable_regulation?"ON":"off") << endl
					<< "NMPC trajectory tracker: " <<  (config.enable_traj_tracking?"ON":"off") << endl
					<< "Current regulation point: " << u_des << ", " << v_des << ", " << r_des << endl
				);
			}

		if (level && WEIGHT_MATRICES)
		 {
				ROS_INFO("Changing the weight of NMPC matrices!");
				Wdiag_u	= config.Wdiag_u;
				Wdiag_v	= config.Wdiag_v;
				Wdiag_r	= config.Wdiag_r;
				Wdiag_Tport	= config.Wdiag_Tport;
				Wdiag_Tstbd	= config.Wdiag_Tstbd;
				Wdiag_UTportdot	= config.Wdiag_UTportdot;
				Wdiag_UTstbddot	= config.Wdiag_UTstbddot;
		 }
		}*/

	/*int readDataFromFile(const char* fileName, std::vector<std::vector<double> > &data)
		{
		std::ifstream file(fileName);
		std::string line;
		int num_of_steps = 0;

		if (file.is_open())
			{
			while(getline(file, line)){
				++num_of_steps;
				std::istringstream linestream( line );
				std::vector<double> linedata;
				double number;

				while( linestream >> number ){
					linedata.push_back( number );
				}
				data.push_back( linedata );
			}

			file.close();
			}
		else
			{
			return 0;
			}

		return num_of_steps;
		}*/

	/*euler quatern2euler(Quaterniond* q)
		{

		euler angle;

		double R11 = 2*(q->w()*q->w()+q->x()*q->x())-1;
		double R21 = 2*(q->x()*q->y()-q->w()*q->z());
		double R31 = 2*(q->x()*q->z()+q->w()*q->y());
		double R32 = 2*(q->y()*q->z()-q->w()*q->x());
		double R33 = 2*(q->w()*q->w()+q->z()*q->z())-1;

		double phi	 = atan2(R32, R33);
		double theta = -asin(R31);
		double psi	 = atan2(R21, R11);

		angle.phi		 = phi;
		angle.theta  = theta;
		angle.psi	   = psi;

		return angle;
		}*/

	/*double deg2Rad(double deg)
		{
		return deg / 180.0 * pi;
		}

	double rad2Deg(double rad)
		{
		return rad * 180.0 / pi;
		}*/

	void nmpcReset()
		{
		acados_free();
		}

	/*int krpm2pwm(double Krpm)
		{
		int pwm = ((Krpm*1000)-4070.3)/0.2685;
		return pwm;
		}*/

	void velocityCallback(const geometry_msgs::Vector3::ConstPtr& _vel)
	{
		u_callback = _vel -> x;
		if (u_callback == 0){
			u_callback = 0.001;
		}
		v_callback = _vel -> y;
		r_callback = _vel -> z;
	}

	void control()
		{
			//try{
					/*switch(policy){

					  case Regulation:
					  {
					    // Update regulation point
					    for (k = 0; k < N+1; k++)
							{
							  yref_sign[k * NY + 0] = u_des; // u
							  yref_sign[k * NY + 1] = v_des;	// v
							  yref_sign[k * NY + 2] = r_des;	// r
							  yref_sign[k * NY + 3] = 1.00;		// qw
							  yref_sign[k * NY + 4] = 0.00;		// qx
							  yref_sign[k * NY + 5] = 0.00;		// qy
							  yref_sign[k * NY + 6] = 0.00;		// qz
							  yref_sign[k * NY + 7] = 0.00;		// vbx
							  yref_sign[k * NY + 8] = 0.00;		// vby
							  yref_sign[k * NY + 9] = 0.00;		// vbz
							  yref_sign[k * NY + 10] = 0.00;	// wx
							  yref_sign[k * NY + 11] = 0.00;	// wy
							  yref_sign[k * NY + 12] = 0.00;	// wz
							  yref_sign[k * NY + 13] = uss;		// w1
							  yref_sign[k * NY + 14] = uss;		// w2
							  yref_sign[k * NY + 15] = uss;		// w3
							  yref_sign[k * NY + 16] = uss;		// w4
					    }
					    break;
					  }

					  case Tracking:
					  {
					    if(iter < N_STEPS-N)
								{
									// Update reference
						 			for (k = 0; k < N+1; k++)
									{
						 		      yref_sign[k * NY + 0] = precomputed_traj[iter + k][xq];
								      yref_sign[k * NY + 1] = precomputed_traj[iter + k][yq];
								      yref_sign[k * NY + 2] = precomputed_traj[iter + k][zq];
								      yref_sign[k * NY + 3] = precomputed_traj[iter + k][qw];
								      yref_sign[k * NY + 4] = precomputed_traj[iter + k][qx];
								      yref_sign[k * NY + 5] = precomputed_traj[iter + k][qy];
								      yref_sign[k * NY + 6] = precomputed_traj[iter + k][qz];
								      yref_sign[k * NY + 7] = precomputed_traj[iter + k][vbx];
								      yref_sign[k * NY + 8] = precomputed_traj[iter + k][vby];
								      yref_sign[k * NY + 9] = precomputed_traj[iter + k][vbz];
								      yref_sign[k * NY + 10] = precomputed_traj[iter + k][wx];
								      yref_sign[k * NY + 11] = precomputed_traj[iter + k][wy];
								      yref_sign[k * NY + 12] = precomputed_traj[iter + k][wz];
								      yref_sign[k * NY + 13] = precomputed_traj[iter + k][13];
								      yref_sign[k * NY + 14] = precomputed_traj[iter + k][14];
								      yref_sign[k * NY + 15] = precomputed_traj[iter + k][15];
								      yref_sign[k * NY + 16] = precomputed_traj[iter + k][16];
					 				}
									++iter;
							//cout << iter << endl;
					    }
			 	    else policy = Position_Hold;
				    break;
					  }

					  case Position_Hold:
					  	{
						    ROS_INFO("Holding last position of the trajectory.");
						    // Get last point of tracketory and hold
						    for (k = 0; k < N+1; k++)
									{
								    yref_sign[k * NY + 0] = precomputed_traj[N_STEPS-1][xq];
								    yref_sign[k * NY + 1] = precomputed_traj[N_STEPS-1][yq];
								    yref_sign[k * NY + 2] = precomputed_traj[N_STEPS-1][zq];
								    yref_sign[k * NY + 3] = 1.00;
								    yref_sign[k * NY + 4] = 0.00;
								    yref_sign[k * NY + 5] = 0.00;
								    yref_sign[k * NY + 6] = 0.00;
								    yref_sign[k * NY + 7] = 0.00;
								    yref_sign[k * NY + 8] = 0.00;
								    yref_sign[k * NY + 9] = 0.00;
								    yref_sign[k * NY + 10] = 0.00;
								    yref_sign[k * NY + 11] = 0.00;
								    yref_sign[k * NY + 12] = 0.00;
								    yref_sign[k * NY + 13] = uss;
								    yref_sign[k * NY + 14] = uss;
								    yref_sign[k * NY + 15] = uss;
								    yref_sign[k * NY + 16] = uss;
						      }
					  }
					  break;*/
            // Update regulation point
			/*
			for (k = 0; k < N+1; k++)
				{
					yref_sign[k * NY + 0] = u_des; // u
					yref_sign[k * NY + 1] = v_des;	// v
					yref_sign[k * NY + 2] = r_des;	// r
					yref_sign[k * NY + 3] = 0.00;		// Tport
					yref_sign[k * NY + 4] = 0.00;		// Tstbd
					yref_sign[k * NY + 5] = 0.00;		// UTportdot
					yref_sign[k * NY + 6] = 0.00;		// UTstbddot
				}
			*/
					    //break;

					//}

			// --- Set Weights
			/*for (ii = 0; ii < ((NY)*(NY)); ii++) {
				acados_in.W[ii] = 0.0;
			}
			for (ii = 0; ii < ((NX)*(NX)); ii++) {
				acados_in.WN[ii] = 0.0;
			}

			acados_in.W[0+0*(NU+NX)]   = Wdiag_u;
			acados_in.W[1+1*(NU+NX)]   = Wdiag_v;
			acados_in.W[2+2*(NU+NX)]   = Wdiag_r;
			acados_in.W[3+3*(NU+NX)]   = Wdiag_Tport;
			acados_in.W[4+4*(NU+NX)]   = Wdiag_Tstbd;
			acados_in.W[5+5*(NU+NX)]   = Wdiag_UTportdot;
			acados_in.W[6+6*(NU+NX)]   = Wdiag_UTstbddot;

			acados_in.WN[0+0*(NX)]   = Wdiag_u*WN_factor;
			acados_in.WN[1+1*(NX)]   = Wdiag_v*WN_factor;
			acados_in.WN[2+2*(NX)]   = Wdiag_r*WN_factor;
			acados_in.WN[3+3*(NX)]   = Wdiag_Tport*WN_factor;
			acados_in.WN[4+4*(NX)]   = Wdiag_Tstbd*WN_factor;
			acados_in.WN[5+5*(NX)]   = Wdiag_UTportdot*WN_factor;
			acados_in.WN[6+6*(NX)]   = Wdiag_UTstbddot*WN_factor;*/


			// --- Read Estimate
			/*// position
			acados_in.x0[xq] = msg->pos.x;
			acados_in.x0[yq] = msg->pos.y;
			acados_in.x0[zq] = msg->pos.z;

			// quaternion
			acados_in.x0[qw] = msg->quat.w;
			acados_in.x0[qx] = msg->quat.x;
			acados_in.x0[qy] = msg->quat.y;
			acados_in.x0[qz] = msg->quat.z;

			// body velocities
			acados_in.x0[vbx] = msg->vel.x;
			acados_in.x0[vby] = msg->vel.y;
			acados_in.x0[vbz] = msg->vel.z;

			// rates
			acados_in.x0[wx] = msg->rates.x;
			acados_in.x0[wy] = msg->rates.y;
			acados_in.x0[wz] = msg->rates.z;*/

			
      		acados_in.x0[u] = u_callback;
			acados_in.x0[v] = v_callback;
			acados_in.x0[r] = r_callback;
      		acados_in.x0[Tport] = past_Tport;
      		acados_in.x0[Tstbd] = past_Tstbd;
			//ROS_INFO_STREAM("state speed: " << acados_in.x0[u] << endl);

			// --- acados NMPC
			ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", acados_in.x0);
			ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", acados_in.x0);

			/*for (i = 0; i < N; i++) {
	    	for (j = 0; j < NY; ++j) acados_in.yref[i*NY + j] = yref_sign[i*NY + j];
			}

			for (i = 0; i < NYN; i++) acados_in.yref_e[i] = yref_sign[N*NY + i];

			for (ii = 0; ii < N; ii++)
				{
				ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "yref", acados_in.yref + ii*NY);
				}
			ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", acados_in.yref_e);*/

			/*#if SET_WEIGHTS
			for (ii = 0; ii < N; ii++)
				{
				ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "W", acados_in.W);
				}
				ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", acados_in.WN);
			#endif

			// set constraints
			#if FIXED_U0
				ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbu", acados_out.u1);
				ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubu", acados_out.u1);
			#endif*/

			acados_in.yref[0] = u_des; // u
			acados_in.yref[1] = v_des;	// v
			acados_in.yref[2] = r_des;	// r
			acados_in.yref[3] = 0.00;	// Tport
			acados_in.yref[4] = 0.00;	// Tstbd
			acados_in.yref[5] = 0.00;	// UTportdot
			acados_in.yref[6] = 0.00;	// UTstbddot

			acados_in.yref_e[0] = u_des;	// u
			acados_in.yref_e[1] = v_des;	// v
			acados_in.yref_e[2] = r_des;	// r
			acados_in.yref_e[3] = 0.00;	// Tport
			acados_in.yref_e[4] = 0.00;	// Tstbd

			for (ii = 0; ii < N; ii++)
				{
				ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "yref", acados_in.yref);
				}
			ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", acados_in.yref_e);

			// call solver
			acados_status = acados_solve();
			if (acados_status != 0){
				ROS_INFO_STREAM("acados returned status " << acados_status << endl);
			}
			// assign output signals
			acados_out.status = acados_status;
			acados_out.KKT_res = (double)nlp_out->inf_norm_res;
			acados_out.cpu_time = (double)nlp_out->total_time;

			// get solution
			//ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", (void *)acados_out.u0);

			// get solution at stage N = 1
			ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", (void *)acados_out.x1);

			// get solution at stage N = 4 which compensates 60 ms delay
			//ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 4, "x", (void *)acados_out.x4);

			// publish acados output
			//crazyflie_controller::PropellerSpeedsStamped propellerspeeds;
			//propellerspeeds.header.stamp = ros::Time::now();

			/*if (FIXED_U0 == 1) {
				propellerspeeds.UTportdot =  acados_out.u1[UTportdot];
				propellerspeeds.UTstbddot =  acados_out.u1[UTstbddot];
			} else {
				propellerspeeds.UTportdot =  acados_out.u0[UTportdot];
				propellerspeeds.UTstbddot =  acados_out.u0[ITstbddot];
			}
			p_motvel.publish(propellerspeeds);*/

			left_thruster.data =  acados_out.x1[Tport];
			right_thruster.data =  acados_out.x1[Tstbd];

			right_thruster_pub.publish(right_thruster);
			left_thruster_pub.publish(left_thruster);

			past_Tport = acados_out.x1[Tport];
			past_Tstbd = acados_out.x1[Tstbd];

			/*acados_in.x0[u] = acados_out.x1[u];
			acados_in.x0[v] = acados_out.x1[v];
			acados_in.x0[r] = acados_out.x1[r];
			acados_in.x0[Tport] = past_Tport;
			acados_in.x0[Tstbd] = past_Tstbd;*/

			/*// Select the set of optimal states to calculate the real cf control inputs
			Quaterniond q_acados_out;
			q_acados_out.w() = acados_out.x4[qw];
			q_acados_out.x() = acados_out.x4[qx];
			q_acados_out.y() = acados_out.x4[qy];
			q_acados_out.z() = acados_out.x4[qz];
			q_acados_out.normalize();

			// Convert acados output quaternion to desired euler angles
			euler eu_imu;
			eu_imu = quatern2euler(&q_acados_out);

			// Publish real control inputs
			geometry_msgs::Twist bodytwist;

			// linear_x -> pitch
			bodytwist.linear.x  = 1.0*rad2Deg(eu_imu.theta);
			// linear_y -> roll
			bodytwist.linear.y  = -1.0*rad2Deg(eu_imu.phi);
			// linear_z -> thrust
			bodytwist.linear.z  = krpm2pwm(
				(acados_out.u1[w1]+acados_out.u1[w2]+acados_out.u1[w3]+acados_out.u1[w4])/4
			);
			// angular_z -> yaw rate
			bodytwist.angular.z = rad2Deg(acados_out.x4[wz]);

			p_bodytwist.publish(bodytwist);*/

			// --- Publish openloop
			/*#if PUB_OPENLOOP_TRAJ

			crazyflie_controller::CrazyflieOpenloopTraj traj_msg;
			traj_msg.header.stamp = ros::Time::now();
			traj_msg.cpu_time = acados_out.cpu_time;

			for(ii=0; ii< N; ii++)
				{
				ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", (void *)(acados_out.xi));
				ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", (void *)(acados_out.ui));

				crazyflie_controller::CrazyflieState crazyflie_state;
				crazyflie_controller::PropellerSpeeds crazyflie_control;

				crazyflie_state.pos.x    = acados_out.xi[xq];
				crazyflie_state.pos.y    = acados_out.xi[yq];
				crazyflie_state.pos.z    = acados_out.xi[zq];
				crazyflie_state.vel.x    = acados_out.xi[vbx];
				crazyflie_state.vel.y    = acados_out.xi[vby];
				crazyflie_state.vel.z    = acados_out.xi[vbz];
				crazyflie_state.quat.w   = acados_out.xi[qw];
				crazyflie_state.quat.x   = acados_out.xi[qx];
				crazyflie_state.quat.y   = acados_out.xi[qy];
				crazyflie_state.quat.z   = acados_out.xi[qz];
				crazyflie_state.rates.x  = acados_out.xi[wx];
				crazyflie_state.rates.y  = acados_out.xi[wy];
				crazyflie_state.rates.z  = acados_out.xi[wz];

				crazyflie_control.w1 = acados_out.ui[w1];
				crazyflie_control.w2 = acados_out.ui[w2];
				crazyflie_control.w3 = acados_out.ui[w3];
				crazyflie_control.w4 = acados_out.ui[w4];

				traj_msg.states.push_back(crazyflie_state);
				traj_msg.controls.push_back(crazyflie_control);
				}

			p_ol_traj.publish(traj_msg);
			#endif*/
			//}

		/*catch (int acados_status)
			{
			ROS_INFO_STREAM("An exception occurred. Exception Nr. " << acados_status << endl);
			}*/
		}
	};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "acados_mpc");

	ros::NodeHandle n("~");
  NMPC nmpc(n);
  ros::Rate loop_rate(20);
	//std::string ref_traj;
	//n.getParam("ref_traj", ref_traj);
	//nmpc.run();

  while(ros::ok()){
    nmpc.control();
    ros::spinOnce();
    loop_rate.sleep();
  }
	return 0;
}
