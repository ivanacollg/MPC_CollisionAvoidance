#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <iostream>
#include <sstream>
#include <fstream>
#include <ios>

#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

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

    struct solver_output{
        double u0[NU];
        double x1[NX];
    };

    struct solver_input{
        double x0[NX];
        double yref[NY];
        double yref_e[NYN];
    };

    // publishers and subscribers
    ros::Publisher right_thruster_pub;
    ros::Publisher left_thruster_pub;
    ros::Subscriber local_vel_sub;

    unsigned int k,i,j,ii;

    // global variables
    double u_des, v_des, r_des;
    double u_callback, v_callback, r_callback, past_Tport, past_Tstbd;
    std_msgs::Float64 right_thruster;
    std_msgs::Float64 left_thruster;

    // acados struct
    solver_input acados_in;
    solver_output acados_out;
    int acados_status;

public:

    NMPC(ros::NodeHandle& n)
        {

        int status = 0;

        status = acados_create();

        if (status != 0){
            ROS_INFO_STREAM("acados_create() returned status " << status << ". Exiting." << endl);
            exit(1);
        }

        // ROS Publishers 
        right_thruster_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/right_thruster", 1);
        left_thruster_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/left_thruster", 1);

        // ROS Subscribers
        local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 5, &NMPC::velocityCallback, this);

        // Initializing control inputs
        for(unsigned int i=0; i < NU; i++) acados_out.u0[i] = 0.0;

        // Define references (to be changed to subscribers)
        u_des = 1.0;
        v_des = 0;
        r_des = 0;

        // Initialize state variables
        past_Tport = 0.0;
        past_Tstbd = 0.0;
        u_callback = 0.001;

        // Initialize the state (x0)
        acados_in.x0[u] = u_callback;
        acados_in.x0[v] = 0.0;
        acados_in.x0[r] = 0.0;
        acados_in.x0[Tport] = past_Tport;
        acados_in.x0[Tstbd] = past_Tstbd;
        }
    void nmpcReset()
        {
        acados_free();
        }

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
            
            acados_in.x0[u] = u_callback;
            acados_in.x0[v] = v_callback;
            acados_in.x0[r] = r_callback;
            acados_in.x0[Tport] = past_Tport;
            acados_in.x0[Tstbd] = past_Tstbd;
            
            // acados NMPC
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", acados_in.x0);
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", acados_in.x0);

            acados_in.yref[0] = u_des;    // u
            acados_in.yref[1] = v_des;    // v
            acados_in.yref[2] = r_des;    // r
            acados_in.yref[3] = 0.00;     // Tport
            acados_in.yref[4] = 0.00;     // Tstbd
            acados_in.yref[5] = 0.00;     // UTportdot
            acados_in.yref[6] = 0.00;     // UTstbddot

            acados_in.yref_e[0] = u_des;    // u
            acados_in.yref_e[1] = v_des;    // v
            acados_in.yref_e[2] = r_des;    // r
            acados_in.yref_e[3] = 0.00;     // Tport
            acados_in.yref_e[4] = 0.00;     // Tstbd

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

            // get solution (as u)
            //ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", (void *)acados_out.u0);

            // get solution at stage N = 1 (as thrust comes from x1 instead of u0 because of the derivative)
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", (void *)acados_out.x1);

            left_thruster.data =  acados_out.x1[Tport];
            right_thruster.data =  acados_out.x1[Tstbd];

            right_thruster_pub.publish(right_thruster);
            left_thruster_pub.publish(left_thruster);

            past_Tport = acados_out.x1[Tport];
            past_Tstbd = acados_out.x1[Tstbd];

        }
    };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "acados_mpc");

    ros::NodeHandle n("~");
    NMPC nmpc(n);
    ros::Rate loop_rate(20);

    while(ros::ok()){
        nmpc.control();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
