#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <iostream>
#include <sstream>
#include <fstream>
#include <ios>
#include <cmath>

#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados/ocp_nlp/ocp_nlp_constraints_bgh.h"
#include "acados/ocp_nlp/ocp_nlp_cost_ls.h"

#include "blasfeo/include/blasfeo_d_aux.h"
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"

#include "usv_model_guidance4_model/usv_model_guidance4_model.h"
#include "acados_solver_usv_model_guidance4.h"

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
#define N 100
// Number of differential state variables
#define NX 4
// Number of control inputs
#define NU 1
// Number of measurements/references on nodes 0..N-1
#define NY 5
// Number of measurements/references on node N
#define NYN 4

class NMPC
    {
    enum systemStates{
        u = 0,
        v = 1,
        ye = 2,
        chie = 3
    };

    enum controlInputs{
        psied = 0
    };

    struct solver_output{
        double u0[NU];
    };

    struct solver_input{
        double x0[NX];
        double yref[NY];
        double yref_e[NYN];
    };

    // publishers and subscribers
    ros::Publisher desired_speed_pub;
    ros::Publisher cross_track_error_pub;
    ros::Publisher desired_heading_pub;

    ros::Subscriber local_vel_sub;
    ros::Subscriber ins_pos_sub;
    ros::Subscriber waypoints_sub;

    unsigned int i,j,ii;

    // global variables
    double u_des, v_des, ye_des, chie_des;
    double u_callback, v_callback;
    double nedx_callback, nedy_callback, psi_callback;
    std_msgs::Float64 eye;
    std_msgs::Float64 d_heading;
    std_msgs::Float64 d_speed;
 
    // acados struct
    solver_input acados_in;
    solver_output acados_out;
    int acados_status;

public:

    ros::Publisher target_pub;
    geometry_msgs::Pose2D waypoint_path;
    std::vector<double> waypoints;
    std::vector<double> last_waypoints;
    int k;

    NMPC(ros::NodeHandle& n)
    {

        int status = 0;

        status = acados_create();

        if (status != 0){
            ROS_INFO_STREAM("acados_create() returned status " << status << ". Exiting." << endl);
            exit(1);
        }

        // ROS Publishers 
        cross_track_error_pub = n.advertise<std_msgs::Float64>("/usv_control/controller/cross_track_error", 1);
        target_pub = n.advertise<geometry_msgs::Pose2D>("/guidance/target", 1);
        desired_speed_pub = n.advertise<std_msgs::Float64>("/guidance/desired_speed", 1);
        desired_heading_pub = n.advertise<std_msgs::Float64>("/guidance/desired_heading", 1);

        // ROS Subscribers
        local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 5, &NMPC::velocityCallback, this);
        ins_pos_sub = n.subscribe("/vectornav/ins_2d/ins_pose", 5, &NMPC::positionCallback, this);
        waypoints_sub = n.subscribe("/mission/waypoints", 5, &NMPC::waypointsCallback, this);

        // Initializing control inputs
        for(unsigned int i=0; i < NU; i++) acados_out.u0[i] = 0.0;

        // Define references (to be changed to subscribers)
        u_des = 0.0;
        v_des = 0.0;
        ye_des = 0.0;
        chie_des = 0.0;

        // Initialize state variables
        nedx_callback = 0.0;
        nedy_callback = 0.0;
        u_callback = 0.001;
        psi_callback = 0.0;

        // Initialize the state (x0)
        acados_in.x0[u] = u_callback;
        acados_in.x0[v] = 0.0;
        acados_in.x0[ye] = 0.0;
        acados_in.x0[chie] = 0.0;

        waypoints.clear();
        last_waypoints.clear();
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
    }

    void positionCallback(const geometry_msgs::Pose2D::ConstPtr& _pos)
    {
        nedx_callback = _pos->x;
        nedy_callback = _pos->y;
        psi_callback = _pos->theta;
    }

    void waypointsCallback(const std_msgs::Float32MultiArray::ConstPtr& _msg)
    {
        waypoints.clear();
        int leng = _msg ->layout.data_offset;

        for (i=0; i<(leng-1); i++)
        {
            waypoints.push_back(_msg -> data[i]);
        }
    }

    void waypoint_manager()
    {
        if (k < (last_waypoints.size())/2)
        {
            double x1 = last_waypoints[2*k - 2];
            double y1 = last_waypoints[2*k - 1];
            double x2 = last_waypoints[2*k];
            double y2 = last_waypoints[2*k + 1];
            waypoint_path.x = x2;
            waypoint_path.y = y2;
            target_pub.publish(waypoint_path);
            double x_squared = pow(x2 - nedx_callback, 2);
            double y_squared = pow(y2 - nedy_callback, 2);
            double distance = pow(x_squared + y_squared, 0.5);
            //std::cout<<"Distance:"<<distance<<".\n";
            d_speed.data = 0.7;
            if (distance > 1)
            {
                double ak = atan2(y2-y1, x2-x1);
                double ye = -(nedx_callback-x1)*sin(ak) 
                            + (nedy_callback-y1)*cos(ak);
                control(x1, y1, ak, ye);
            }
            else
            {
                std::cout<<"Next waypoint";
                k += 1;
            }
        }
        else
        {
            d_speed.data = 0.0;
            desired_speed_pub.publish(d_speed);
        }
    }

    void control(double _x1, double _y1, double _ak, double _ye)
        {
            double beta = std::atan2(v_callback, u_callback+.001);
            if (v*v + u*u > 0){
                beta = std::atan2(v_callback, u_callback);
            }
            double chie_new = psi_callback + beta - _ak;
            if (fabs(chie_new) > M_PI){
              chie_new = (chie_new/fabs(chie_new))*(fabs(chie_new) - 2*M_PI);
            }
            acados_in.x0[u] = u_callback;
            acados_in.x0[v] = v_callback;
            acados_in.x0[ye] = _ye;
            acados_in.x0[chie] = chie_new;
            
            // acados NMPC
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", acados_in.x0);
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", acados_in.x0);

            ye_des = 0.00;

            acados_in.yref[0] = u_des;         // u
            acados_in.yref[1] = v_des;         // v
            acados_in.yref[2] = ye_des;         // ye
            acados_in.yref[3] = chie_des;         // chie
            acados_in.yref[4] = 0.00;          // psied

            acados_in.yref_e[0] = u_des;         // u
            acados_in.yref_e[1] = v_des;         // v
            acados_in.yref_e[2] = ye_des;         // ye
            acados_in.yref_e[3] = chie_des;         // chie

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
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", (void *)acados_out.u0);

            // get solution at stage N = 1 (as thrust comes from x1 instead of u0 because of the derivative)
            //ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", (void *)acados_out.x1);
            float psid = acados_out.u0[psied] + _ak;
            if (fabs(psid) > M_PI){
              psid = (psid/fabs(psid))*(fabs(psid) - 2*M_PI);
            }
            d_heading.data = psid;
            desired_heading_pub.publish(d_heading); 
            desired_speed_pub.publish(d_speed);
            eye.data = _ye;
            cross_track_error_pub.publish(eye);
        }


    };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmpc_guidance4");

    ros::NodeHandle n("~");
    NMPC nmpc(n);
    ros::Rate loop_rate(N);
    nmpc.last_waypoints.clear();

    while(ros::ok()){
        if(nmpc.last_waypoints != nmpc.waypoints)
        {
            nmpc.k = 1;
            nmpc.last_waypoints = nmpc.waypoints;
            double x1 = nmpc.last_waypoints[0];
            double y1 = nmpc.last_waypoints[1];
            nmpc.waypoint_path.x = x1;
            nmpc.waypoint_path.y = y1;
            nmpc.target_pub.publish(nmpc.waypoint_path);
        }
        
        nmpc.waypoint_manager();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
