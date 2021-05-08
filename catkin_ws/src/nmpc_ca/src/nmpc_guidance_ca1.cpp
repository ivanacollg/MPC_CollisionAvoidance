#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <simulation/obstacles_list.h>

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

#include "usv_model_guidance_ca1_model/usv_model_guidance_ca1_model.h"
#include "acados_solver_usv_model_guidance_ca1.h"

/**
 * Represents a 3x3 matrix
 * */
typedef Eigen::Matrix<double, 3, 3> xMatrix3;
/**
 * Represents a 3x1 vector
 * */
typedef Eigen::Matrix<double, 3, 1> xVector3;

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
#define NX 8
// Number of control inputs
#define NU 1
// Number of measurements/references on nodes 0..N-1
#define NY 9
// Number of measurements/references on node N
#define NYN 8
// Number of obstacles allowed times 2 (x,y)
#define XYOBS 16
// Number of obstalces
#define OBS 8

class NMPC
    {
    enum systemStates{
        u = 0,
        v = 1,
        ye = 2,
        chie = 3,
        psied = 4,
        xned = 5,
        yned = 6,
        psi = 7
    };

    enum controlInputs{
        psieddot = 0
    };

    struct solver_output{
        double u0[NU];
        double x1[NX];
    };

    struct solver_input{
        double x0[NX];
        double yref[NY];
        double yref_e[NYN];
        double p_obs[XYOBS];
        double r_obs[OBS];
    };

    // publishers and subscribers
    ros::Publisher desired_speed_pub;
    ros::Publisher cross_track_error_pub;
    ros::Publisher desired_heading_pub;
    ros::Publisher desired_r_pub;
    
    ros::Subscriber local_vel_sub;
    ros::Subscriber ins_pos_sub;
    ros::Subscriber waypoints_sub;
    ros::Subscriber obstacles_sub;

    unsigned int i,j,ii;

    // global variables
    double u_des, v_des, ye_des, chie_des, psied_des;
    double u_callback, v_callback;
    double nedx_callback, nedy_callback;
    std_msgs::Float64 eye;
    std_msgs::Float64 d_heading;
    std_msgs::Float64 d_speed;
    std_msgs::Float64 d_r;

    /**
    * Obstacle list vector
    * */
    std::vector<Eigen::Vector3f> obstacles_list_;
    const double boat_radius_ = 0.5;
    const unsigned int obs_num_ = 8;
    const unsigned int init_obs_pos_ = 1000;
 
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
    float past_psied;
    float past_psieddot;
    double psi_callback;

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
        desired_r_pub = n.advertise<std_msgs::Float64>("/guidance/desired_r", 1);

        // ROS Subscribers
        local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 5, &NMPC::velocityCallback, this);
        ins_pos_sub = n.subscribe("/vectornav/ins_2d/ins_pose", 5, &NMPC::positionCallback, this);
        waypoints_sub = n.subscribe("/mission/waypoints", 5, &NMPC::waypointsCallback, this);
        obstacles_sub = n.subscribe("/usv_perception/lidar_detector/obstacles",  5, &NMPC::obstaclesCallback, this);

        // Initializing control inputs
        for(unsigned int i=0; i < NU; i++) acados_out.u0[i] = 0.0;

        // Define references (to be changed to subscribers)
        u_des = 0.0;
        v_des = 0.0;
        ye_des = 0.0;
        chie_des = 0.0;
        psied_des = 0.0;
        past_psied = 0.0;//-M_PI/2;
        past_psieddot = 0;

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
        acados_in.x0[psied] = 0.0;

        //Initialize Obstacles
        initializeObstacles();

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

    void obstaclesCallback(const simulation::obstacles_list::ConstPtr& _msg)
    {
        Eigen::Vector3f obstacle_body;
        Eigen::Vector3f obstacle_ned;

        // If there are more obstacles than MPC algorithim can handle
        if (_msg->len > obs_num_)
        {
          Eigen::VectorXd obstacle_distances(_msg->len);

          // Calculate the distance to all obstacles
          for (int i = 0; i < _msg->len; i++)
          {
            double body_x = _msg->obstacles[i].x;
            double body_y = _msg->obstacles[i].y;
            double radius = _msg->obstacles[i].z + boat_radius_;
            double distance =  sqrt(body_x*body_x + body_y*body_y) - boat_radius_;
            obstacle_distances(i) = distance;
          }

          // Sort obstacles in terms of ditances (closest to farthest)
          Eigen::VectorXi index_vec;
          Eigen::VectorXd sorted_vec;
          sort_vec(obstacle_distances,sorted_vec,index_vec);
          /*std::cout<<"Original Vector: \n";
          std::cout<<obstacle_distances<<std::endl<<std::endl;
          std::cout<<"After sorting: \n";
          std::cout<<sorted_vec<<std::endl<<std::endl;
          std::cout<<"Positioning the position in the original vector corresponding to each element of the vector"<<endl;
          std::cout<<index_vec<<std::endl;*/

          // Use only 8 closest obstacles
          for (int i = 0; i < obs_num_; i++)
          {
            int index = index_vec[i];
            double body_x = _msg->obstacles[index].x;
            double body_y = _msg->obstacles[index].y;
            double radius = _msg->obstacles[index].z + boat_radius_;
            obstacle_body << body_x, body_y, 0;
            obstacle_ned = body2NED(obstacle_body);
            obstacle_ned(2) = radius;
            //std::cout<<"Obstacles body x: "<< obstacle_ned[0] <<".\n";
            obstacles_list_[i] = obstacle_ned;
          }
        }

        // If MPC can handle # of obstacles
        else
        {
          // Initialize obstalces to a far distance with 0 radius
          initializeObstacles();

          // Obstain obstacle values
          for (int i = 0; i < _msg->len; i++)
          {
            double body_x = _msg->obstacles[i].x;
            double body_y = _msg->obstacles[i].y;
            double radius = _msg->obstacles[i].z + boat_radius_;
            obstacle_body << body_x, body_y, 0;
            obstacle_ned = body2NED(obstacle_body);
            obstacle_ned(2) = radius;
            //std::cout<<"Obstacles body x: "<< obstacle_ned[0] <<".\n";
            obstacles_list_[i] = obstacle_ned;
          }
        }

    }

    Eigen::Vector3f body2NED(const Eigen::Vector3f _obstacle_body)
    {
        Eigen::Matrix3f R;
        Eigen::Vector3f obstacle_ned;
        
        R << cos(psi_callback), -sin(psi_callback), 0.0,
            sin(psi_callback), cos(psi_callback),  0.0,
            0.0,               0.0,                1.0;
        
        obstacle_ned = R*_obstacle_body;

        obstacle_ned(0) = obstacle_ned(0) + nedx_callback;
        obstacle_ned(1) = obstacle_ned(1) + nedy_callback;
        
        return obstacle_ned;
    }

    void initializeObstacles()
    {
        obstacles_list_.clear();
        Eigen::Vector3f obstacle;
        obstacle << init_obs_pos_, init_obs_pos_, 0;

        for(i=0; i<8; i++)
        {
          obstacles_list_.push_back(obstacle);
        }

    }

    /** sorts vectors from large to small
     * vec: vector to be sorted
     * sorted_vec: sorted results
     * ind: the position of each element in the sort result in the original vector
    */
    void sort_vec(const VectorXd& vec, VectorXd& sorted_vec,  VectorXi& index){
      index = VectorXi::LinSpaced(vec.size(),0,vec.size()-1);//[0 1 2 3 ... N-1]

      auto rule=[vec](int i, int j)->bool
      {
        return vec(i)<vec(j);
      };// regular expression, as a predicate of sort

      std::sort(index.data(),index.data()+index.size(),rule);
      //The data member function returns a pointer to the first element of 
      //VectorXd, similar to begin()
      sorted_vec.resize(vec.size());

      for(int i=0;i<vec.size();i++){
        sorted_vec(i)=vec(index(i));
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
            acados_in.x0[psied] = past_psied;
            acados_in.x0[xned] = nedx_callback;
            acados_in.x0[yned] = nedy_callback;
            acados_in.x0[psi] = psi_callback;
            
            
            // acados NMPC
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", acados_in.x0);
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", acados_in.x0);

            ye_des = 0.00;

            acados_in.yref[u] = u_des;         // u
            acados_in.yref[v] = v_des;         // v
            acados_in.yref[ye] = ye_des;         // ye
            acados_in.yref[chie] = chie_des;         // chie
            acados_in.yref[psied] = psied_des;         // psied
            acados_in.yref[xned] = 0.0;
            acados_in.yref[yned] = 0.0;
            acados_in.yref[psi] = 0.0;
            acados_in.yref[psieddot] = 0.0;

            acados_in.yref_e[u] = u_des;         // u
            acados_in.yref_e[v] = v_des;         // v
            acados_in.yref_e[ye] = ye_des;         // ye
            acados_in.yref_e[chie] = chie_des;         // chie
            acados_in.yref_e[psied] = psied_des;         // psied
            acados_in.yref_e[xned] = 0.0;
            acados_in.yref_e[yned] = 0.0;
            acados_in.yref_e[psi] = 0.0;

            acados_in.p_obs[0]  = obstacles_list_[0][0];
            acados_in.p_obs[1]  = obstacles_list_[0][1];
            acados_in.p_obs[2]  = obstacles_list_[1][0];
            acados_in.p_obs[3]  = obstacles_list_[1][1];
            acados_in.p_obs[4]  = obstacles_list_[2][0];
            acados_in.p_obs[5]  = obstacles_list_[2][1];
            acados_in.p_obs[6]  = obstacles_list_[3][0];
            acados_in.p_obs[7]  = obstacles_list_[3][1];
            acados_in.p_obs[8]  = obstacles_list_[4][0];
            acados_in.p_obs[9]  = obstacles_list_[4][1];
            acados_in.p_obs[10] = obstacles_list_[5][0];
            acados_in.p_obs[11] = obstacles_list_[5][1];
            acados_in.p_obs[12] = obstacles_list_[6][0];
            acados_in.p_obs[13] = obstacles_list_[6][1];
            acados_in.p_obs[14] = obstacles_list_[7][0];
            acados_in.p_obs[15] = obstacles_list_[7][1];

            acados_in.r_obs[0] = obstacles_list_[0][2];
            acados_in.r_obs[1] = obstacles_list_[1][2];
            acados_in.r_obs[2] = obstacles_list_[2][2];
            acados_in.r_obs[3] = obstacles_list_[3][2];
            acados_in.r_obs[4] = obstacles_list_[4][2];
            acados_in.r_obs[5] = obstacles_list_[5][2];
            acados_in.r_obs[6] = obstacles_list_[6][2];
            acados_in.r_obs[7] = obstacles_list_[7][2];

            for (ii = 0; ii < N; ii++)
                {
                ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "yref", acados_in.yref);
                acados_update_params( ii, acados_in.p_obs, 16);
                ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, ii, "lh", acados_in.r_obs);
                }
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", acados_in.yref_e);
            acados_update_params( N, acados_in.p_obs, 16);

            // call solver
            acados_status = acados_solve();
            if (acados_status != 0){
                ROS_INFO_STREAM("acados returned status " << acados_status << endl);
            }

            // get solution (as u)
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", (void *)acados_out.u0);

            // get solution at stage N = 1 (as thrust comes from x1 instead of u0 because of the derivative)
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", (void *)acados_out.x1);
            float psid = acados_out.x1[psied] + _ak;
            std::cout<<"psieddot: "<< acados_out.u0[psieddot]<<".\n";
            std::cout<<"psied: "<< acados_out.x1[psied]<<".\n";
            if (fabs(psid) > M_PI){
              psid = (psid/fabs(psid))*(fabs(psid) - 2*M_PI);
            }
            past_psied = acados_out.x1[psied];
            d_heading.data = psid;
            desired_heading_pub.publish(d_heading);
            //d_r.data = acados_out.u0[psieddot];
            //desired_r_pub.publish(d_r);
            desired_speed_pub.publish(d_speed);
            eye.data = _ye;
            cross_track_error_pub.publish(eye);
        }


    };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmpc_guidance_ca");

    ros::NodeHandle n("~");
    NMPC nmpc(n);
    ros::Rate loop_rate(20);
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
            double x2 = nmpc.last_waypoints[2];
            double y2 = nmpc.last_waypoints[3];
            double ak = atan2(y2-y1, x2-x1);
            nmpc.past_psied = nmpc.psi_callback - ak;
            if (fabs(nmpc.past_psied) > M_PI){
              nmpc.past_psied = (nmpc.past_psied/fabs(nmpc.past_psied))*(fabs(nmpc.past_psied) - 2*M_PI);
            }
        }
        
        nmpc.waypoint_manager();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
