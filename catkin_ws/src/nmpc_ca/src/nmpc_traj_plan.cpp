#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>

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

#include "usv_model_pf_ca_model/usv_model_pf_ca_model.h"
#include "acados_solver_usv_model_pf_ca.h"

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
#define N 400
// Number of differential state variables
#define NX 12
// Number of control inputs
#define NU 2
// Number of measurements/references on nodes 0..N-1
#define NY 14
// Number of measurements/references on node N
#define NYN 12
// Number of obstacles allowed times 2 (x,y)
#define XYOBS 16
// Number of obstalces
#define OBS 8

class NMPC
    {
    enum systemStates{
        psi = 0,
        psisin = 1,
        psicos = 2,
        u = 3,
        v = 4,
        r = 5,
        ye = 6,
        ak = 7,
        nedx = 8,
        nedy = 9,
        Tport = 10,
        Tstbd = 11
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
        double p_obs[XYOBS];
        double r_obs[OBS];
    };

    // publishers and subscribers
    ros::Publisher desired_traj_pub;
    ros::Publisher desired_traj_dot_pub;
    ros::Publisher marker_pub;

    ros::Subscriber local_vel_sub;
    ros::Subscriber ins_pos_sub;
    ros::Subscriber waypoints_sub;
    ros::Subscriber obstacles_sub;

    unsigned int i,j,ii;

    // global variables
    double ak_des, psisin_des, psicos_des, u_des, v_des, r_des, ye_des;
    double psi_callback, psisin_callback, psicos_callback, u_callback, v_callback, r_callback, past_Tport, past_Tstbd;
    double nedx_callback, nedy_callback;

    geometry_msgs::Pose2D trajectory;
    geometry_msgs::Pose2D trajectory_dot;
    /**
    * Safety visualization Markers
    * */
    visualization_msgs::Marker marker;

    /**
    * Obstacle list vector
    * */
    std::vector<Eigen::Vector3f> obstacles_list_;
    const double boat_radius_ = 0.5;
    const unsigned int obs_num_ = 8;
    const unsigned int init_obs_pos_ = 1000;
    const double safety_radius_ = 0.2;

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
        desired_traj_pub = n.advertise<geometry_msgs::Pose2D>("/mission/trajectory", 1000);
        desired_traj_dot_pub = n.advertise<geometry_msgs::Pose2D>("/mission/trajectory_derivative", 1000);
        target_pub = n.advertise<geometry_msgs::Pose2D>("/guidance/target", 1000);
        marker_pub = n.advertise<visualization_msgs::Marker>("/nmpc_ca/safety_vizualization", 1);

        // ROS Subscribers
        local_vel_sub = n.subscribe("/vectornav/ins_2d/local_vel", 1000, &NMPC::velocityCallback, this);
        ins_pos_sub = n.subscribe("/vectornav/ins_2d/NED_pose", 1000, &NMPC::positionCallback, this);
        waypoints_sub = n.subscribe("/mission/waypoints", 1000, &NMPC::waypointsCallback, this);
        obstacles_sub = n.subscribe("/usv_perception/lidar_detector/obstacles",  5, &NMPC::obstaclesCallback, this);

        // Initializing control inputs
        for(unsigned int i=0; i < NU; i++) acados_out.u0[i] = 0.0;

        // Define references (to be changed to subscribers)
        ak_des = 0.0;
        psisin_des = std::sin(ak_des);
        psicos_des = std::cos(ak_des);
        u_des = 0.0;
        v_des = 0.0;
        r_des = 0.0;

        // Initialize state variables
        past_Tport = 0.0;
        past_Tstbd = 0.0;
        u_callback = 0.001;
        psi_callback = 0.0;
        psisin_callback = std::sin(psi_callback);
        psicos_callback = std::cos(psi_callback);

        // Initialize the state (x0)
        acados_in.x0[psi] = psi_callback;
        acados_in.x0[psisin] = psisin_callback;
        acados_in.x0[psicos] = psicos_callback;
        acados_in.x0[u] = u_callback;
        acados_in.x0[v] = 0.0;
        acados_in.x0[r] = 0.0;
        acados_in.x0[Tport] = past_Tport;
        acados_in.x0[Tstbd] = past_Tstbd;

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
        r_callback = _vel -> z;
    }

    void positionCallback(const geometry_msgs::Pose2D::ConstPtr& _pos)
    {
        nedx_callback = _pos->x;
        nedy_callback = _pos->y;
        psi_callback = _pos->theta;

        circleDraw(nedx_callback, nedy_callback, boat_radius_, "boat_pose", 0);
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
            double distance =  sqrt(body_x*body_x + body_y*body_y) - radius;
            obstacle_distances(i) = distance;
          }

          // Sort obstacles in terms of ditances (closest to farthest)
          Eigen::VectorXi index_vec;
          Eigen::VectorXd sorted_vec;
          sortVec(obstacle_distances,sorted_vec,index_vec);
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
            circleDraw(obstacle_ned(0), 
                       obstacle_ned(1), 
                       obstacle_ned(2), 
                       "obstacle_circle", 
                       i);
            circleDraw(obstacle_ned(0), 
                       obstacle_ned(1), 
                       obstacle_ned(2) + safety_radius_, 
                       "obstacle_circle", 
                       i + obs_num_);
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

            double distance =  sqrt(body_x*body_x + body_y*body_y);
            //std::cout<<"Obstacle "<< i << " distance "<< distance << " radius "<< radius << ".\n";
            if (distance<radius)
            {
              ROS_WARN("COLLISION Obstacle %i distance %f", i, distance);
            }

            obstacle_body << body_x, body_y, 0;
            obstacle_ned = body2NED(obstacle_body);
            obstacle_ned(2) = radius;
            //std::cout<<"Obstacles body r: "<< obstacle_ned[2] <<".\n";
            obstacles_list_[i] = obstacle_ned;
            circleDraw(obstacle_ned(0), 
                       obstacle_ned(1), 
                       obstacle_ned(2), 
                       "obstacle_circle", 
                       i);
            circleDraw(obstacle_ned(0), 
                       obstacle_ned(1), 
                       obstacle_ned(2) + safety_radius_, 
                       "obstacle_circle", 
                       i + obs_num_);
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

        for(i=0; i<obs_num_; i++)
        {
          obstacles_list_.push_back(obstacle);
        }

    }

    void circleDraw(double h, double k, double r, std::string ns, int i)
    {
      marker.header.frame_id = "/world";
      marker.header.stamp = ros::Time::now();
      marker.ns = ns;
      marker.id = i;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = h;
      marker.pose.position.y = -k;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.color.b = 0.5;
      marker.color.a = 1.0;
      marker.scale.x = 0.1;
      marker.lifetime = ros::Duration();
      geometry_msgs::Point p;
      marker.points.clear();
      p.z = 0;
      int numOfPoints = 30;
      for (double i = -r; i <= r; i += 2 * r / numOfPoints)
      {
        p.y = i;
        p.x = sqrt(pow(r, 2) - pow(i, 2));
        marker.points.push_back(p);
      }
      for (double i = r; i >= -r; i -= 2 * r / numOfPoints)
      {
        p.y = i;
        p.x = -sqrt(pow(r, 2) - pow(i, 2));
        marker.points.push_back(p);
      }
      marker_pub.publish(marker);
    }

    /** sorts vectors from large to small
     * vec: vector to be sorted
     * sorted_vec: sorted results
     * ind: the position of each element in the sort result in the original vector
    */
    void sortVec(const VectorXd& vec, VectorXd& sorted_vec,  VectorXi& index){
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
            u_des = 0.7;
            if (distance > 1)
            {
                double ak = atan2(y2-y1, x2-x1);
                psisin_des = std::sin(ak);
                psicos_des = std::cos(ak);
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
            u_des = 0.0;
            trajectory.x =  nedx_callback;
            trajectory.y =  nedy_callback;
            trajectory.theta =  0.0;
        }
    }

    void control(double _x1, double _y1, double _ak, double _ye)
        {

            double beta = std::atan2(v_callback, u_callback+.001);
            if (v*v + u*u > 0){
                beta = std::atan2(v_callback, u_callback);
            }
            double chi = psi_callback + beta;
            psisin_callback = std::sin(chi);
            psicos_callback = std::cos(chi);

            acados_in.x0[psi] = psi_callback;
            acados_in.x0[psisin] = psisin_callback;
            acados_in.x0[psicos] = psicos_callback;
            acados_in.x0[u] = u_callback;
            acados_in.x0[v] = v_callback;
            acados_in.x0[r] = r_callback;
            acados_in.x0[ye] = _ye;
            acados_in.x0[ak] = _ak;
            acados_in.x0[nedx] = nedx_callback;
            acados_in.x0[nedy] = nedy_callback;
            acados_in.x0[Tport] = past_Tport;
            acados_in.x0[Tstbd] = past_Tstbd;
            
            // acados NMPC
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", acados_in.x0);
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", acados_in.x0);

            ye_des = 0.00;

            acados_in.yref[0] = 0.00;       // psi
            acados_in.yref[1] = psisin_des;    // psisin
            acados_in.yref[2] = psicos_des;    // psicos
            acados_in.yref[3] = u_des;         // u
            acados_in.yref[4] = v_des;         // v
            acados_in.yref[5] = r_des;         // r
            acados_in.yref[6] = ye_des;         // ye
            acados_in.yref[9] = 0.00;         // ak
            acados_in.yref[10] = 0.00;         // nedx
            acados_in.yref[11] = 0.00;         // nedy
            acados_in.yref[12] = 0.00;          // Tport
            acados_in.yref[13] = 0.00;          // Tstbd
            acados_in.yref[14] = 0.00;          // UTportdot
            acados_in.yref[15] = 0.00;          // UTstbddot

            acados_in.yref_e[0] = 0.00;       // psi
            acados_in.yref_e[1] = psisin_des;    // psisin
            acados_in.yref_e[2] = psicos_des;    // psicos
            acados_in.yref_e[3] = u_des;         // u
            acados_in.yref_e[4] = v_des;         // v
            acados_in.yref_e[5] = r_des;         // r
            acados_in.yref_e[6] = ye_des;         // ye
            acados_in.yref_e[9] = 0.00;         // ak
            acados_in.yref_e[10] = 0.00;         // nedx
            acados_in.yref_e[11] = 0.00;         // nedy
            acados_in.yref_e[12] = 0.00;          // Tport
            acados_in.yref_e[13] = 0.00;          // Tstbd

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

            //std::cout<<"Obstacle position: "<< obstacles_list_[0][0] <<".\n";
            //std::cout<<"Obstacle 0 min distance: "<< obstacles_list_[0][2] <<".\n";
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
            //ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", (void *)acados_out.u0);

            // get solution at stage N = 1 (as thrust comes from x1 instead of u0 because of the derivative)
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 100, "x", (void *)acados_out.x1);

            trajectory.x =  acados_out.x1[nedx];
            trajectory.y =  acados_out.x1[nedy];
            trajectory.theta = 1.0;
            trajectory_dot.x = acados_out.x1[u]*std::cos(acados_out.x1[psi]) - acados_out.x1[v]*std::sin(acados_out.x1[psi]);
            trajectory_dot.y = acados_out.x1[u]*std::sin(acados_out.x1[psi]) + acados_out.x1[v]*std::cos(acados_out.x1[psi]);


            if (u_des == 0.0){
                trajectory.x =  nedx_callback;
                trajectory.y =  nedy_callback;
                trajectory.theta =  0.0;
            }

            desired_traj_pub.publish(trajectory);
            desired_traj_dot_pub.publish(trajectory_dot);
            past_Tport = acados_out.x1[Tport];
            past_Tstbd = acados_out.x1[Tstbd];
        }


    };

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmpc_traj_plan");

    ros::NodeHandle n("~");
    NMPC nmpc(n);
    ros::Rate loop_rate(1);
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