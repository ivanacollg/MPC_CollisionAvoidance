cmake_minimum_required(VERSION 3.0.2)
project(nmpc_ca)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
    catkin REQUIRED COMPONENTS
    std_msgs
    tf
    roscpp
    rospy 
    std_msgs 
    genmsg
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

find_package(
	Eigen3 REQUIRED
)

set(acados_include "${PROJECT_SOURCE_DIR}/acados/include")
set(acados_lib "${PROJECT_SOURCE_DIR}/acados/lib")

set(full_model_build ${PROJECT_SOURCE_DIR}/scripts/usv_acados/c_generated_code)
set(full_model_low_level ${PROJECT_SOURCE_DIR}/scripts/usv_pf/c_generated_code)

#generate_dynamic_reconfigure_options(
#	config/crazyflie_params.cfg
#	config/crazyflie_estimator.cfg
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
	${full_model_build}
	${full_model_low_level}
	${acados_include}
	${acados_include}/blasfeo/include/
	${acados_include}/hpipm/include/
	${acados_include}/acados/
	${acados_include}/qpOASES_e/
)

link_directories(${full_model_build})
link_directories(${full_model_low_level})

# NMPC node
add_executable(acados_mpc src/acados_mpc.cpp)
target_link_libraries(acados_mpc
	${full_model_build}/libacados_ocp_solver_usv_model.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)

# NMPC pf node
add_executable(nmpc_low_level src/nmpc_low_level.cpp)
target_link_libraries(nmpc_low_level
	${full_model_low_level}/libacados_ocp_solver_usv_model_low_level.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)

#add_dependencies(acados_mpc ${PROJECT_NAME}_gencfg)
#add_dependencies(acados_mpc ${PROJECT_NAME}_gencpp)
