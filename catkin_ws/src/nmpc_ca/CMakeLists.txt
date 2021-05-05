cmake_minimum_required(VERSION 3.0.2)
project(nmpc_ca)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
		simulation
)

find_package(
	Eigen3 REQUIRED
)

set(acados_include "${PROJECT_SOURCE_DIR}/acados/include")
set(acados_lib "${PROJECT_SOURCE_DIR}/acados/lib")

set(full_model_build ${PROJECT_SOURCE_DIR}/scripts/usv_acados/c_generated_code)
set(full_model_low_level ${PROJECT_SOURCE_DIR}/scripts/usv_low_level/c_generated_code)
set(full_model_pf ${PROJECT_SOURCE_DIR}/scripts/usv_pf/c_generated_code)
set(full_model_guidance ${PROJECT_SOURCE_DIR}/scripts/usv_guidance/c_generated_code)
set(full_model_guidance2 ${PROJECT_SOURCE_DIR}/scripts/usv_guidance2/c_generated_code)
set(full_model_guidance3 ${PROJECT_SOURCE_DIR}/scripts/usv_guidance3/c_generated_code)
set(full_model_guidance4 ${PROJECT_SOURCE_DIR}/scripts/usv_guidance4/c_generated_code)
set(full_model_guidance5 ${PROJECT_SOURCE_DIR}/scripts/usv_guidance5/c_generated_code)
set(full_model_guidance_ca ${PROJECT_SOURCE_DIR}/scripts/usv_guidance_ca/c_generated_code)
set(full_model_guidance_ca1 ${PROJECT_SOURCE_DIR}/scripts/usv_guidance_ca1/c_generated_code)

#generate_dynamic_reconfigure_options(
#	config/crazyflie_params.cfg
#	config/crazyflie_estimator.cfg
#)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  DEPENDS eigen
#  INCLUDE_DIRS include
#  LIBRARIES sensors
  CATKIN_DEPENDS rospy simulation
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
	${full_model_build}
	${full_model_low_level}
	${full_model_pf}
	${full_model_guidance}
	${full_model_guidance2}
	${full_model_guidance3}
	${full_model_guidance4}
	${full_model_guidance5}
  ${full_model_guidance_ca}
	${full_model_guidance_ca1}
	${acados_include}
	${acados_include}/blasfeo/include/
	${acados_include}/hpipm/include/
	${acados_include}/acados/
	${acados_include}/qpOASES_e/
)

link_directories(${full_model_build})
link_directories(${full_model_low_level})
link_directories(${full_model_pf})
link_directories(${full_model_guidance})
link_directories(${full_model_guidance2})
link_directories(${full_model_guidance3})
link_directories(${full_model_guidance4})
link_directories(${full_model_guidance5})
link_directories(${full_model_guidance_ca})
link_directories(${full_model_guidance_ca1})

# NMPC node
add_executable(acados_mpc src/acados_mpc.cpp)
target_link_libraries(acados_mpc
	${full_model_build}/libacados_ocp_solver_usv_model.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)

# NMPC low level node
add_executable(nmpc_low_level src/nmpc_low_level.cpp)
target_link_libraries(nmpc_low_level
	${full_model_low_level}/libacados_ocp_solver_usv_model_low_level.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)

# NMPC path following node
add_executable(nmpc_pf src/nmpc_pf.cpp)
target_link_libraries(nmpc_pf
	${full_model_pf}/libacados_ocp_solver_usv_model_pf.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)

# NMPC guidance node
add_executable(nmpc_guidance src/nmpc_guidance.cpp)
target_link_libraries(nmpc_guidance
	${full_model_guidance}/libacados_ocp_solver_usv_model_guidance.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)

add_executable(nmpc_guidance2 src/nmpc_guidance2.cpp)
target_link_libraries(nmpc_guidance2
	${full_model_guidance2}/libacados_ocp_solver_usv_model_guidance2.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)

add_executable(nmpc_guidance3 src/nmpc_guidance3.cpp)
target_link_libraries(nmpc_guidance3
	${full_model_guidance3}/libacados_ocp_solver_usv_model_guidance3.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)

add_executable(nmpc_guidance4 src/nmpc_guidance4.cpp)
target_link_libraries(nmpc_guidance4
	${full_model_guidance4}/libacados_ocp_solver_usv_model_guidance4.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)

add_executable(nmpc_guidance5 src/nmpc_guidance5.cpp)
target_link_libraries(nmpc_guidance5
	${full_model_guidance5}/libacados_ocp_solver_usv_model_guidance5.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)

add_executable(nmpc_guidance_ca src/nmpc_guidance_ca.cpp)
target_link_libraries(nmpc_guidance_ca
	${full_model_guidance_ca}/libacados_ocp_solver_usv_model_guidance_ca.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)

add_executable(nmpc_guidance_ca1 src/nmpc_guidance_ca1.cpp)
target_link_libraries(nmpc_guidance_ca1
	${full_model_guidance_ca1}/libacados_ocp_solver_usv_model_guidance_ca1.so
	#${acados_lib}/libacados.a
	${acados_lib}/libacados.so
	${catkin_LIBRARIES}
)
add_dependencies(nmpc_guidance_ca1 ${catkin_EXPORTED_TARGETS})


#add_dependencies(acados_mpc ${PROJECT_NAME}_gencfg)
#add_dependencies(acados_mpc ${PROJECT_NAME}_gencpp)
