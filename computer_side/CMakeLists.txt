cmake_minimum_required(VERSION 3.16)
set(PROJECT FRI_control)

project(${PROJECT} LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)

set(COMPILE_JOINT_OVERLAY OFF)
set(J_DIR examples/LBRJointSineOverlay)

if(COMPILE_JOINT_OVERLAY)
	set(JOINT_OVERLAY_SOURCES
		${J_DIR}/LBRJointSineOverlayApp.cpp
		${J_DIR}/LBRJointSineOverlayClient.cpp		${J_DIR}/LBRJointSineOverlayClient.h
	)

	set(JOINT_OVERLAY_NAME
		example_joint
	)
endif()

set(COMPILE_TORQUE_OVERLAY ON)
set(T_DIR examples/LBRTorqueSineOverlay)

set(CONTROL ${CMAKE_CURRENT_SOURCE_DIR}/control)
set(IK ${CMAKE_CURRENT_SOURCE_DIR}/ik)
set(SERVER ${CMAKE_CURRENT_SOURCE_DIR}/udp)
set(LOCKFREE ${CMAKE_CURRENT_SOURCE_DIR}/lockfree)
set(KUKAFRI ${CMAKE_CURRENT_SOURCE_DIR}/kukafri)
set(LOGGER ${CMAKE_CURRENT_SOURCE_DIR}/logger)

if(${COMPILE_TORQUE_OVERLAY})
	set(TORQUE_OVERLAY_SOURCES
		${T_DIR}/LBRTorqueSineOverlayApp.cpp
		${T_DIR}/LBRTorqueSineOverlayClient.cpp		${T_DIR}/LBRTorqueSineOverlayClient.h
	)

	set(TORQUE_OVERLAY_NAME
		example_torque
	)
endif()

set(SOURCES
	main.cpp
	main_controller.cpp						main_controller.hpp 
	${KUKAFRI}/customlbrclient.hpp			${KUKAFRI}/customlbrclient.cpp
	${KUKAFRI}/kukafricontroller.hpp		${KUKAFRI}/kukafricontroller.cpp
	${KUKAFRI}/apiserver.hpp				${KUKAFRI}/apiserver.cpp
	${KUKAFRI}/helper_functions.hpp

	${LOGGER}/logger.hpp				${LOGGER}/logger.cpp
	# Сюда сорцы
	${SERVER}/udp_server.hpp			${SERVER}/udp_server.cpp
	${SERVER}/json.hpp
	${LOCKFREE}/lockfree.hpp
	${CONTROL}/control_interface.hpp
	${CONTROL}/control.hpp				${CONTROL}/control.cpp
	${CONTROL}/task_space_control.hpp	${CONTROL}/task_space_control.cpp
	${IK}/drake_kinematic.hpp			${IK}/drake_kinematic.cpp			${IK}/cost.hpp
)


find_library(FRI_LIB libFRIClient NAMES libFRIClient FRIClient HINTS deps/lib REQUIRED)

# Control
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(drake CONFIG REQUIRED)

message("-- FRI library found in: ${FRI_LIB}")

find_package(Threads REQUIRED)

add_executable(${PROJECT}
    ${SOURCES}
)

target_link_libraries(${PROJECT}
    ${FRI_LIB}
	Threads::Threads
    Eigen3::Eigen
	drake::drake
)

target_include_directories(${PROJECT} PUBLIC
	deps/include/
)

if(COMPILE_JOINT_OVERLAY)
	add_executable(${JOINT_OVERLAY_NAME}
		${JOINT_OVERLAY_SOURCES}
	)

	target_link_libraries(${JOINT_OVERLAY_NAME}
		${FRI_LIB}
	)

	target_include_directories(${JOINT_OVERLAY_NAME} PUBLIC
		deps/include/FRI/
	)
endif()

if(COMPILE_TORQUE_OVERLAY)
	add_executable(${TORQUE_OVERLAY_NAME}
		${TORQUE_OVERLAY_SOURCES}
	)

	target_link_libraries(${TORQUE_OVERLAY_NAME}
		${FRI_LIB}
	)

	target_include_directories(${TORQUE_OVERLAY_NAME} PUBLIC
		deps/include/FRI/
	)
endif()
