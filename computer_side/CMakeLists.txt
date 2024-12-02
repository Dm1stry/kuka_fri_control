cmake_minimum_required(VERSION 3.16)
set(PROJECT FRI_control)

project(${PROJECT} LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)

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

if(${COMPILE_TORQUE_OVERLAY})
	set(TORQUE_OVERLAY_SOURCES
		${T_DIR}/LBRTorqueSineOverlayApp.cpp
		${T_DIR}/LBRTorqueSineOverlayClient.cpp		${T_DIR}/LBRTorqueSineOverlayClient.h
	)

	set(TORQUE_OVERLAY_NAME
		example_torque
	)
endif()

set(LOGGER_SOURCES
	logger/ilogger.hpp
	logger/filelogger.hpp		logger/filelogger.cpp
	logger/jarraylogger.hpp		logger/jarraylogger.cpp
)

set(SOURCES
	main.cpp
	customlbrclient.hpp					customlbrclient.cpp
	# jointpositionclient.hpp				jointpositionclient.cpp
	# torqueclient.hpp					torqueclient.cpp
	kukafricontroller.hpp				kukafricontroller.cpp
	apiserver.hpp						apiserver.cpp

	${LOGGER_SOURCES}
	# Сюда сорцы
)


find_library(FRI_LIB libFRIClient NAMES libFRIClient FRIClient HINTS deps/lib REQUIRED)
message("-- FRI library found in: ${FRI_LIB}")

find_package(Boost 1.71)
if(Boost_FOUND)
	message("-- Boost libraries found in: ${Boost_LIBRARIES}")
	message("-- Boost headers found in: ${Boost_INCLUDE_DIRS}")
endif()

find_package(Threads REQUIRED)

find_package(nlohmann_json REQUIRED)

add_executable(${PROJECT}
    ${SOURCES}
)

target_link_libraries(${PROJECT}
    ${FRI_LIB}
	${Boost_LIBRARIES}
	Threads::Threads
	nlohmann_json::nlohmann_json
)

target_include_directories(${PROJECT} PUBLIC
	deps/include/
	${Boost_INCLUDE_DIRS}
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