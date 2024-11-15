cmake_minimum_required(VERSION 3.16)
set(PROJECT FRI_control)

project(${PROJECT} LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)

set(COMPILE_JOINT_OVERLAY OFF)

if(COMPILE_JOINT_OVERLAY)
	set(JOINT_OVERLAY_SOURCES
		LBRJointSineOverlayApp.cpp
		LBRJointSineOverlayClient.cpp		LBRJointSineOverlayClient.h
	)

	set(JOINT_OVERLAY_NAME
		example
	)
endif()

set(SOURCES
	main.cpp
	customlbrclient.hpp					customlbrclient.cpp
	kukafricontroller.hpp				kukafricontroller.cpp
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

add_executable(${PROJECT}
    ${SOURCES}
)

target_link_libraries(${PROJECT}
    ${FRI_LIB}
	${Boost_LIBRARIES}
	Threads::Threads
)

target_include_directories(${PROJECT} PUBLIC
	deps/include/
	${Boost_INCLUDE_DIRS}
)

if(COMPILE_JOINT_OVERLAY)
	add_executable(${JOINT_OVERLAY_NAME}
		${JOINT_OVERLAY_SOURCES}
	)

	target_link_libraries(${PROJECT}
		${FRI_LIB}
	)

	target_include_directories(${PROJECT} PUBLIC
		deps/include/FRI/
	)
endif()