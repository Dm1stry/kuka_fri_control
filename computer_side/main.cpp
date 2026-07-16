#include <chrono>
#include <iostream>
#include <thread>

#include "main_controller.hpp"
#include "trajectory/trajectory_generator.hpp"
#include "udp/udp_server.hpp"

using namespace KUKA_CONTROL;
using namespace server;

int main(int argc, char **argv)
{
    // auto mode = KUKA_CONTROL::TORQUE;
    auto mode = KUKA_CONTROL::JOINT_POSITION;

    bool use_task_space = true;
    bool use_udp_source = false;
 
    // --------------------------- Инициализация сервера

    UDPServer<12,25> server("127.0.0.1", 8081, "127.0.0.1", 8080);
    if (use_udp_source)
    {
        server.start();

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        std::cout << "Server started" << std::endl;
    }

    KukaController controller(mode, "../robots/iiwa.urdf", use_task_space);
    controller.start();

    Eigen::Vector3d target_pos;
    Eigen::Matrix<double,3,3> target_rot;

    Eigen::Array<double,12,1> position_msg;
    Eigen::Array<double,25,1> obs_msg = controller.getObservation();

    target_pos << obs_msg[7], obs_msg[8], obs_msg[9];
    target_rot << obs_msg[10], obs_msg[11], obs_msg[12],
                  obs_msg[13], obs_msg[14], obs_msg[15],
                  obs_msg[16], obs_msg[17], obs_msg[18];


    // trajectory::TrajectoryGenerator trajectory_generator(
    //     target_pos,
    //     target_rot,
    //     trajectory::TrajectoryType::SquareXYZ,
    //     0.05,
    //     0.15);

    trajectory::TrajectoryGenerator trajectory_generator(
        target_pos,
        target_rot,
        trajectory::TrajectoryType::CircleXYZ,
        0.05,
        0.15);
    
    // trajectory::TrajectoryGenerator trajectory_generator(
    //     target_pos,
    //     target_rot,
    //     trajectory::TrajectoryType::LineY,
    //     0.15,
    //     0.4);

    while (true)
    {
        if (use_udp_source)
        {
            if (server.getMsg(position_msg))  // Чтение пришедших по UDP данных
            {
                target_pos[0] += position_msg[0];
                target_pos[1] += position_msg[1];
                target_pos[2] += position_msg[2];

                target_rot << position_msg[3], position_msg[4], position_msg[5],
                              position_msg[6], position_msg[7], position_msg[8],
                              position_msg[9], position_msg[10], position_msg[11];
            }
        }
        else
        {
            trajectory_generator.getTarget(target_pos, target_rot);
        }

        // if (++i > 3000)
        // {
        //     target_pos[1] += 0.4;
            
        //     i = -10000000;
        // }

        controller.setTarget(target_pos, target_rot);

        obs_msg = controller.getObservation();
        // if (use_udp_source)
        // {
        //     server.setMsg(obs_msg);
        // }

        std::this_thread::sleep_for(std::chrono::microseconds(900));
    }

    controller.stop();
    if (use_udp_source)
    {
        server.stop();
    }

    return 0;
}
