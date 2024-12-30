#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <Eigen/Dense>
#include <algorithm>  
#include <iostream>

namespace controller_one_joint
{
    constexpr int NUM_J = 7;

    class Control
    {
    private:
        const double time_tick_;
        // const Eigen::VectorXd Kp_;
        // const Eigen::VectorXd Kd_;
        // const Eigen::VectorXd Kv_;
        // const Eigen::VectorXd static_friction_;
        // const Eigen::VectorXd dynamic_friction_;

        // Eigen::VectorXd q_previous_;

        // Eigen::VectorXd torque_;
        // Eigen::VectorXd q_;
        // Eigen::VectorXd v_;

        // Eigen::VectorXd q_d_;

        // Временное

        const double Kp_;
        const double Kd_;

        const double v_max_;
        const double torque_max_;

        double q_previous_;

        double torque_;
        double q_;
        double v_;

        double q_d_;

        double gamma_;
        double k_;
        double lambda_;
        double s_;

        double I_h_;
        double b_h_;
        double T_h_;
        double mr_h_;

        unsigned long count = 0;

    public: 

        Control(const double Kp = 1, const double Kd = 0, const double v_max = 2, const double time_tick = 0.005);
        // std::array<double, NUM_J> clacTorque(std::array<double, NUM_J> q, std::array<double, NUM_J> q_d);

        void setPreviousPos(double q);

        double calcTorque(double q, double q_d);

        int sat(double s);
    };

    int sign(double a);
}


#endif