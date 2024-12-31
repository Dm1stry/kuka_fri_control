#ifndef CONTROL_HPP
#define CONTROL_HPP

#define NUM_JOINTS 7

#include <algorithm>  
#include <iostream>
#include <array>

#include <Eigen/Dense>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

// #include "../kukafricontroller.hpp"

// using namespace KUKA_CONTROL;

namespace controller
{
    using std_jarray = std::array<double, NUM_JOINTS>;
    using eigen_jarray = Eigen::Matrix<double, NUM_JOINTS, 1>;              // Вектор-столбец

    // using eigen_jvector = Eigen::Matrix<double, NUM_JOINTS, 1>;             // Вектор-столбец
    using eigen_jmatrix = Eigen::Matrix<double, NUM_JOINTS, NUM_JOINTS>;    // Матрица

    class Control
    {
    private:
        const double torque_lim_[NUM_JOINTS] = {100, 100, 100, 50, 10, 10, 10};

        const double time_tick_;

        eigen_jarray v_max_;
        eigen_jarray previous_q_;

        eigen_jarray q_;
        eigen_jarray q_d_;

        eigen_jarray dq_;
        eigen_jarray dq_d_;
        eigen_jarray ddq_d_;

        eigen_jarray u_;
        eigen_jarray torque_;

        // Модель Pinocchio

        pinocchio::Model model_;
        pinocchio::Data data_;

        // M C g
        eigen_jmatrix M_;
        eigen_jmatrix C_;
        eigen_jarray g_;

        // PD control
        eigen_jarray Kp_;
        eigen_jarray Kd_;

    public: 

        Control(std_jarray v_max, const double time_tick = 0.005);

        bool setPreviousPos(const std_jarray& q);
        void calcVel();
        void clipTorque();

        // void calcAllTerms(const eigen_jarray& q, const eigen_jarray& dq);
        void calcAllTerms();

        bool setPDParameters(std_jarray Kp, std_jarray Kd);
        std_jarray calcTorquePD(const std_jarray& q, const std_jarray& q_d, const std_jarray& dq_d = {0,0,0,0,0,0,0}, const std_jarray& ddq_d = {0,0,0,0,0,0,0});
    };
}


#endif