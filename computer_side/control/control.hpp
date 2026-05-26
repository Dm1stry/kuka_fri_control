#ifndef TRAJECTORY
#define TRAJECTORY

#include <Eigen/Dense>
#include <list>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <string>

#include <iostream>
#include <fcntl.h>      // shm_open
#include <sys/mman.h>   // mmap, PROT_*, MAP_*
#include <unistd.h>     // ftruncate, close

#include "control_interface.hpp"
#include "../ik/drake_kinematic.hpp"

namespace control
{
    const double delta_thetta = 1*M_PI/180;
    const int points_per_delta = 5;


    class Control : public IControl
    {
        private:

            double time_tick_ = 0.005;
            double v_min_ = 0.001;
            double v_max_ = 0.003;

            const double e_min_ = 0.05*M_PI/180;
            const double e_max_ = 0.1*M_PI/180;
            const double max_d_ = 15*M_PI/180;
            const double target_pos_eps_ = 1e-4;
            const double target_rot_eps_ = 1e-3;

            Eigen::Array<double,N_JOINTS,1> eps_min_;
            Eigen::Array<double,N_JOINTS,1> eps_max_;
            Eigen::Array<double,N_JOINTS,1> max_delta_;

            std::list<Eigen::Array<double,N_JOINTS,1>> points_;
            bool done_ = true;

            Eigen::Array<double,N_JOINTS,1> virtual_thetta_;
            Eigen::Array<double,N_JOINTS,1> next_thetta_;
            Eigen::Array<double,N_JOINTS,1> current_thetta_;
            Eigen::Array<double,N_JOINTS,1> current_torque_;
            Eigen::Array<double,N_JOINTS,1> previous_current_thetta_;
            Eigen::Array<double,N_JOINTS,1> previous_target_thetta_;
            Eigen::Array<double,N_JOINTS,1> target_torque_;
            Eigen::Array<double,N_JOINTS,1> target_thetta_;

            iiwa_kinematics::DrakeKinematic solver_;
            Eigen::Vector3d target_pos_;
            Eigen::Matrix<double,3,3> target_rot_;
            Eigen::Vector3d current_pos_;
            Eigen::Matrix<double,3,3> current_rot_;
            Eigen::Matrix<double,6,1> force_;
            int ik_state_ = 0;

            Eigen::Array<double,N_JOINTS,1> stiffness_;
            Eigen::Array<double,N_JOINTS,1> damping_;
            bool torque_initialized_ = false;

        public:
            Control(const Eigen::Array<double,N_JOINTS,1> &first_thetta, double& time_tick, const std::string &urdf_name = "../robots/iiwa.urdf");

            bool push(const Eigen::Array<double,N_JOINTS,1> &thetta);
            bool pop(Eigen::Array<double,N_JOINTS,1> &thetta);

            int updateTarget(const Eigen::Vector3d &target_pos, const Eigen::Matrix<double,3,3> &target_rot) override;
            void updateCurrentState(const Eigen::Array<double,N_JOINTS,1> &current_thetta, const Eigen::Array<double,N_JOINTS,1> &current_torque) override;
            Eigen::Array<double,N_JOINTS,1> getTorque() override;
            Eigen::Array<double,N_JOINTS,1> getNextPoint() override;
            Eigen::Array<double,N_JOINTS,1> getTargetThetta() const override;
            Eigen::Vector3d getCurrentPosition() const override;
            Eigen::Matrix<double,3,3> getCurrentRotation() const override;
            Eigen::Matrix<double,6,1> getForce() const override;
            int getState() const override;
            int getIKState() const;

            Eigen::Array<double,N_JOINTS,1> getDelta(const Eigen::Array<double,N_JOINTS,1> &next_thetta, const Eigen::Array<double,N_JOINTS,1> &current_thetta);
            Eigen::Array<double,N_JOINTS,1>& getNextPoint(const Eigen::Array<double,N_JOINTS,1> &next_thetta, const Eigen::Array<double,N_JOINTS,1> &current_thetta);
            bool getDone();

            Eigen::Array<double,N_JOINTS,1>& getTorque(const Eigen::Array<double,N_JOINTS,1> &next_thetta, const Eigen::Array<double,N_JOINTS,1> &current_thetta);

            size_t size();

    };

    bool eigenArrayEqual(const Eigen::Array<double,N_JOINTS,1> &arr1, const Eigen::Array<double,N_JOINTS,1> &arr2, const Eigen::Array<double,N_JOINTS,1> &eps);
    bool eigenArrayDiff(const Eigen::Array<double,N_JOINTS,1> &arr1, const Eigen::Array<double,N_JOINTS,1> &arr2, const Eigen::Array<double,N_JOINTS,1> &diff);
    int sign(double a);

    void waitConnection();
}

#endif
