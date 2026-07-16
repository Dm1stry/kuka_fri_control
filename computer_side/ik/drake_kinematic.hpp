#ifndef DRAKE_KINEMATIC_HPP
#define DRAKE_KINEMATIC_HPP

#include <Eigen/Dense>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/common/find_resource.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/common/drake_assert.h>
#include <drake/geometry/scene_graph.h>
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/solvers/solve.h"
#include "drake/math/rotation_matrix.h"
#include "drake/solvers/cost.h"

#include "cost.hpp"

#include <iostream>
#include <vector>
#include <string>

namespace iiwa_kinematics
{
    const int N_JOINTS = 7;

    const double LIMITS_MAX[N_JOINTS] = {168, 118, 168, 118, 168, 118, 173};
    const double LIMITS_MIN[N_JOINTS] = {-168, -118, -168, -118, -168, -118, -173};

    const double LIMITS_VELOCITY[N_JOINTS] = {10, 10, 10, 10, 10, 10, 10};
    const double LIMITS_ACCELERATION[N_JOINTS] = {100, 100, 100, 100, 100, 100, 100};

    const double INIT[N_JOINTS] = {-0.19652407, 0.15269886, 0.21432643, 1.96000475, 2.93215314/2, -0.15599753, 1.40481551};

    const double LINKS[N_JOINTS] = {0.34, 0, 0.4, 0, 0.4, 0, 0.126};

    const double LINKS_MASS[N_JOINTS] = {/*5,*/ 4, 4, 3, 2.7, 1.7, 1.8, 0.3};
    const double LINKS_INERTIA[N_JOINTS][3] = {/*{0.05, 0.06, 0.03},*/
                                                {0.1, 0.09, 0.02},
                                                {0.05, 0.018, 0.044},
                                                {0.08, 0.075, 0.01},
                                                {0.03, 0.01, 0.029},
                                                {0.02, 0.018, 0.005},
                                                {0.005, 0.0036, 0.0047},
                                                {0.001, 0.001, 0.001}};
 
    const int MAX_ITER = 100;
    const double EPS = 1e-2;

    class DrakeKinematic
    {
    private: 

        // --------------------------------------------- Для инициализации

        drake::multibody::MultibodyPlant<double> plant_;
        std::unique_ptr<drake::systems::Context<double>> context_;

        std::shared_ptr<PotentialEnergyCost> e_cost_;

        // std::shared_ptr<PotentialEnergyCost> cost_;

        // drake::multibody::InverseKinematics* ik_;

        Eigen::Matrix<double, N_JOINTS, 1> thetta_max_;
        Eigen::Matrix<double, N_JOINTS, 1> thetta_min_;

        // --------------------------------------------- Расчеты

        Eigen::Matrix<double, N_JOINTS, 1> thetta_previous_;
        Eigen::Matrix<double, N_JOINTS, 1> thetta_;

        Eigen::Vector3d position_previous_;
        Eigen::Vector3d position_;

        Eigen::Matrix3d rotation_previous_;
        Eigen::Matrix3d rotation_;

        // --------------------------------------------- Вспомогательное

        Eigen::Vector3d p_BQ_;          // Точка Q в системе координат B
        Eigen::Vector3d lower_bound_;   // Нижняя граница позиции
        Eigen::Vector3d upper_bound_;   // Верхняя граница позиции

        const drake::math::RotationMatrix<double> identity_orientation_;    // Эталонная ориентация (мировая)

        drake::math::RotationMatrix<double> target_orientation_;            // Целевая ориентация (для перезаписи)

        drake::math::RigidTransform<double> rt_;
        

        // Eigen::Matrix<double,N_JOINTS,3> joint_pose_;            // Вектор положения углов

        

    public:
        DrakeKinematic(std::string urdf_name);
        ~DrakeKinematic();

        void setQ(const Eigen::Array<double,N_JOINTS,1> &thetta);
        Eigen::Array<double,N_JOINTS,1> getQ();

        void setRotationMatrix(const Eigen::Matrix<double,3,3> &rotation);
        Eigen::Matrix<double,3,3> getRotationMatrix();

        void setPositionVector(const Eigen::Vector3d &position);
        Eigen::Vector3d getPositionVector();

        Eigen::Matrix<double,N_JOINTS,3> getJointPose();
        
        Eigen::Array<double,N_JOINTS,1> getTorque(const Eigen::Array<double,N_JOINTS,1> &thetta, 
                                                const Eigen::Array<double,N_JOINTS,1> &d_thetta,
                                                const Eigen::Array<double,N_JOINTS,1> &dd_thetta);

        Eigen::Matrix<double,6,1> getForce(const Eigen::Array<double,N_JOINTS,1> &thetta, const Eigen::Array<double,N_JOINTS,1> &torque);

        int FK();
        int IK();
        int IK_maxU();
        int IK_minQ();
        
    };
}

#endif