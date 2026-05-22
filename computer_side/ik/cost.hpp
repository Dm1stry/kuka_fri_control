#ifndef COST
#define COST

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
#include <drake/math/autodiff_gradient.h>
#include <drake/systems/framework/context.h>

#include <variant>

#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <drake/common/autodiff.h>
#include <drake/common/drake_throw.h>

class PotentialEnergyCost {
public:
    PotentialEnergyCost(const std::string& urdf_path) {
        using drake::multibody::MultibodyPlant;
        using drake::multibody::Parser;

        // 1️⃣ Создаем и парсим модель в MultibodyPlant<double>
        plant_double_ = std::make_unique<MultibodyPlant<double>>(0.0);
        Parser(plant_double_.get()).AddModels(urdf_path);
        plant_double_->Finalize();
        context_double_ = plant_double_->CreateDefaultContext();

        // 2️⃣ Создаем AutoDiffXd-версию через ToAutoDiffXd
        plant_autodiff_system_ = plant_double_->ToAutoDiffXd();
        plant_autodiff_ = dynamic_cast<MultibodyPlant<drake::AutoDiffXd>*>(plant_autodiff_system_.get());
        DRAKE_DEMAND(plant_autodiff_ != nullptr);
        context_autodiff_ = plant_autodiff_->CreateDefaultContext();
    }

    int numInputs() const {
        return plant_double_->num_positions();
    }

    int numOutputs() const {
        return 1;
    }

    template <typename T>
    void eval(const Eigen::Ref<const Eigen::Matrix<T, -1, 1>>& q,
              Eigen::Matrix<T, -1, 1>* y) const {
        if constexpr (std::is_same_v<T, double>) {
            plant_double_->SetPositions(context_double_.get(), q);
            *y = Eigen::Matrix<T, -1, 1>::Constant(1, -plant_double_->CalcPotentialEnergy(*context_double_));
        } else if constexpr (std::is_same_v<T, drake::AutoDiffXd>) {
            plant_autodiff_->SetPositions(context_autodiff_.get(), q);
            *y = Eigen::Matrix<T, -1, 1>::Constant(1, -plant_autodiff_->CalcPotentialEnergy(*context_autodiff_));
        } else {
            throw std::logic_error("Unsupported scalar type in PotentialEnergyCost::eval");
        }
    }

private:
    // Double
    std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_double_;
    std::unique_ptr<drake::systems::Context<double>> context_double_;

    // AutoDiff
    std::unique_ptr<drake::systems::System<drake::AutoDiffXd>> plant_autodiff_system_;
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>* plant_autodiff_ = nullptr;
    std::unique_ptr<drake::systems::Context<drake::AutoDiffXd>> context_autodiff_;
};

#endif