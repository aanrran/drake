#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {
namespace examples {
namespace unitree_g1 {

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::multibody::MultibodyPlant;

template <typename T>
class UnitreeG1Controller : public LeafSystem<T> {
 public:
  explicit UnitreeG1Controller(const MultibodyPlant<T>& plant);

 private:
  const MultibodyPlant<T>& plant_;
  std::unique_ptr<Context<T>> plant_context_;
  Eigen::ArrayXd damping_gains_;

  void CalcTorque(const Context<T>& context, BasicVector<T>* torque) const;
};

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake


