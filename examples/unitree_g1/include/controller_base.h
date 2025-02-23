// controller_base.h
#ifndef CONTROLLER_BASE_H_
#define CONTROLLER_BASE_H_

#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {
namespace unitree_g1 {

class ControllerBase : public systems::LeafSystem<double> {
 public:
  explicit ControllerBase(const multibody::MultibodyPlant<double>* plant);
  virtual void ComputeControl(const systems::Context<double>& context, Eigen::VectorXd* torques) const = 0;

 protected:
  const multibody::MultibodyPlant<double>* plant_;
};

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake

#endif  // CONTROLLER_BASE_H_
