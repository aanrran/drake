// unitree_g1_whole_body_controller.h
#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "examples/unitree_g1/includes/pd_controller.h"

namespace drake {
namespace examples {
namespace unitree_g1 {

class UnitreeG1WholeBodyController : public systems::Diagram<double> {
 public:
  UnitreeG1WholeBodyController(
      const multibody::MultibodyPlant<double>& plant,
      const Eigen::VectorXd& kp,
      const Eigen::VectorXd& kd);

 private:
  const multibody::MultibodyPlant<double>& plant_;
  PD_Controller<double>* pd_controller_system_;
};

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
