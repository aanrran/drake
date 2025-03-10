#pragma once

#include <memory>
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "examples/unitree_g1/includes/pd_controller.h"

namespace drake {
namespace examples {
namespace unitree_g1 {

class UnitreeG1Controller : public drake::systems::Diagram<double> {
 public:
  UnitreeG1Controller(const multibody::MultibodyPlant<double>& plant,
                      const Eigen::VectorXd& kp,
                      const Eigen::VectorXd& kd,
                      double kappa, double kd_approx);

 private:
  const multibody::MultibodyPlant<double>& plant_;
  PD_Controller<double>* pd_controller_;
};

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
