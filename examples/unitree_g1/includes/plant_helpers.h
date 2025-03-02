#pragma once

#include <iostream>
#include "drake/multibody/plant/multibody_plant.h"


namespace drake {
namespace examples {
namespace unitree_g1 {
namespace helper {

    void AddActuatorsToPlant(drake::multibody::MultibodyPlant<double>& plant);
    void AddGroundPlaneToPlant(drake::multibody::MultibodyPlant<double>& plant);
    Eigen::MatrixXd StateSelectionMatrix(drake::multibody::MultibodyPlant<double>& plant);
}  // namespace helper
}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
