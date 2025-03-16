#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/vector_system.h"

namespace drake {
namespace examples {
namespace unitree_g1 {
// Aliases for commonly used Drake classes to improve code readability
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;
/**
 * @brief Controller for the Unitree G1 robot.
 *
 * This controller is implemented as a Drake LeafSystem. It reads the full state
 * of the robot (joint positions and velocities) and applies damping-based
 * torque control.
 *
 * @tparam T The scalar type (e.g., double) used for numerical calculations.
 */
template <typename T>
class UnitreeG1Controller : public LeafSystem<T> {
 public:
  /**
   * @brief Constructs the Unitree G1 Controller.
   *
   * @param plant Reference to the MultibodyPlant representing the robot model.
   *
   * This constructor sets up the controller as a LeafSystem with:
   * - An input port for the robot's full state (positions + velocities).
   * - An output port that provides torque commands.
   */
  explicit UnitreeG1Controller(const MultibodyPlant<T>& plant);

 private:
  /** Reference to the MultibodyPlant model of the robot (not owned by this
   * class). */
  const MultibodyPlant<T>& plant_;
  /**
   * @brief Internal context for the plant model.
   *
   * This is used to update the robot's state internally without modifying the
   * main plant context.
   */
  std::unique_ptr<Context<T>> plant_context_;

  Eigen::ArrayXd damping_gains_;
  /**
   * @brief Computes torque output for the robot based on velocity damping.
   *
   * This function extracts the state input (q, v), applies a simple damping
   * model, and outputs torques to resist joint velocities.
   *
   * @param context The system's current state.
   * @param torque Output torque vector that is computed based on damping.
   */
  void CalcTorque(const Context<T>& context, BasicVector<T>* torque) const;
};

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
