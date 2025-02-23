#include "./../include/controller_base.h"

namespace drake {
namespace examples {
namespace unitree_g1 {

// Define the constructor
ControllerBase::ControllerBase(const multibody::MultibodyPlant<double>* plant)
    : plant_(plant) {}

}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake
