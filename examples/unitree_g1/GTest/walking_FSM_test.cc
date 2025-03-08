#include <gflags/gflags.h>
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/context.h"
#include <matplotlibcpp.h> 
#include "examples/unitree_g1/includes/walking_pattern_generator.h"

namespace plt = matplotlibcpp;

namespace drake {
namespace examples {
namespace unitree_g1 {
namespace walking_pattern {

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

int do_main() {
    drake::systems::DiagramBuilder<double> builder;

    // ✅ Add WalkingFSM
    auto walking_fsm = builder.AddSystem<WalkingFSM>(11, 0.5, 0.1, 0.5);

    // ✅ Build Diagram
    auto diagram = builder.Build();

    // ✅ Create default context for Diagram
    auto diagram_context = diagram->CreateDefaultContext();

    std::vector<double> right_x, right_y;
    std::vector<double> left_x, left_y;

    // ✅ Extract footstep placements from WalkingFSM
    for (const auto& pos : walking_fsm->get_right_foot_placements()) {
        right_x.push_back(pos(0));
        right_y.push_back(pos(1));
    }
    for (const auto& pos : walking_fsm->get_left_foot_placements()) {
        left_x.push_back(pos(0));
        left_y.push_back(pos(1));
    }

    // ✅ Plot the foot placements
    plt::scatter(right_x, right_y, 50, {{"label", "Right Foot"}, {"color", "red"}});
    plt::scatter(left_x, left_y, 50, {{"label", "Left Foot"}, {"color", "blue"}});
    plt::xlabel("X Position (m)");
    plt::ylabel("Y Position (m)");
    plt::title("WalkingFSM Foot Placements");
    plt::legend();
    plt::show();  // ✅ Opens a window to display the plot

    return 0;  // ✅ Normal program exit
}

}  // namespace walking_pattern
}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return drake::examples::unitree_g1::walking_pattern::do_main();
  }
  