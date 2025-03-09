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
    int n_steps = 5;
    double step_length = 0.5, step_height = 0.1, step_time = 0.5;

    // ✅ Add WalkingFSM
    auto walking_fsm = builder.AddSystem<WalkingFSM>(n_steps, step_length, step_height, step_time);

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
    plt::figure();
    // ✅ Plot the foot placements
    plt::scatter(right_x, right_y, 50, {{"label", "Right Foot"}, {"color", "red"}});
    plt::scatter(left_x, left_y, 50, {{"label", "Left Foot"}, {"color", "blue"}});
    // plt::xlabel("X Position (m)");
    // plt::ylabel("Y Position (m)");
    plt::title("WalkingFSM Foot Placements");
    plt::legend();
    // plt::show();  // ✅ Opens a window to display the plot

    // ✅ Extract ZMP trajectory
    std::vector<double> zmp_x, zmp_y, time_points;
    double dt = 0.05;  // Sampling time for plotting
    for (double t = 0; t <= walking_fsm->get_total_time(); t += dt) {
        Eigen::Vector2d zmp = walking_fsm->get_zmp_trajectory().value(t);
        zmp_x.push_back(zmp(0));
        zmp_y.push_back(zmp(1));
        time_points.push_back(t);
    }

    // ✅ Plot the ZMP trajectory
    plt::plot(zmp_x, zmp_y, {{"label", "ZMP Trajectory"}, {"color", "green"}});
    plt::xlabel("ZMP X Position (m)");
    plt::ylabel("ZMP Y Position (m)");
    plt::title("ZMP Trajectory");
    // ✅ Re-show the plot with ZMP
    plt::legend();
    plt::show();


    // ✅ Create 3D plot for ZMP trajectory
    plt::figure();
    plt::plot3(zmp_x, zmp_y, time_points, {{"label", "ZMP Trajectory"}, {"color", "purple"}});

    // ✅ Label axes
    plt::xlabel("ZMP X Position (m)");
    plt::ylabel("ZMP Y Position (m)");
    plt::set_zlabel("Time (s)");
    plt::title("ZMP Trajectory Over Time");
    plt::legend();

    // ✅ Show the final 3D plot
    // plt::show();

    // ✅ Extract CoM trajectory
    std::vector<double> com_x, com_y, com_time;

    // ✅ Extract CoM trajectory
    for (double t = 0; t <= walking_fsm->get_total_time(); t += dt) {
        Eigen::Vector2d com = walking_fsm->get_com_trajectory().value(t);
        com_x.push_back(com(0));
        com_y.push_back(com(1));
        com_time.push_back(t);
    }

    // ✅ Plot CoM trajectory (on the same figure)
    plt::plot3(com_x, com_y, com_time, {{"label", "CoM Trajectory"}, {"color", "blue"}});

    // ✅ Label axes
    plt::xlabel("X Position (m)");
    plt::ylabel("Y Position (m)");
    plt::set_zlabel("Time (s)");
    plt::title("3D ZMP & CoM Trajectories Over Time");
    plt::legend();

    // ✅ Show the final 3D plot
    plt::show();

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
  