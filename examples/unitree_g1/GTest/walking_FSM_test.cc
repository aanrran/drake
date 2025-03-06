#include <gtest/gtest.h>
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/context.h"
#include "examples/unitree_g1/includes/walking_pattern_generator.h"

namespace drake {
namespace examples {
namespace unitree_g1 {
namespace walking_pattern {

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

class StandingFSMTest : public ::testing::Test {
protected:
    void SetUp() override {
        // ✅ Create DiagramBuilder
        drake::systems::DiagramBuilder<double> builder;

        // ✅ Add MultibodyPlant (for later use)
        plant_ = builder.AddSystem<MultibodyPlant<double>>(0.0);
        Parser parser(plant_);

        // ✅ Load Unitree G1 URDF
        parser.AddModels("examples/unitree_g1/robots/g1_description/g1_23dof.urdf");

        // ✅ Ensure the joint exists before accessing it
        if (!plant_->HasJointNamed("left_hip_pitch_joint")) {
            throw std::runtime_error("Joint 'left_hip_pitch_joint' not found in URDF!");
        }

        plant_->Finalize();  // ✅ Finalize the plant

        // ✅ Add StandingFSM to the system
        standing_fsm_ = builder.AddSystem<StandingFSM>();

        // ✅ Build the complete system into a Diagram
        diagram_ = builder.Build();

        // ✅ Create default context for Diagram
        diagram_context_ = diagram_->CreateDefaultContext();
    }

    std::unique_ptr<drake::systems::Diagram<double>> diagram_;
    std::unique_ptr<drake::systems::Context<double>> diagram_context_;
    MultibodyPlant<double>* plant_;
    StandingFSM* standing_fsm_;
};

// ✅ Test CoM Output While Keeping `plant_`
TEST_F(StandingFSMTest, TestCoMOutput) {
    // ✅ Extract `StandingFSM` context from `diagram_`
    const drake::systems::Context<double>& standing_fsm_context =
        diagram_->GetSubsystemContext(*standing_fsm_, *diagram_context_);

    // ✅ Evaluate the output port
    Eigen::Vector3d com_output = standing_fsm_->get_output_port_com_position().Eval(standing_fsm_context);
    Eigen::Vector3d right_foot_output = standing_fsm_->get_output_port_right_foot_position().Eval(standing_fsm_context);
    Eigen::Vector3d left_foot_output = standing_fsm_->get_output_port_left_foot_position().Eval(standing_fsm_context);

    // ✅ Expected CoM Position (from constructor)
    Eigen::Vector3d expected_com(0.0, 0.0, 0.90);
    // ✅ Expected CoM Position (from constructor)
    Eigen::Vector3d expected_right_foot(-0.065, -0.138, 0.1); 
    // ✅ Expected CoM Position (from constructor)
    Eigen::Vector3d expected_left_foot(-0.065, 0.138, 0.1);
    // ✅ Check output matches expected value
    EXPECT_TRUE(com_output.isApprox(expected_com, 1e-6));
    EXPECT_TRUE(right_foot_output.isApprox(expected_right_foot, 1e-6));
    EXPECT_TRUE(left_foot_output.isApprox(expected_left_foot, 1e-6));
}

// ✅ Test if `plant_` Exists
TEST_F(StandingFSMTest, TestPlantExists) {
    // ✅ Just check if `plant_` is non-null
    EXPECT_NE(plant_, nullptr);
}

}  // namespace walking_pattern
}  // namespace unitree_g1
}  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
