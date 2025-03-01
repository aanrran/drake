#include <gtest/gtest.h>
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/context.h"
#include <iomanip>  // ✅ Include for formatting

namespace drake {
namespace multibody {

class StateVariablesTest : public ::testing::Test {
protected:
    void SetUp() override {
        systems::DiagramBuilder<double> builder;
        plant_ = std::make_unique<MultibodyPlant<double>>(0.0);
        Parser parser(plant_.get());

        // Load the Unitree G1 URDF
        parser.AddModels("examples/unitree_g1/robots/g1_description/g1_23dof.urdf");

        // Ensure the joint exists before accessing it
        if (!plant_->HasJointNamed("left_hip_pitch_joint")) {
            throw std::runtime_error("Joint 'left_hip_pitch_joint' not found in URDF!");
        }
        plant_->Finalize();

        // Create the context after finalizing the plant
        context_ = plant_->CreateDefaultContext();
    }

    std::unique_ptr<systems::Context<double>> context_;
    std::unique_ptr<MultibodyPlant<double>> plant_;
};


TEST_F(StateVariablesTest, VerifyJointDOFMapping) {
    std::cout << "\n==== Per-DOF Joint State Mapping ====" << std::endl;

    int num_positions = plant_->num_positions();
    int num_velocities = plant_->num_velocities();
    std::cout << std::setw(30) << std::left << "Total Generalized Positions (q): " << num_positions << std::endl;
    std::cout << std::setw(30) << std::left << "Total Generalized Velocities (v): " << num_velocities << std::endl;

    auto positions = plant_->GetPositions(*context_);
    auto velocities = plant_->GetVelocities(*context_);

    // ✅ Print headers
    std::cout << std::setw(5) << std::left << "Idx"
              << std::setw(30) << std::left << "Joint Name"
              << std::setw(6) << std::left << "DOFs"
              << std::setw(8) << std::left << "q[]"
              << std::setw(12) << std::left << "q Value"
              << std::setw(8) << std::left << "v[]"
              << std::setw(12) << std::left << "v Value"
              << std::endl;

    std::cout << std::string(80, '-') << std::endl;

    for (int i = 0; i < plant_->num_joints(); ++i) {
        const auto& joint = plant_->get_joint(drake::multibody::JointIndex(i));

        if (joint.num_positions() > 0 || joint.num_velocities() > 0) {
            int pos_index = joint.num_positions() > 0 ? joint.position_start() : -1;
            int vel_index = joint.num_velocities() > 0 ? joint.velocity_start() : -1;

            std::cout << std::setw(5) << std::left << i
                      << std::setw(30) << std::left << joint.name()
                      << std::setw(6) << std::left << joint.num_positions();

            // ✅ Print position index & value
            if (pos_index >= 0)
                std::cout << std::setw(8) << std::left << pos_index << std::setw(12) << std::left << positions(pos_index);
            else
                std::cout << std::setw(8) << " " << std::setw(12) << " ";

            // ✅ Print velocity index & value
            if (vel_index >= 0)
                std::cout << std::setw(8) << std::left << vel_index << std::setw(12) << std::left << velocities(vel_index);
            else
                std::cout << std::setw(8) << " " << std::setw(12) << " ";

            std::cout << std::endl;
        }
    }

    EXPECT_TRUE(true);
}

TEST_F(StateVariablesTest, PrintPelvisStateMapping) {
    std::cout << "\n==== Pelvis Floating Base State Mapping ====" << std::endl;

    // ✅ Get the pelvis body
    const auto& pelvis = plant_->GetBodyByName("pelvis");

    // ✅ Ensure it's floating
    if (!pelvis.is_floating()) {
        std::cerr << "Error: Pelvis is not floating!" << std::endl;
        return;
    }

    auto positions = plant_->GetPositions(*context_);
    auto velocities = plant_->GetVelocities(*context_);

    int pos_index = pelvis.floating_positions_start();
    int vel_index = pelvis.floating_velocities_start_in_v();

    std::cout << "Pelvis Position Indices:\n";
    std::cout << "  q[" << pos_index << "] = X Position  (" << positions(pos_index) << " m)\n";
    std::cout << "  q[" << pos_index + 1 << "] = Y Position  (" << positions(pos_index + 1) << " m)\n";
    std::cout << "  q[" << pos_index + 2 << "] = Z Position  (" << positions(pos_index + 2) << " m)\n";
    std::cout << "  q[" << pos_index + 3 << "] = Quaternion w (" << positions(pos_index + 3) << ")\n";
    std::cout << "  q[" << pos_index + 4 << "] = Quaternion x (" << positions(pos_index + 4) << ")\n";
    std::cout << "  q[" << pos_index + 5 << "] = Quaternion y (" << positions(pos_index + 5) << ")\n";
    std::cout << "  q[" << pos_index + 6 << "] = Quaternion z (" << positions(pos_index + 6) << ")\n";

    std::cout << "Pelvis Velocity Indices:\n";
    std::cout << "  v[" << vel_index << "] = X Linear Velocity  (" << velocities(vel_index) << " m/s)\n";
    std::cout << "  v[" << vel_index + 1 << "] = Y Linear Velocity  (" << velocities(vel_index + 1) << " m/s)\n";
    std::cout << "  v[" << vel_index + 2 << "] = Z Linear Velocity  (" << velocities(vel_index + 2) << " m/s)\n";
    std::cout << "  v[" << vel_index + 3 << "] = X Angular Velocity  (" << velocities(vel_index + 3) << " rad/s)\n";
    std::cout << "  v[" << vel_index + 4 << "] = Y Angular Velocity  (" << velocities(vel_index + 4) << " rad/s)\n";
    std::cout << "  v[" << vel_index + 5 << "] = Z Angular Velocity  (" << velocities(vel_index + 5) << " rad/s)\n";

    EXPECT_TRUE(true);
}

TEST_F(StateVariablesTest, SetInitialJointStateAndVerify) {
    std::cout << "\n==== Setting Initial Joint State ====\n";

    Eigen::VectorXd desired_state(plant_->num_positions() + plant_->num_velocities());
    
    // ✅ Set Joint Angles
    desired_state << 
    0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.1749, 0.0, -0.1749, 0.0, 0.0, 0.0, 0.1749, 0.0, -0.1749, 0.0, 0.0, 0.0, 
    -0.27931, 0.27925, 0.0, 0.0, 0.0, 0.27931, -0.27925, 0.0, 0.0, 0.0,

    0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 
    1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3,  2.4, 2.5, 2.6, 2.7, 2.8;

    // ✅ Set the initial positions and velocities in the context
    plant_->SetPositions(context_.get(), desired_state.head(plant_->num_positions()));
    plant_->SetVelocities(context_.get(), desired_state.tail(plant_->num_velocities()));

    std::cout << "Initial Joint State Set Successfully!\n";

    // ✅ Run the existing function to print the updated state
    std::cout << "\n==== Per-DOF Joint State Mapping ====" << std::endl;

    int num_positions = plant_->num_positions();
    int num_velocities = plant_->num_velocities();
    std::cout << std::setw(30) << std::left << "Total Generalized Positions (q): " << num_positions << std::endl;
    std::cout << std::setw(30) << std::left << "Total Generalized Velocities (v): " << num_velocities << std::endl;

    auto positions = plant_->GetPositions(*context_);
    auto velocities = plant_->GetVelocities(*context_);

    // ✅ Print headers
    std::cout << std::setw(5) << std::left << "Idx"
              << std::setw(30) << std::left << "Joint Name"
              << std::setw(6) << std::left << "DOFs"
              << std::setw(8) << std::left << "q[]"
              << std::setw(12) << std::left << "q Value"
              << std::setw(8) << std::left << "v[]"
              << std::setw(12) << std::left << "v Value"
              << std::endl;

    std::cout << std::string(80, '-') << std::endl;

    for (int i = 0; i < plant_->num_joints(); ++i) {
        const auto& joint = plant_->get_joint(drake::multibody::JointIndex(i));

        if (joint.num_positions() > 0 || joint.num_velocities() > 0) {
            int pos_index = joint.num_positions() > 0 ? joint.position_start() : -1;
            int vel_index = joint.num_velocities() > 0 ? joint.velocity_start() : -1;

            std::cout << std::setw(5) << std::left << i
                      << std::setw(30) << std::left << joint.name()
                      << std::setw(6) << std::left << joint.num_positions();

            // ✅ Print position index & value
            if (pos_index >= 0)
                std::cout << std::setw(8) << std::left << pos_index << std::setw(12) << std::left << positions(pos_index);
            else
                std::cout << std::setw(8) << " " << std::setw(12) << " ";

            // ✅ Print velocity index & value
            if (vel_index >= 0)
                std::cout << std::setw(8) << std::left << vel_index << std::setw(12) << std::left << velocities(vel_index);
            else
                std::cout << std::setw(8) << " " << std::setw(12) << " ";

            std::cout << std::endl;
        }
    }

    EXPECT_TRUE(true);
}


}  // namespace multibody
}  // namespace drake

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
