#include <gtest/gtest.h>
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

class WeldLockTest : public ::testing::Test {
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

        std::cout << "...All Joint Type and Index..." << std::endl;
        for (int i = 0; i < plant_->num_joints(); ++i) {
            const auto& j = plant_->get_joint(drake::multibody::JointIndex(i));
            std::cout << "Joint: " << j.name() << ", Type: " << j.type_name() << ", Index: " << j.index() <<std::endl;
        }

        std::cout << "...None Fix Joint Type and Index..." << std::endl;
        for (int i = 0; i < plant_->num_joints(); ++i) {
            const auto& j = plant_->get_joint(drake::multibody::JointIndex(i));
            if(j.type_name()!="weld") std::cout << "Joint: " << j.name() << ", Type: " << j.type_name() << ", Index: " << j.index() <<std::endl;
        }

    }

    std::unique_ptr<systems::Context<double>> context_;
    std::unique_ptr<MultibodyPlant<double>> plant_;


};

// Test whether WeldFrames() keeps the joint in the state
TEST_F(WeldLockTest, TestWeldFrames) {

    // Apply WeldFrames() before finalizing the plant
    const auto& joint = plant_->GetJointByName<RevoluteJoint>("left_hip_pitch_joint");
    plant_->WeldFrames(plant_->world_frame(), joint.frame_on_child());
    // Finalize the plant after all modeling is complete
    plant_->Finalize();

    // Create the context after finalizing the plant
    context_ = plant_->CreateDefaultContext();
    // Before welding
    int num_positions_before = plant_->num_positions();
    int num_velocities_before = plant_->num_velocities();

    // After welding (since welding is done in SetUp, we check the counts here)
    int num_positions_after = plant_->num_positions();
    int num_velocities_after = plant_->num_velocities();

    EXPECT_EQ(num_positions_after, num_positions_before - 1);  // Welding should remove 1 DOF
    EXPECT_EQ(num_velocities_after, num_velocities_before - 1);
}

// Test whether Lock() correctly removes the joint from the state space
// TEST_F(WeldLockTest, TestLock) {
//     plant_->Finalize();

//     // Create the context after finalizing the plant
//     context_ = plant_->CreateDefaultContext();
//     plant_->GetMutableJointByName<drake::multibody::RevoluteJoint>("left_hip_pitch_joint").Lock(context_.get());
//     // if(plant_->get_joint(drake::multibody::JointIndex(21)).can_rotate()) std::cout<<"can rotate"<<std::endl;


//     // std::cout << "Checking constraints before locking..." << std::endl;
//     // for (int i = 0; i < plant_->num_joints(); ++i) {
//     //     const auto& j = plant_->get_joint(drake::multibody::JointIndex(i));
//     //     std::cout << "Joint: " << j.name() << ", Type: " << j.type_name() << std::endl;
//     // }

//     // Before locking
//     int num_positions_before = plant_->num_positions();
//     int num_velocities_before = plant_->num_velocities();

//     // // After locking
//     int num_positions_after = plant_->num_positions();
//     int num_velocities_after = plant_->num_velocities();

//     EXPECT_EQ(num_positions_after, num_positions_before - 1);  // Locking should remove 1 DOF
//     EXPECT_EQ(num_velocities_after, num_velocities_before - 1);
// }

}  // namespace multibody
}  // namespace drake

// Main function to run all tests
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
