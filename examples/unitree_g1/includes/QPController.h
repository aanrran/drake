// QPController.h - Header File
#ifndef QP_CONTROLLER_H
#define QP_CONTROLLER_H

#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/osqp_solver.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include "drake/multibody/parsing/parser.h"
#include <Eigen/Dense>
#include <memory>

class QPController {
public:
    explicit QPController(const drake::multibody::MultibodyPlant<double>& plant, const drake::systems::Context<double>& context);
    Eigen::VectorXd SolveQP(
        const Eigen::VectorXd& q, 
        const Eigen::VectorXd& q_dot);

private:
    const drake::multibody::MultibodyPlant<double>& plant_;
    const drake::systems::Context<double>& context_;

    // Time step for Euler integration
    double dt_;
    // Angular momentum constraint matrices
    Eigen::MatrixXd A_am_;
    Eigen::VectorXd hdot_des_;
    // Contact information
    std::vector<drake::multibody::BodyIndex> contact_bodies_;
    std::vector<Eigen::Vector3d> contact_points_;


    std::unique_ptr<drake::solvers::OsqpSolver> solver_;

    // System matrices and vectors
    Eigen::MatrixXd W_tau_, W_f_, S_, J_task_, J_dot_task_, A_friction_;
    Eigen::VectorXd tau_ref_, f_ref_, tau_min_, tau_max_, b_friction_;


};

#endif // QP_CONTROLLER_H
