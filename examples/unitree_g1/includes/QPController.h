// QPController.h - Header File
#ifndef QP_CONTROLLER_H
#define QP_CONTROLLER_H

#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/osqp_solver.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <Eigen/Dense>
#include <memory>

class QPController {
public:
    explicit QPController(const drake::multibody::MultibodyPlant<double>& plant);
    Eigen::VectorXd SolveQP(
        const Eigen::VectorXd& q, 
        const Eigen::VectorXd& q_dot, 
        const Eigen::VectorXd& x_task_des, 
        const Eigen::VectorXd& f_task_des);

private:
    const drake::multibody::MultibodyPlant<double>& plant_;
    std::unique_ptr<drake::systems::Context<double>> context_;


    std::vector<drake::multibody::BodyIndex> contact_bodies_;
    std::vector<Eigen::Vector3d> contact_points_;


    std::unique_ptr<drake::solvers::OsqpSolver> solver_;

    Eigen::MatrixXd W_tau_, W_f_, S_, J_task_, J_dot_task_, A_friction_;
    Eigen::VectorXd tau_ref_, f_ref_, tau_min_, tau_max_, b_friction_, ddq_;
};

#endif // QP_CONTROLLER_H
