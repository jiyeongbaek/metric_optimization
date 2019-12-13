#pragma once

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <metric_optimization/optimization/problem.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>

using namespace std;
using namespace Eigen;

namespace ifopt
{
JointVariables::JointVariables(RobotModelPtr &model) : JointVariables(model, "joint_variable") {}
JointVariables::JointVariables(RobotModelPtr &model, const std::string &name) : VariableSet(7, name), model_(model)
{
    q_.setZero(7);
    // q_(0) = M_PI / 6;
    // q_(3) = -M_PI / 2; //-1.571;
    // q_(5) = M_PI / 2;  //1.571;
    // q_(6) = M_PI / 6;  //0.785;
}

void JointVariables::SetVariables(const Eigen::VectorXd &x)
{
    q_ = x;
}

Eigen::VectorXd JointVariables::GetValues() const
{
    return q_;
}

Component::VecBound JointVariables::GetBounds() const
{
    // TOOD: This is only for Franka-Emika Panda
    Component::VecBound b(GetRows());
    Eigen::MatrixXd joint_limits = model_->getJointLimit();
    for (int i = 0; i < GetRows(); i++)
    {
        b[i] = Bounds(joint_limits(i, 0), joint_limits(i, 1));
    }
    return b;
}
void JointVariables::setInitValue(Eigen::VectorXd q) { q_ = q; }

/* TASK SPACE position constraint*/
PositionConstraint::PositionConstraint(RobotModelPtr &model) : PositionConstraint(model, "joint_variable") {}
PositionConstraint::PositionConstraint(RobotModelPtr &model, const std::string &name) : ConstraintSet(3, name), model_(model) {}
VectorXd PositionConstraint::GetValues() const
{
    VectorXd q = GetVariables()->GetComponent("joint_variable")->GetValues();
    Vector3d g = model_->getTransform(q).translation();
    return g;
}
Component::VecBound PositionConstraint::GetBounds() const
{
    Component::VecBound b(GetRows());
    for (int i = 0; i < GetRows(); i++)
    {
        b[i] = BoundGreaterZero;
    }
    return b;
}

void PositionConstraint::FillJacobianBlock(std::string var_set, Jacobian &jac_block) const
{
    if (var_set == "joint_variable")
    {
        Eigen::VectorXd q = GetVariables()->GetComponent("joint_variable")->GetValues();
        MatrixXd jv = model_->getJacobian(q).block(0, 0, 3, 7);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j <7; j++)
            {
                jac_block.coeffRef(i, j) = jv(i, j);
            }
        }
    }
}

RotationConstraint::RotationConstraint(RobotModelPtr &model) : RotationConstraint(model, "joint_variable") {}
RotationConstraint::RotationConstraint(RobotModelPtr &model, const std::string &name) : ConstraintSet(1, name), model_(model) {}
VectorXd RotationConstraint::GetValues() const
{
    Vector7d q = GetVariables()->GetComponent("joint_variable")->GetValues();
    Matrix3d current_rot = model_->getTransform(q).linear();
    Vector3d axis = model_->getAxis(current_rot);
    // std::cout << "current_rot : " << current_rot << std::endl;
    // std::cout << "current axis : " << axis.transpose() << std::endl;
    VectorXd g(1);
    // g(0) = acos(axis.transpose() * target_axis_);
    // std::cout << "angle : " << g(0) << std::endl;
    g(0) = current_rot(2, 2);
    return g;
}
Component::VecBound RotationConstraint::GetBounds() const
{
    Component::VecBound b(GetRows());
    for (int i = 0; i < GetRows(); i++)
    {
        // b[i] = Bounds(-epsilon_, +epsilon_);
        b[i] = Bounds(-1.0, 1.0);
    }
    return b;
}

void RotationConstraint::FillJacobianBlock(std::string var_set, Jacobian &jac_block) const
{
    // if (var_set == "joint_variable")
    // {
       
    // }
}
void RotationConstraint::setTargetAxis(const Eigen::Vector3d &axis)
{
    target_axis_ = axis;
}

LogEuclideanCost::LogEuclideanCost(RobotModelPtr &model) : LogEuclideanCost(model, "log_cost") {}
LogEuclideanCost::LogEuclideanCost(RobotModelPtr &model, const std::string &name) : CostTerm(name), model_(model) {}
double LogEuclideanCost::GetCost() const
{
    VectorXd q = GetVariables()->GetComponent("joint_variable")->GetValues();
    MatrixXd j_ = model_->getJacobian(q);
    MatrixXd jv_ = j_.block<3, 7>(0, 0);
    Matrix3d current_ = jv_ * jv_.transpose();
    Matrix3d X_ = current_.log() - desired_.log();
    double result = (X_ * X_).trace();
    return result;
}
void LogEuclideanCost::FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
    // if (var_set == "joint_variable") {

    // }
}

double LogEuclideanCost::setDesiredEllipsoid(const Eigen::Matrix3d &desired)
{
    desired_ = desired;
}

} // namespace ifopt
