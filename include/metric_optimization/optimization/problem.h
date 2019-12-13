#pragma once

#include <iostream>
#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <metric_optimization/robot/panda_rbdl_model.h>
#include <metric_optimization/robot/robot_model.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
using namespace std;
using namespace Eigen;

namespace ifopt
{
class JointVariables : public VariableSet {
public:
    JointVariables(RobotModelPtr &model);
    JointVariables(RobotModelPtr &model, const std::string &name);

    void SetVariables(const VectorXd &q) override;
    VectorXd GetValues() const override;
    VecBound GetBounds() const override;
    void setInitValue(VectorXd q);

private:
    VectorXd q_;
    RobotModelPtr model_;
};

class PositionConstraint : public ConstraintSet {
public:
    PositionConstraint(RobotModelPtr &model);
    PositionConstraint(RobotModelPtr &model, const std::string &name);

    VectorXd GetValues() const override;
    VecBound GetBounds() const override;

    void FillJacobianBlock(std::string var_set, Jacobian &jac_block) const override;

private:
    VectorXd q_;
    RobotModelPtr model_;
};

class RotationConstraint : public ConstraintSet {
public:
    RotationConstraint(RobotModelPtr &model);
    RotationConstraint(RobotModelPtr &model, const std::string &name);

    VectorXd GetValues() const override;
    VecBound GetBounds() const override;

    void FillJacobianBlock(std::string var_set, Jacobian &jac_block) const override;
    void setTargetAxis(const Eigen::Vector3d &axis);

private:
    VectorXd q_;
    RobotModelPtr model_;
    Eigen::Vector3d target_axis_;
    double epsilon_{0.1}; // 5도정도
};

class LogEuclideanCost : public CostTerm {
public:
    LogEuclideanCost(RobotModelPtr &model);
    LogEuclideanCost(RobotModelPtr &model, const std::string &name);

    double GetCost() const override;
    void FillJacobianBlock(std::string var_set, Jacobian &jac) const override;
    double setDesiredEllipsoid(const Eigen::Matrix3d &desired);

private:
    VectorXd q_;
    RobotModelPtr model_;
    double epsilon_{0.01};
    Matrix3d desired_;
};

} // namespace ifopt
