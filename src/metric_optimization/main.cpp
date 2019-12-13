#include <iostream>
#include <ros/ros.h>
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include <metric_optimization/optimization/problem.h>
#include <metric_optimization/robot/panda_rbdl_model.h>

using namespace ifopt;

int main()
{
  RobotModelPtr model_ = std::make_shared<PandaRBDLModel>();

  std::shared_ptr<ifopt::JointVariables> var = std::make_shared<ifopt::JointVariables>(model_);
  std::shared_ptr<ifopt::PositionConstraint> constraint_pos = std::make_shared<ifopt::PositionConstraint>(model_);
  std::shared_ptr<ifopt::RotationConstraint> constraint_ori = std::make_shared<ifopt::RotationConstraint>(model_);
  std::shared_ptr<ifopt::LogEuclideanCost> cost = std::make_shared<ifopt::LogEuclideanCost>(model_);

  Matrix3d ellipsoid;
  Vector3d target_axis_;
  ellipsoid << 0.01, 0, 0,
              0, 0.01, 0,
              0, 0, 0.8;
  target_axis_ << 0.924, 0.383, 0.0; // zaxis
  cost->setDesiredEllipsoid(ellipsoid);
  constraint_ori->setTargetAxis(target_axis_);

  // 1. define the problem
  Problem nlp;
  nlp.AddVariableSet  (var);
  nlp.AddConstraintSet(constraint_pos);
  nlp.AddConstraintSet(constraint_ori);
  nlp.AddCostSet(cost);
  nlp.PrintCurrent();

  // 2. choose solver and options
  IpoptSolver ipopt;
  ipopt.SetOption("linear_solver", "mumps"); //"ma27, ma57, ma77, ma86, ma97" 
  ipopt.SetOption("jacobian_approximation", "finite-difference-values"); //exact or finite difference-values

  // 3 . solve
  ipopt.Solve(nlp);
  Eigen::VectorXd q = nlp.GetOptVariables()->GetValues();
  for (int i = 0; i < 7; i++)
    q[i] = q[i] * 180 / M_PI;
  std::cout << q.transpose() << std::endl;
}
