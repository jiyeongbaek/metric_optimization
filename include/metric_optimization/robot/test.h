#pragma once

#include <Eigen/Dense>
#include <rbdl/rbdl.h>

using namespace std;
using namespace Eigen;
using namespace RigidBodyDynamics;
typedef Eigen::Matrix<double, 7, 1> Vector7d;

class RobotRBDLModel
{
public:
  RobotRBDLModel()
  {
    initialize();
  }

  Affine3d getTransform(const VectorXd &q)
  {
    auto x = CalcBodyToBaseCoordinates(rbdl_model_, q, body_id_[6], ee_position_, true);
    Eigen::Matrix3d rotation = RigidBodyDynamics::CalcBodyWorldOrientation(rbdl_model_, q, body_id_[6], true).transpose();
    Eigen::Matrix3d body_to_ee_rotation;
    body_to_ee_rotation.setZero();
    body_to_ee_rotation(0, 0) = 1;
    body_to_ee_rotation(1, 1) = -1;
    body_to_ee_rotation(2, 2) = -1;

    rotation = rotation * Eigen::AngleAxisd(M_PI / 4., Eigen::Vector3d::UnitZ()) * body_to_ee_rotation;
    // std::cout <<" 2nd : " << rotation << std::endl;

    Eigen::Affine3d transform;
    transform.linear() = rotation;
    transform.translation() = x;
    return transform;
  }

  MatrixXd getJacobian(const VectorXd &q)
  {
    MatrixXd j_temp;
    j_temp.resize(6, 7);
    CalcPointJacobian6D(rbdl_model_, q, body_id_[6], ee_position_, j_temp, true);

    Matrix<double, 6, 7> j;

    for (int i = 0; i < 2; i++)
      j.block<3, 7>(i * 3, 0) = j_temp.block<3, 7>(3 - i * 3, 0);
    return j;
  }

  MatrixXd getJointLimit()
  {
    Eigen::Matrix<double, 7, 2> joint_limits;
    joint_limits << -2.8973, 2.8973,
        -1.7628, 1.7628,
        -2.8973, 2.8973,
        -3.0718, -0.0698,
        -2.8973, 2.8973,
        -0.0175, 3.7525,
        -2.8973, 2.8973;
    return joint_limits;
  }

private:
  void initialize()
  {
    rbdl_model_.gravity = Vector3d(0., 0, -9.81);
    mass[0] = 1.0;
    mass[1] = 1.0;
    mass[2] = 1.0;
    mass[3] = 1.0;
    mass[4] = 1.0;
    mass[5] = 1.0;
    mass[6] = 1.0;

    Vector3d axis[7];
    axis[0] = Vector3d::UnitZ();
    axis[1] = Vector3d::UnitY();
    axis[2] = Vector3d::UnitZ();
    axis[3] = -1.0 * Vector3d::UnitY();
    axis[4] = Vector3d::UnitZ();
    axis[5] = -1.0 * Vector3d::UnitY();
    axis[6] = -1.0 * Vector3d::UnitZ();

    global_joint_position[0] = Eigen::Vector3d(0.0, 0.0, 0.3330);
    global_joint_position[1] = global_joint_position[0];
    global_joint_position[2] = Eigen::Vector3d(0.0, 0.0, 0.6490);
    global_joint_position[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490);
    global_joint_position[4] = Eigen::Vector3d(0.0, 0.0, 1.0330);
    global_joint_position[5] = Eigen::Vector3d(0.0, 0.0, 1.0330);
    global_joint_position[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330);

    joint_position_[0] = global_joint_position[0];
    for (int i = 1; i < 7; i++)
      joint_position_[i] = global_joint_position[i] - global_joint_position[i - 1];

    com_position_[0] = Eigen::Vector3d(0.000096, -0.0346, 0.2575);
    com_position_[1] = Eigen::Vector3d(0.0002, 0.0344, 0.4094);
    com_position_[2] = Eigen::Vector3d(0.0334, 0.0266, 0.6076);
    com_position_[3] = Eigen::Vector3d(0.0331, -0.0266, 0.6914);
    com_position_[4] = Eigen::Vector3d(0.0013, 0.0423, 0.9243);
    com_position_[5] = Eigen::Vector3d(0.0421, -0.0103, 1.0482);
    com_position_[6] = Eigen::Vector3d(0.1, -0.0120, 0.9536);
    ee_position_ = Eigen::Vector3d(0.0880, 0, 0.9260);
    ee_position_ -= global_joint_position[6];

    for (int i = 0; i < 7; i++)
      com_position_[i] -= global_joint_position[i];

    Math::Vector3d inertia[7];
    for (int i = 0; i < 7; i++)
      inertia[i] = Vector3d::Identity() * 0.001;

    for (int i = 0; i < 7; i++)
    {
      body_[i] = Body(mass[i], com_position_[i], inertia[i]);
      joint_[i] = Joint(JointTypeRevolute, axis[i]);
      if (i == 0)
        body_id_[i] = rbdl_model_.AddBody(0, Math::Xtrans(joint_position_[i]), joint_[i], body_[i]);
      else
        body_id_[i] = rbdl_model_.AddBody(body_id_[i - 1], Math::Xtrans(joint_position_[i]), joint_[i], body_[i]);
    }
  }

  RigidBodyDynamics::Model rbdl_model_;
  double mass[7];
  unsigned int body_id_[7];
  Vector3d global_joint_position[7];
  Vector3d joint_position_[7];
  RigidBodyDynamics::Math::Vector3d com_position_[7];
  RigidBodyDynamics::Math::Vector3d ee_position_;
  RigidBodyDynamics::Body body_[7];
  RigidBodyDynamics::Joint joint_[7];
};
