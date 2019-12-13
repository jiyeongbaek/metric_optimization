#pragma once

#include <memory>
#include <Eigen/Dense>
#include <Eigen/Geometry>
class RobotModel
{
public:
  RobotModel() {}

  virtual Eigen::MatrixXd getJacobian(const Eigen::VectorXd &q) = 0;
  virtual Eigen::Affine3d getTransform(const Eigen::VectorXd &q) = 0;
  virtual Eigen::MatrixXd getJointLimit() = 0;
  
  Eigen::Vector3d getPhi(const Eigen::Matrix3d a, const Eigen::Matrix3d b)
  {
    Eigen::Vector3d phi;
    Eigen::Vector3d s[3], v[3], w[3];

    for (int i = 0; i < 3; i++) {
      v[i] = a.block<3, 1>(0, i);
      w[i] = b.block<3, 1>(0, i);
      s[i] = v[i].cross(w[i]);
    }
    phi = s[0] + s[1] + s[2];
    phi = -0.5* phi;

    return phi;
  }
  Eigen::Vector3d getAxis(const Eigen::Matrix3d a)
  {
    Eigen::Vector3d axis;
    Eigen::AngleAxisd AngleAxis(a);
    axis = AngleAxis.axis();
    return axis;
  }
};

typedef std::shared_ptr<RobotModel> RobotModelPtr;
