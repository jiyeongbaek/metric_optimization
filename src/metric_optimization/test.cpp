#include <dlib/optimization.h>
#include <dlib/global_optimization.h>
#include <iostream>
#include <metric_optimization/robot/test.h>
#include <chrono>
#include <metric_optimization/robot/panda_rbdl_model.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <algorithm>
#include <cmath>
using namespace std;
using namespace dlib;

int main()
{
    Matrix3d ellipsoid;
    ellipsoid << 0.04, 0, 0,
        0, 0.6, 0,
        0, 0, 0.85;
    Vector7d q;
    RobotModelPtr model_ = std::make_shared<PandaRBDLModel>();

    MatrixXd j_, jv_;
    Matrix3d current_, X_;
    double original_cost, ori_constraint_, u, log_barrier, new_cost, quad_penalty;
    Vector3d pos_constraint_;
    u = 10000;
    auto GetCost = [&](double q1, double q2, double q3, double q4, double q5, double q6, double q7) {
        q << q1, q2, q3, q4, q5, q6, q7;
        // jv_ = model_->getJacobian(q).block<3, 7>(0, 0);
        jv_ = model_->getJacobian(q).block<3, 7>(0, 0);
        current_ = jv_ * jv_.transpose();
        X_ = current_.log() - ellipsoid.log();
        original_cost = (X_ * X_).trace();
        pos_constraint_ = model_->getTransform(q).translation();
        ori_constraint_ = model_->getTransform(q).linear()(2, 2);
        quad_penalty = pow(max(0.0, -pos_constraint_(0)), 2) + pow(max(0.0, -pos_constraint_(1) + 0.15), 2) + pow(max(0.0, -pos_constraint_(2) + 0.6), 2) 
                    + pow(max(0.0, (ori_constraint_ + 0.995)), 2);
        // u = 0.5*u;
        new_cost = original_cost + u * (quad_penalty);
        return new_cost;
    };

    auto result = find_min_global(GetCost,
                                  {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}, // lower bounds
                                  {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973},       // upper bounds
                                  max_function_calls(10000)
                                  //   std::chrono::milliseconds(1000)                                   // run this long
    );
    cout.precision(9);
    
    Vector7d result_;
    for (int i = 0; i < 7; i++)
    {
        result.x(i) = result.x(i) * 180 / M_PI;
    }
    result_ << -1.39117083, -0.804575646  , 1.14795111 , -1.76591437  ,0.745291994  , 1.51584423 ,-0.477745851;
    std::cout << "solution x: \n"
         << result.x << std::endl;
    std::cout << "object function value   : " << result.y << std::endl;
    MatrixXd jv = model_->getJacobian(result_).block<3, 7>(0, 0);
    Matrix3d jvt = jv*jv.transpose();
    Matrix3d co = jvt.log() - ellipsoid.log();
    std::cout << "original function value : " << (co * co).trace() << std::endl;

    return 0;
}
