#include <dlib/optimization.h>
#include <dlib/global_optimization.h>
#include <iostream>
#include <metric_optimization/robot/test.h>
#include <chrono>
#include <unsupported/Eigen/MatrixFunctions>
#include <crtdbg.h>

using namespace std;
using namespace dlib;

int main()
{
    Matrix3d ellipsoid;
    ellipsoid << 0.01, 0, 0,
            0, 0.01, 0,
            0, 0, 0.8;
    Vector7d q;
    RobotRBDLModel model_;
    MatrixXd j_, jv_;
    Matrix3d current_, X_;
    double original_cost, ori_constraint_, pos_log_barrier, u, ori_log_barrier, new_cost;
    Vector3d pos_constraint_;
    u = 100;
    auto GetCost = [&](double q1, double q2, double q3, double q4, double q5, double q6, double q7) {
        q<< q1, q2, q3, q4, q5, q6, q7 ;
        j_ = model_.getJacobian(q);
        jv_ = j_.block<3, 7>(0, 0);       
        current_ = jv_ * jv_.transpose();
         X_ = current_.log() - ellipsoid.log();                
        original_cost = (X_ * X_).trace();
        pos_constraint_ = model_.getTransform(q).translation();
        ori_constraint_ = model_.getTransform(q).linear()(2, 2);
        std::cout <<pos_constraint_(1)<< std::endl;
        pos_log_barrier = log(pos_constraint_(1)); //+ log(pos_constraint_(2));
        u = 100;
        ori_log_barrier = log(-ori_constraint_-0.92);
        new_cost = original_cost - u*(ori_log_barrier + pos_log_barrier);
        return new_cost;
    };

    auto result = find_min_global(GetCost,
                                  {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}, // lower bounds
                                  {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973},       // upper bounds
                                  max_function_calls(100)
                                //   std::chrono::milliseconds(500)                                   // run this long
    );
    cout.precision(9);
    // These cout statements will show that find_min_global() found the
    // globally optimal solution to 9 digits of precision:
    cout << "object function value : " << result.y << endl;
   
    for (int i = 0; i < 7; i++)
        result.x(i) = result.x(i) * 180 / M_PI;
     cout << "solution x:\n"
         << result.x << endl;
    return 0;
}
