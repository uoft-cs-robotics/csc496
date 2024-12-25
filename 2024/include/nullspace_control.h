#pragma once 
#include "base_controller.h"
#include <eigen3/Eigen/Dense>
#include <franka/rate_limiting.h>
#include "context.h"

class NullSpaceController: public ControllersBase{
public: 
    NullSpaceController(); 
    ~NullSpaceController(); 
    franka::Torques operator()(const franka::RobotState& robot_state, const franka::Duration &period, franka::RobotState initial_state);
private: 
    Eigen::VectorXd ee_goal_pose; 
}; 

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{

	Eigen::JacobiSVD< _Matrix_Type_ > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // For a non-square matrix
        // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

