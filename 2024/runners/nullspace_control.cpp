

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/rate_limiting.h>

#include "nullspace_control.h"

namespace robotContext {
    franka::Robot *robot;
    franka::Gripper *gripper;
    franka::Model *model;
}

NullSpaceController::NullSpaceController(){
  ee_goal_pose.resize(DOF);    
  ControllersBase::control_mode_ = ControlMode::ABSOLUTE;
}

NullSpaceController::~NullSpaceController(){}


franka::Torques NullSpaceController::operator()(const franka::RobotState& robot_state, const franka::Duration &period, franka::RobotState initial_state){

    // Compliance parameters
    const double translational_stiffness{150.0};
    const double rotational_stiffness{20.0};
    const double ns_damping{10.0};
    // const double ns_damping =  2.0 * sqrt(ns_stiffness);

    Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    damping.setZero();
    damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                        Eigen::MatrixXd::Identity(3, 3);
    damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                            Eigen::MatrixXd::Identity(3, 3);

                                        
    // get state variables
    std::array<double, 7> coriolis_array = robotContext::model->coriolis(robot_state);
    std::array<double, 42> jacobian_array =
    robotContext::model->zeroJacobian(franka::Frame::kEndEffector, robot_state);
    // convert to Eigen
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.rotation());

    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.rotation());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());    

    double k = 0.2;
    double z_offset = k * (initial_state.elbow[0] - robot_state.elbow[0]);
    position_d(1) += z_offset;




    // translation error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d;

    // orientation error
    // "difference" quaternion
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.rotation() * error.tail(3);    

    // compute control
    Eigen::VectorXd tau_task(7), tau_d(7), tau_ns(7), tau_0(7);
    // for (int i = 0; i < 7; i++)
    // {
    //     tau_0[i] = -1.0 * ns_stiffness * (q[i] - initial_state.q[i]) - ns_damping * dq[i];
    // }         
    // Eigen::Matrix<double, DOF, 6> jacobian_T = jacobian.transpose();
    // Eigen::MatrixXd jacobian_T_inverse = pseudoInverse<Eigen::MatrixXd>(jacobian_T);
    // tau_ns = ( Eigen::Matrix<double, DOF, DOF>::Identity() - jacobian_T * jacobian_T_inverse) * tau_0;



    Eigen::MatrixXd jacobian_inverse = pseudoInverse<Eigen::MatrixXd>(jacobian);
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_ext(robot_state.tau_ext_hat_filtered.data());
    Eigen::Matrix<double, 7, 1> q_uc = -1.0 * tau_ext / ns_damping;
    Eigen::Matrix<double, 7, 1> dq_ns = (Eigen::Matrix<double, DOF, DOF>::Identity() - jacobian_inverse * jacobian) * q_uc;

    // Spring damper system with damping ratio=1
    tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * (dq - dq_ns) ));
    tau_d << tau_task + coriolis;

    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
    return tau_d_array;

}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    try {
        NullSpaceController ns_controller = NullSpaceController();
        franka::Robot robot(argv[1]);
        robotContext::robot = &robot;
        franka::Model model = robot.loadModel();
        robotContext::model = &model;
        franka::RobotState initial_state = robot.readOnce();   
        double time = 0.0;

        robotContext::robot->control([&time, &initial_state, &ns_controller](const franka::RobotState& robot_state,
                                            franka::Duration period) -> franka::Torques {
        time += period.toSec();
        franka::Torques output_torques = ns_controller(robot_state, period, initial_state);

        if (time >= 1000.0) {
            output_torques = {0.0, 0.0 ,0.0, 0.0, 0.0, 0.0, 0.0}; 
            return franka::MotionFinished(output_torques);
        }
        
        return output_torques;
        });  
    }  catch (const franka::Exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
    }
}