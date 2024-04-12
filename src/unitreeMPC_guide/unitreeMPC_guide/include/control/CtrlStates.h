//
// Created by shuoy on 10/18/21.
//

#ifndef CTRLSTATES_H
#define CTRLSTATES_H

#include <Eigen/Dense>

#include "common/Params.h"

class CtrlStates {
public:
    // this init function sets all variables to that used in orbit.issac.a1 controller
    CtrlStates() {
        reset();
    }

    void reset() {

        root_pos_d.setZero();
        root_euler_d.setZero();
        root_lin_vel_d.setZero();
        root_ang_vel_d.setZero();

        robot_mass = 12.0;
        inertia << 0.0792, 0.0, 0.0,
                0.0, 0.2085, 0.0,
                0.0, 0.0, 0.2265;

        q_weights.resize(13);
        r_weights.resize(12);


        q_weights << 20.0, 20.0, 20.0,
                0.0, 0.0, 270.0,
                0.0, 0.0, 200.0,
                20.0, 20.0, 80.0,
                0.0;        
        r_weights << 1e-5, 1e-5, 1e-6,
                1e-5, 1e-5, 1e-6,
                1e-5, 1e-5, 1e-6,
                1e-5, 1e-5, 1e-6;

        root_pos.setZero();
        root_quat.setIdentity();
        root_euler.setZero();
        root_rot_mat.setZero();
        root_rot_mat_z.setZero();
        root_lin_vel.setZero();
        root_ang_vel.setZero();
        root_acc.setZero();

        foot_force.setZero();

        foot_pos_target_world.setZero();
        foot_pos_target_abs.setZero();
        foot_pos_target_rel.setZero();
        foot_pos_start.setZero();
        foot_pos_world.setZero();
        foot_pos_abs.setZero();
        foot_pos_rel.setZero();
        foot_pos_abs_mpc.setZero();
        foot_pos_rel_last_time.setZero();
        foot_pos_target_last_time.setZero();
        foot_pos_cur.setZero();
        foot_pos_recent_contact.setZero();
        foot_vel_world.setZero();
        foot_vel_abs.setZero();
        foot_vel_rel.setZero();
        j_foot.setIdentity();
        contacts.setZero();

    }

    // control target
    Eigen::Vector3d root_pos_d;
    Eigen::Vector3d root_euler_d;
    Eigen::Vector3d root_lin_vel_d;
    Eigen::Vector3d root_lin_vel_d_world;
    Eigen::Vector3d root_ang_vel_d;
    Eigen::Vector3d root_ang_vel_d_world;

    Eigen::Matrix<double, MPC_STATE_DIM, 1> mpc_states;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> mpc_states_d;

    // important kinematics constants
    double robot_mass;

    Eigen::Matrix3d inertia;

//     Eigen::Matrix<double, 3, NUM_LEG> default_foot_pos;

    // MPC parameters
    Eigen::VectorXd q_weights;
    Eigen::VectorXd r_weights;

    // terrain related
//     double terrain_pitch_angle;  // the estimated terrain angle on pitch direction

    // important kinematics variables
    Eigen::Vector3d root_pos;
    Eigen::Quaterniond root_quat;
    Eigen::Vector3d root_euler;
    Eigen::Matrix3d root_rot_mat;
    Eigen::Matrix3d root_rot_mat_z;
    Eigen::Vector3d root_lin_vel;
    Eigen::Vector3d root_ang_vel;
    Eigen::Vector3d root_acc;

    Eigen::Vector4d foot_force;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_world; // in the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_abs; // in a frame which centered at the robot frame's origin but parallels to the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_rel; // in the robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_start;

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_world; // in the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs; // in a frame which centered at the robot frame's origin but parallels to the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel; // in the robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs_mpc;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel_last_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_last_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_recent_contact;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_world;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_abs;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_rel;
    Eigen::Matrix<double, 12, 12> j_foot;
    Eigen::Matrix<int, 4, 1> contacts;
};

#endif //A1_CPP_A1CTRLSTATES_H
