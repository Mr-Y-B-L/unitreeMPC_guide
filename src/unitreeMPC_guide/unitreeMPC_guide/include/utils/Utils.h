//
// Created by shuoy on 10/19/21.
//

#ifndef A1_CPP_UTILS_H
#define A1_CPP_UTILS_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "common/Params.h"

class Utils {
public:
    // compare to Eigen's default eulerAngles
    // this function returns yaw angle within -pi to pi
    static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);
    static Eigen::Matrix3d skew(Eigen::Vector3d vec);
    static Eigen::Matrix3d pseudo_inverse(const Eigen::Matrix3d &mat);
    static double cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2);
};

#endif //A1_CPP_UTILS_H
