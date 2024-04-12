//
// Created by shuoy on 10/19/21.
//

#include "utils/Utils.h"

Eigen::Vector3d Utils::quat_to_euler(Eigen::Quaterniond quat) {
    Eigen::Vector3d rst;

    // order https://github.com/libigl/eigen/blob/master/Eigen/src/Geometry/Quaternion.h
    Eigen::Matrix<double, 4, 1> coeff = quat.coeffs();
    double x = coeff(0);
    double y = coeff(1);
    double z = coeff(2);
    double w = coeff(3);

    double y_sqr = y*y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y_sqr);

    rst[0] = atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > +1.0 ? +1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    rst[1] = asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y_sqr + z * z);
    rst[2] = atan2(t3, t4);
    return rst;
}

Eigen::Matrix3d Utils::skew(Eigen::Vector3d vec) {
    Eigen::Matrix3d rst; rst.setZero();
    rst <<            0, -vec(2),  vec(1),
            vec(2),             0, -vec(0),
            -vec(1),  vec(0),             0;
    return rst;
}

// https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f
Eigen::Matrix3d Utils::pseudo_inverse(const Eigen::Matrix3d &mat) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double epsilon = std::numeric_limits<double>::epsilon();
    // For a non-square matrix
    // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(mat.cols(), mat.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() *
           svd.matrixU().adjoint();
}

double Utils::cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2) {
    // surface 1: a1 * x + b1 * y + c1 * z + d1 = 0, coef: [a1, b1, c1]
    // surface 2: a2 * x + b2 * y + c2 * z + d2 = 0, coef: [a1, b2, c2]
    double angle_cos =
            abs(surf_coef_1[0] * surf_coef_2[0] + surf_coef_1[1] * surf_coef_2[1] + surf_coef_1[2] * surf_coef_2[2])
            / (sqrt(surf_coef_1[0] * surf_coef_1[0] + surf_coef_1[1] * surf_coef_1[1] + surf_coef_1[2] * surf_coef_1[2]) *
               sqrt(surf_coef_2[0] * surf_coef_2[0] + surf_coef_2[1] * surf_coef_2[1] + surf_coef_2[2] * surf_coef_2[2]));
    return acos(angle_cos);
}
