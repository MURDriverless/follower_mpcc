//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#ifndef FOLLOWER_MPCC_CUBIC_SPLINE2D_H
#define FOLLOWER_MPCC_CUBIC_SPLINE2D_H

#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "cubic_spline.h"

using Eigen::VectorXd;
using Eigen::Vector2d;


class CubicSpline2D {
public:
    CubicSpline2D();
    CubicSpline2D(const VectorXd &x_data, const VectorXd &y_data);
    Vector2d getPosition(double s) const;
    Vector2d getDerivative(double s) const;
    Vector2d getSecondDerivative(double s) const;
private:
    static VectorXd calcLineDistances(const VectorXd &x_data, const VectorXd &y_data);
    double constrainInput(double s) const;
    VectorXd s_vector;
    CubicSpline sx;
    CubicSpline sy;
};

#endif //FOLLOWER_MPCC_CUBIC_SPLINE2D_H
