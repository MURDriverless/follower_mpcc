//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#ifndef FOLLOWER_MPCC_CONSTRAINTS_H
#define FOLLOWER_MPCC_CONSTRAINTS_H

#include <cmath>
#include <Eigen/Dense>
#include "../config.h"
#include "../types.h"
#include "../Splines/track.h"

using Eigen::Vector2d;
using Eigen::Matrix;

struct TrackConstraints {
    Matrix<double, 2, NX> J_p;
    Vector2d lower_bound;
    Vector2d upper_bound;
};

class Constraints {
private:
    static TrackConstraints getTrackConstraint(const Track &track, const State &xk, double safety_margin = 0.0);
};

#endif //FOLLOWER_MPCC_CONSTRAINTS_H
