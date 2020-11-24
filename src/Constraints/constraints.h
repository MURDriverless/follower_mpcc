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
#include "../Models/dynamic_bicycle.h"

using Eigen::Vector2d;
using Eigen::Matrix;

struct Constraint1D {
    C_i_MPC C_i;
    const double lower_bound;
    const double upper_bound;
};

struct ConstraintsMatrix {
    // dl <= C*xk + D*uk <= du
    C_MPC C;
    D_MPC D;
    d_MPC dl;
    d_MPC du;
};

class Constraints {
public:
    Constraints();
    explicit Constraints(const DynamicBicycleModel &model_args);
    ConstraintsMatrix getConstraints(const Track &track, const State &xk) const;
private:
    static Constraint1D getTrackConstraint(const Track &track, const State &xk, double safety_margin = 0.0);

    // Tire force ellipsis constraint
    Constraint1D getRearTireConstraint(const State &xk) const;
    Constraint1D getFrontTireConstraint(const State &xk) const;

    // Tire slip angle constraint
    Constraint1D getRearAlphaConstraint(const State &xk) const;
    Constraint1D getFrontAlphaConstraint(const State &xk) const;

    DynamicBicycleModel model;

    // Normal forces which remains constant and can be computed during initialisation
    double Fn_front;
    double Fn_rear;
};

#endif //FOLLOWER_MPCC_CONSTRAINTS_H
