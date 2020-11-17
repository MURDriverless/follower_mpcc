//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#ifndef FOLLOWER_MPCC_COST_H
#define FOLLOWER_MPCC_COST_H

#include <cmath>
#include <Eigen/Dense>
#include "../config.h"
#include "../types.h"
#include "../Splines/cubic_spline2d.h"
#include "../Params/params.h"

using Eigen::Matrix;


// Return type for getCost() function below
struct CostMatrix {
    Q_MPC Q;
    R_MPC R;
    S_MPC S;
    q_MPC q;
    r_MPC r;
    Z_MPC Z;
    z_MPC z;
};


// Return type for contouring, input, soft constraints and slip angle cost functions
template <class QuadCost, class LinCost>
struct CostTerm {
    QuadCost quad_cost;
    LinCost lin_cost;
};


// Position and partial derivatives of x, y and yaw with respect to theta (spline parameter)
struct RefPoint {
    const double x;
    const double y;
    const double dx;
    const double dy;
    const double yaw;
    const double dyaw;
};

// Contains the exact non-linear error and Jacobian of the error at the predicted "xk"
struct ContouringError {
    const Matrix<double, 2, 1> error;
    const Matrix<double, 2, NX> d_error;
};


// General container to hold linearisation variables: setpoint value (bar) and Jacobian (jac)
struct LinearisedVar {
    const double bar;
    const Matrix<double, 1, NX> jac;
};


class Cost {
public:
    Cost();
    Cost(const CostParams &costParams, const ModelParams &modelParams);
    CostMatrix getCost(const CubicSpline2D &path, const State &xk);
private:
    static RefPoint getRefPoint(const CubicSpline2D &path, const State &xk);
    static ContouringError getContouringError(const CubicSpline2D &path, const State &xk);
    CostTerm<Q_MPC, q_MPC> getContouringCost(const CubicSpline2D &path, const State &xk) const;
    CostTerm<Q_MPC, q_MPC> getRawInputCost() const;
    CostTerm<R_MPC, r_MPC> getInputChangeCost() const;
    CostTerm<Z_MPC, z_MPC> getSoftConstraintsCost() const;

    // Helper functions to determine vehicular slip angle (beta; alpha is for tire slip angles)
    static LinearisedVar getBetaKin(const State &xk, double lf, double lr);
    static LinearisedVar getBetaDyn(const State &xk);
    CostTerm<Q_MPC, q_MPC> getBetaCost(const State &xk) const;

    CostParams cost_params;
    ModelParams model_params;
};

#endif //FOLLOWER_MPCC_COST_H
