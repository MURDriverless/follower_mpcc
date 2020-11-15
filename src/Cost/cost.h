//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#ifndef FOLLOWER_MPCC_COST_H
#define FOLLOWER_MPCC_COST_H

#include "../config.h"
#include "../types.h"
#include "../Splines/cubic_spline2d.h"
#include "../Params/params.h"


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


class Cost {
public:
    CostMatrix getCost();
private:
    CostTerm<Q_MPC, q_MPC> getContouringCost(const CubicSpline2D &path, const State &xk) const;
    CostTerm<Q_MPC, q_MPC> getRawInputCost() const;
    CostTerm<R_MPC, r_MPC> getInputChangeCost() const;
    CostTerm<Z_MPC, z_MPC> getSoftConstraintsCost() const;

    CostParams cost_params;
};

#endif //FOLLOWER_MPCC_COST_H
