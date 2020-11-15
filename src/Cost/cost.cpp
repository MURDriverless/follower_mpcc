//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#include "cost.h"

CostTerm<Q_MPC, q_MPC> Cost::getRawInputCost() const {
    // Raw input cost is linear, so there are only quadratic terms
    Q_MPC Q = Q_MPC::Zero();
    Q(IndexMap.accel_D, IndexMap.accel_D) = cost_params.r_accel_D;
    Q(IndexMap.steering_angle, IndexMap.steering_angle) = cost_params.r_steering_angle;
    Q(IndexMap.virtual_input, IndexMap.virtual_input) = cost_params.r_virtual_input;
    return { Q, q_MPC::Zero() };
}

CostTerm<R_MPC, r_MPC> Cost::getInputChangeCost() const {
    // Input change cost is also linear, so there are only quadratic terms
    R_MPC R = R_MPC::Zero();
    R(IndexMap.d_accel_D, IndexMap.d_accel_D) = cost_params.r_d_accel_D;
    R(IndexMap.d_steering_angle, IndexMap.d_steering_angle) = cost_params.r_d_steering_angle;
    R(IndexMap.d_virtual_input, IndexMap.d_virtual_input) = cost_params.r_d_virtual_input;
    return { R, r_MPC::Zero() };
}

CostTerm<Z_MPC, z_MPC> Cost::getSoftConstraintsCost() const {
    Z_MPC Z_cost = Z_MPC::Zero();
    z_MPC z_cost = z_MPC::Zero();
    // Quadratic cost
    Z_cost(IndexMap.constraint_track, IndexMap.constraint_track) = cost_params.sc_quad_track;
    Z_cost(IndexMap.constraint_tire, IndexMap.constraint_tire) = cost_params.sc_quad_tire;
    Z_cost(IndexMap.constraint_alpha, IndexMap.constraint_alpha) = cost_params.sc_quad_alpha;
    // Linear cost
    z_cost(IndexMap.constraint_track) = cost_params.sc_lin_track;
    z_cost(IndexMap.constraint_tire) = cost_params.sc_lin_tire;
    z_cost(IndexMap.constraint_alpha) = cost_params.sc_lin_alpha;
    return { Z_cost, z_cost };
}


