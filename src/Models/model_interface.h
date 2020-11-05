//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#ifndef FOLLOWER_MPCC_MODEL_INTERFACE_H
#define FOLLOWER_MPCC_MODEL_INTERFACE_H

#include "../types.h"

class IModel {
public:
    // Predict the next state using 4th order Runge-Katta approximate discretisation
    virtual State predictRK4(const State &xk, const Input &uk, double Ts) const = 0;

    // For linearising the Jacobians, Eigen has exp() which is the exponential matrix function
    virtual LinModelMatrix lineariseExpm(const State &xk, const Input &uk, double Ts) const = 0;
};

#endif //FOLLOWER_MPCC_MODEL_INTERFACE_H
