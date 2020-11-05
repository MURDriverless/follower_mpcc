//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#ifndef FOLLOWER_MPCC_TYPES_H
#define FOLLOWER_MPCC_TYPES_H

#include <Eigen/Dense>
#include "config.h"

using Eigen::Matrix;

typedef Matrix<double, NX, 1> State;
typedef Matrix<double, NU, 1> Input;

/*
 * Linearisation matrices
 * */
// Linearised state matrix
typedef Eigen::Matrix<double, NX, NX> A_MPC;
// Linearised input matrix
typedef Eigen::Matrix<double, NX, NU> B_MPC;
// Linearised offset such that the dynamics forms -> xk1 = A*xk + B*uk + g
typedef Eigen::Matrix<double, NX, 1> g_MPC;
typedef Matrix<double, NX, NX> A_MPC;
typedef Matrix<double, NX, NU> B_MPC;
typedef Matrix<double, NX, 1> g_MPC;
// Container to store linear model matrices
struct LinModelMatrix {
    A_MPC A;
    B_MPC B;
    g_MPC g;
};



#endif //FOLLOWER_MPCC_TYPES_H
