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
typedef Matrix<double, NX, NX> A_MPC;
// Linearised input matrix
typedef Matrix<double, NX, NU> B_MPC;
// Linearised offset such that the dynamics forms -> xk1 = A*xk + B*uk + g
typedef Matrix<double, NX, 1> g_MPC;
typedef Matrix<double, NX, NX> A_MPC;
typedef Matrix<double, NX, NU> B_MPC;
typedef Matrix<double, NX, 1> g_MPC;
// Container to store linear model matrices
struct LinModelMatrix {
    A_MPC A;
    B_MPC B;
    g_MPC g;
};

/*
 * Cost terms
 * */
// Quadratic cost
typedef Matrix<double, NX, NX> Q_MPC;  // Error cost
typedef Matrix<double, NU, NU> R_MPC;  // Input cost
typedef Matrix<double, NX, NX> S_MPC;  // Polytopic cost
// Linear cost
typedef Matrix<double, NX, 1> q_MPC;
typedef Matrix<double, NU, 1> r_MPC;
// Soft constraints
typedef Eigen::Matrix<double, NS, NS> Z_MPC;
typedef Eigen::Matrix<double, NS, 1> z_MPC;

/*
 * Constraints and bounds
 * */
// Polytopic constraints
typedef Eigen::Matrix<double, NPC, NX> C_MPC;
typedef Eigen::Matrix<double, 1, NX> C_i_MPC;
typedef Eigen::Matrix<double, NPC, NU> D_MPC;
typedef Eigen::Matrix<double, NPC, 1> d_MPC;

// Bounds matrices
typedef Eigen::Matrix<double, NX, 1> Bounds_x;
typedef Eigen::Matrix<double, NU, 1> Bounds_u;
typedef Eigen::Matrix<double, NS, 1> Bounds_s;


#endif //FOLLOWER_MPCC_TYPES_H
