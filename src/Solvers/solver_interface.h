//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#ifndef FOLLOWER_MPCC_SOLVER_INTERFACE_H
#define FOLLOWER_MPCC_SOLVER_INTERFACE_H

#include <array>
#include "../config.h"
#include "../types.h"

using std::array;

struct OptVariables;
struct MPCStage;

class ISolver {
public:
    virtual array<OptVariables, N+1> solveMPC(array<MPCStage, N+1> &stages, const State &xk, int *status);
};

#endif //FOLLOWER_MPCC_SOLVER_INTERFACE_H
