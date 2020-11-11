//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#ifndef FOLLOWER_MPCC_BINARY_SEARCH_H
#define FOLLOWER_MPCC_BINARY_SEARCH_H

#include <iostream>
#include <Eigen/Dense>

using Eigen::VectorXd;

namespace utils {
    int binary_search_left(const VectorXd &arr, double x);
}

#endif //FOLLOWER_MPCC_BINARY_SEARCH_H
