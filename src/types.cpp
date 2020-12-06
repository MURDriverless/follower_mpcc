//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#include "types.h"

void mpcc::vxNonZero(State xk, double vx_non_zero) {
    if (xk(IndexMap.vx) < vx_non_zero) {
        xk(IndexMap.vx) = vx_non_zero;
        xk(IndexMap.vy) = 0.0;
        xk(IndexMap.wz) = 0.0;
        xk(IndexMap.steering_angle) = 0.0;
    }
}

void mpcc::sanitiseState(State xk, double path_length) {
    if (xk(IndexMap.yaw) > M_PI) {
        xk(IndexMap.yaw) -= 2.0 * M_PI;
    }
    if (xk(IndexMap.yaw) < -M_PI) {
        xk(IndexMap.yaw) += 2.0 * M_PI;
    }
    if (xk(IndexMap.virtual_state) > path_length) {
        xk(IndexMap.virtual_state) -= path_length;
    }
    if (xk(IndexMap.virtual_state) < 0) {
        xk(IndexMap.virtual_state) += path_length;
    }
}
