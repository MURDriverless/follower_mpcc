//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#include "bounds.h"

Bounds::Bounds() {
    x_lower.setZero();
    x_upper.setZero();
    u_lower.setZero();
    u_upper.setZero();
    s_lower.setZero();
    s_upper.setZero();
}

Bounds::Bounds(const std::string &file_path) {
    std::ifstream file_stream(file_path);
    json jsonBounds = json::parse(file_stream);
    setLowerStateBounds(jsonBounds);
    setUpperStateBounds(jsonBounds);
    setLowerInputBounds(jsonBounds);
    setUpperInputBounds(jsonBounds);
    setLowerSoftBounds(jsonBounds);
    setUpperSoftBounds(jsonBounds);
}

/*
 * Public getters
 * */
Bounds_x Bounds::getLowerStateBounds(const State &xk) const { return x_lower - xk; }
Bounds_x Bounds::getUpperStateBounds(const State &xk) const { return x_upper - xk; }
Bounds_u Bounds::getLowerInputBounds(const Input &uk) const { return u_lower - uk; }
Bounds_u Bounds::getUpperInputBounds(const Input &uk) const { return u_upper - uk; }
Bounds_s Bounds::getLowerSoftBounds() const { return s_lower; }
Bounds_s Bounds::getUpperSoftBounds() const { return s_upper; }

/*
 * Private setters
 * */
void Bounds::setLowerStateBounds(const json &jsonBounds) {
    x_lower(IndexMap.X) = jsonBounds["X_l"];
    x_lower(IndexMap.Y) = jsonBounds["Y_l"];
    x_lower(IndexMap.yaw) = jsonBounds["yaw_l"];
    x_lower(IndexMap.vx) = jsonBounds["vx_l"];
    x_lower(IndexMap.vy) = jsonBounds["vy_l"];
    x_lower(IndexMap.wz) = jsonBounds["wz_l"];
    x_lower(IndexMap.virtual_state) = jsonBounds["virtual_state_l"];
    x_lower(IndexMap.accel_D) = jsonBounds["accel_D_l"];
    x_lower(IndexMap.steering_angle) = jsonBounds["steering_angle_l"];
    x_lower(IndexMap.virtual_input) = jsonBounds["virtual_input_l"];
}

void Bounds::setUpperStateBounds(const json &jsonBounds) {
    x_upper(IndexMap.X) = jsonBounds["X_u"];
    x_upper(IndexMap.Y) = jsonBounds["Y_u"];
    x_upper(IndexMap.yaw) = jsonBounds["yaw_u"];
    x_upper(IndexMap.vx) = jsonBounds["vx_u"];
    x_upper(IndexMap.vy) = jsonBounds["vy_u"];
    x_upper(IndexMap.wz) = jsonBounds["wz_u"];
    x_upper(IndexMap.virtual_state) = jsonBounds["virtual_state_u"];
    x_upper(IndexMap.accel_D) = jsonBounds["accel_D_u"];
    x_upper(IndexMap.steering_angle) = jsonBounds["steering_angle_u"];
    x_upper(IndexMap.virtual_input) = jsonBounds["virtual_input_u"];
}

void Bounds::setLowerInputBounds(const json &jsonBounds) {
    u_lower(IndexMap.d_accel_D) = jsonBounds["d_accel_D_lower"];
    u_lower(IndexMap.d_steering_angle) = jsonBounds["d_steering_angle_lower"];
    u_lower(IndexMap.d_virtual_input) = jsonBounds["d_virtual_input_lower"];
}

void Bounds::setUpperInputBounds(const json &jsonBounds) {
    u_upper(IndexMap.d_accel_D) = jsonBounds["d_accel_D_upper"];
    u_upper(IndexMap.d_steering_angle) = jsonBounds["d_steering_angle_upper"];
    u_upper(IndexMap.d_virtual_input) = jsonBounds["d_virtual_input_upper"];
}

void Bounds::setLowerSoftBounds(const json &jsonBounds) {
    // Set zero for now
    s_lower.setZero();
}

void Bounds::setUpperSoftBounds(const json &jsonBounds) {
    // Set zero for now
    s_upper.setZero();
}
