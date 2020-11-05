//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#ifndef FOLLOWER_MPCC_CONFIG_H
#define FOLLOWER_MPCC_CONFIG_H

#define NX 10
#define NU 3

struct StateInputIndex {
    // State indices
    int X = 0;  // global x position
    int Y = 1;  // global y position
    int yaw = 2;  // global heading of car
    int vx = 3;  // linear velocity in local frame
    int vy = 4;  // linear velocity in local frame
    int wz = 5;  // angular velocity around z-axis (yaw)
    int virtual_state = 6;  // progression along the track (virtual state)
    int accel_D = 7;  // duty cycle of acceleration: [-1, 1]
    int steering_angle = 8;  // steering angle in radians
    int virtual_input = 9;  // input to increment theta (virtual state)

    // Input indices
    int d_accel_D = 0;
    int d_steering_angle = 1;
    int d_virtual_input = 2;

    // Constraint indices
    int constraint_track = 0;
    int constraint_tire = 1;
    int constraint_alpha = 2;
};

static const StateInputIndex IndexMap;


#endif //FOLLOWER_MPCC_CONFIG_H
