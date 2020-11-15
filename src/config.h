//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#ifndef FOLLOWER_MPCC_CONFIG_H
#define FOLLOWER_MPCC_CONFIG_H

#define NX 10
#define NU 3

// Number of polytopic constraints (don't worry about the term "polytopic"):
// 1. Track constraint
// 2. Rear tire forces ellipse constraint
// 3. Front tire forces ellipse constraint
// while state and input constraints are termed as lower and upper "bounds"
#define NPC 3
// Number of soft constraints (we have 3 for all our polytopic constraints):
#define NS 3

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
    int constraint_track = 0;  // constrain the car within the track
    int constraint_tire = 1;   // tire force ellipsis
    int constraint_alpha = 2;  // max. slip angle
};

static const StateInputIndex IndexMap;


#endif //FOLLOWER_MPCC_CONFIG_H
