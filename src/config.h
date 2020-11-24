//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#ifndef FOLLOWER_MPCC_CONFIG_H
#define FOLLOWER_MPCC_CONFIG_H

#define NX 10
#define NU 3

// Number of polytopic constraints:
// 1. Track constraint
// 2. Rear tire forces ellipse constraint
// 3. Front tire forces ellipse constraint
// 4. Rear alpha constraint
// 5. Front alpha constraint
// while state and input constraints are termed as lower and upper "bounds"
#define NPC 5
// Number of soft constraints (we have 5 for all our polytopic constraints):
#define NS 5

#define INF 1E5

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
    int constraint_tire_rear = 1;   // rear tire force ellipsis
    int constraint_tire_front = 2;  // front tire force ellipsis
    int constraint_alpha_rear = 3;  // rear tire slip angle
    int constraint_alpha_front = 4; // front tire slip angle
};

static const StateInputIndex IndexMap;


#endif //FOLLOWER_MPCC_CONFIG_H
