//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#ifndef FOLLOWER_MPCC_PARAMS_H
#define FOLLOWER_MPCC_PARAMS_H

#include <string>
#include <fstream>
#include <nlohmann/json.hpp>

class ModelParams {
public:
    ModelParams();
    explicit ModelParams(const std::string &file_path);

    // Coefficients for determining longitudinal forces
    double Cm1;
    double Cm2;
    double Cr0;
    double Cr2;
    // Rear tires coefficients
    double Br;
    double Cr;
    double Dr;
    // Front tires coefficients
    double Bf;
    double Cf;
    double Df;
    // Mass of car
    double m;
    // Moment of inertia about z-axis
    double Iz;
    // Length of front wheel axle to centre of mass
    double lf;
    // Length of rear wheel axle to centre of mass
    double lr;
    // Length of vehicle: sometimes can be approximated to lf + lr
    double car_l;
    // Width of vehicle
    double car_w;
    // Acceleration due to gravity
    double g;
    // Coefficients for tire friction ellipse constraint (see report)
    double e_long;
    double e_eps;
    // Max slip angle
    double max_alpha;
    // initial warm start and trust region (model dependent)
    double initial_velocity;
    double s_trust_region;
    //
    double vx_zero;
};

#endif //FOLLOWER_MPCC_PARAMS_H
