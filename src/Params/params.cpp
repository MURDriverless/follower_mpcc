//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#include "params.h"

using json = nlohmann::json;

ModelParams::ModelParams() {
    // Assign all fields to be 0.0
    Cm1 = Cm2 = Cr0 = Cr2 = Br = Cr = Dr = Bf = Cf = Df = m = Iz = lf = lr = car_l = car_w = g = 0.0;
    e_long = e_eps = max_alpha = initial_velocity = s_trust_region = vx_zero = 0.0;
}

ModelParams::ModelParams(const std::string &file_path) {
    std::ifstream file_stream(file_path);
    json jsonModel = json::parse(file_stream);

    // Assign to variables
    Cm1 = jsonModel["Cm1"];
    Cm2 = jsonModel["Cm2"];
    Cr0 = jsonModel["Cr0"];
    Cr2 = jsonModel["Cr2"];
    Br 	= jsonModel["Br"];
    Cr 	= jsonModel["Cr"];
    Dr 	= jsonModel["Dr"];
    Bf 	= jsonModel["Bf"];
    Cf 	= jsonModel["Cf"];
    Df 	= jsonModel["Df"];
    m 	= jsonModel["m"];
    Iz 	= jsonModel["Iz"];
    lf 	= jsonModel["lf"];
    lr 	= jsonModel["lr"];
    car_l = jsonModel["car_l"];
    car_w = jsonModel["car_w"];
    g = jsonModel["g"];
    e_long = jsonModel["e_long"];
    e_eps = jsonModel["e_eps"];
    max_alpha = jsonModel["max_alpha"];
    initial_velocity = jsonModel["initial_velocity"];
    s_trust_region = jsonModel["s_trust_region"];
    vx_zero = jsonModel["vx_zero"];
}

CostParams::CostParams() {
    // Assign all fields to be 0.0
    q_c = q_l = q_virtual_input = q_beta = 0.0;
    r_accel_D, r_steering_angle, r_virtual_input = 0.0;
    r_d_accel_D, r_d_steering_angle, r_d_virtual_input = 0.0;
    sc_quad_track = sc_quad_tire = sc_quad_alpha = 0.0;
    sc_lin_track = sc_lin_tire = sc_lin_alpha = 0.0;
}

CostParams::CostParams(const std::string &file_path) {
    std::ifstream file_stream(file_path);
    json jsonCost = json::parse(file_stream);

    // Assign to variables
    q_c = jsonCost["q_c"];
    q_l = jsonCost["q_l"];
    q_virtual_input = jsonCost["q_virtual_input"];
    q_beta = jsonCost["q_beta"];
    r_accel_D = jsonCost["r_accel_D"];
    r_steering_angle = jsonCost["r_steering_angle"];
    r_virtual_input = jsonCost["r_virtual_input"];
    r_d_accel_D = jsonCost["r_d_accel_D"];
    r_d_steering_angle = jsonCost["r_d_steering_angle"];
    r_d_virtual_input = jsonCost["r_d_virtual_input"];
    sc_quad_track = jsonCost["sc_quad_track"];
    sc_quad_tire = jsonCost["sc_quad_tire"];
    sc_quad_alpha = jsonCost["sc_quad_alpha"];
    sc_lin_track = jsonCost["sc_lin_track"];
    sc_lin_tire = jsonCost["sc_lin_tire"];
    sc_lin_alpha = jsonCost["sc_lin_alpha"];
}
