//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#ifndef FOLLOWER_MPCC_BOUNDS_H
#define FOLLOWER_MPCC_BOUNDS_H

#include <fstream>
#include <nlohmann/json.hpp>
#include "../types.h"

using json = nlohmann::json;


class Bounds {
public:
    Bounds();
    explicit Bounds(const std::string &file_path);
    Bounds_x getLowerStateBounds();
    Bounds_x getUpperStateBounds();
    Bounds_u getLowerInputBounds();
    Bounds_u getUpperInputBounds();
    Bounds_s getLowerSoftBounds();
    Bounds_s getUpperSoftBounds();
private:
    Bounds_x x_lower;
    Bounds_x x_upper;
    Bounds_u u_lower;
    Bounds_u u_upper;
    Bounds_s s_lower;
    Bounds_s s_upper;
    void setLowerStateBounds(const json &jsonBounds);
    void setUpperStateBounds(const json &jsonBounds);
    void setLowerInputBounds(const json &jsonBounds);
    void setUpperInputBounds(const json &jsonBounds);
    void setLowerSoftBounds(const json &jsonBounds);
    void setUpperSoftBounds(const json &jsonBounds);
};

#endif //FOLLOWER_MPCC_BOUNDS_H
