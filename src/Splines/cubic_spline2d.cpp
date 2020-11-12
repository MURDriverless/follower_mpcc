//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#include "cubic_spline2d.h"

CubicSpline2D::CubicSpline2D() = default;

CubicSpline2D::CubicSpline2D(const VectorXd &x_data, const VectorXd &y_data) {
    s_vector = calcLineDistances(x_data, y_data);
    sx = CubicSpline(s_vector, x_data);
    sy = CubicSpline(s_vector, y_data);
}

VectorXd CubicSpline2D::calcLineDistances(const VectorXd &x_data, const VectorXd &y_data) {
    int n_points = x_data.size();
    VectorXd distances = VectorXd::Zero(n_points);
    double dx, dy;
    int i;
    // Start from 1 as index 0 is the starting point, and will always have distance = 0
    for (i = 1; i < n_points; i++)  {
        dx = x_data(i) - x_data(i-1);
        dy = y_data(i) - y_data(i-1);
        // Cumulative sum
        distances(i) = distances(i-1) + hypot(dx, dy);
    }
    return distances;
}

double CubicSpline2D::constrainInput(double s) const {
    if (s < s_vector(0)) {
        return s_vector(0);
    }
    if (s > s_vector(s_vector.size()-1)) {
        return s_vector(s_vector.size()-1);
    }
    return s;
}

Vector2d CubicSpline2D::getPosition(double s) const {
    s = constrainInput(s);
    Vector2d position = Vector2d::Zero();
    position(0) = sx.getPosition(s);
    position(1) = sy.getPosition(s);
    return position;
}

Vector2d CubicSpline2D::getDerivative(double s) const {
    s = constrainInput(s);
    Vector2d velocity = Vector2d::Zero();
    velocity(0) = sx.getDerivative(s);
    velocity(1) = sy.getDerivative(s);
    return velocity;
}

Vector2d CubicSpline2D::getSecondDerivative(double s) const {
    s = constrainInput(s);
    Vector2d acceleration = Vector2d::Zero();
    acceleration(0) = sx.getSecondDerivative(s);
    acceleration(1) = sy.getSecondDerivative(s);
    return acceleration;
}
