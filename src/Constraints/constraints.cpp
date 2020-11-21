//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//

#include "constraints.h"

using Eigen::Matrix2d;

TrackConstraints Constraints::getTrackConstraint(const Track &track, const State &xk, double safety_margin) {
    const double s = xk(IndexMap.virtual_state);

    const Vector2d outer_pos = track.outer.getPosition(s);
    const Vector2d centre_pos = track.centre.getPosition(s);

    // diff_vec = [x_diff; y_diff]
    const Vector2d diff_vec = outer_pos - centre_pos;
    const double diff_vec_norm_gain = 1 / (pow(diff_vec(0), 2) + pow(diff_vec(1), 2));

    // G
    const Matrix2d G = diff_vec_norm_gain * Matrix<double, 2, 2> {
        diff_vec(0), diff_vec(1),
        -diff_vec(1), diff_vec(0)
    };

    // Construct Jacobian first
    Matrix<double, 2, NX> J_p = Matrix<double, 2, NX>::Zero();
    // dp_dx
    J_p.col(IndexMap.X) = G * Vector2d {1.0, 0.0};
    // dp_dy
    J_p.col(IndexMap.Y) = G * Vector2d { 0.0, 1.0 };

    // dG_dtheta
    const Vector2d J_centre_pos = track.centre.getDerivative(s);
    const Vector2d J_diff_vec = track.outer.getDerivative(s) - J_centre_pos;
    const Matrix2d dG_dtheta_1 = diff_vec_norm_gain * Matrix2d {
        J_diff_vec(0), J_diff_vec(1),
        -J_diff_vec(1), J_diff_vec(0)
    };
    const Matrix2d dG_dtheta_2 = pow(diff_vec_norm_gain, 2) *
                                (2*diff_vec(0)*J_diff_vec(0) + 2*diff_vec(1)*J_diff_vec(1)) *
                                Matrix2d {
                                    diff_vec(0), diff_vec(1),
                                    -diff_vec(1), diff_vec(0)
                                };
    const Matrix2d dG_dtheta = dG_dtheta_1 - dG_dtheta_2;

    // dp_dtheta
    const Vector2d state_from_centre = Vector2d {
        xk(IndexMap.X) - centre_pos(0),
        xk(IndexMap.Y) - centre_pos(1)
    };
    J_p.col(IndexMap.virtual_state) = -G * J_centre_pos + dG_dtheta * state_from_centre;

    // Compute limits
    const Vector2d p_bar = G * state_from_centre;
    const Vector2d lower_bound = Vector2d {-1.0, -INF} - p_bar;
    const Vector2d upper_bound = Vector2d {1.0, INF} - p_bar;

    return { J_p, lower_bound, upper_bound };
}
