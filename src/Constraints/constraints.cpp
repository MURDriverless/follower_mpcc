//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//

#include "constraints.h"

using Eigen::Matrix2d;

Constraints::Constraints() = default;

Constraints::Constraints(const DynamicBicycleModel &model_args) {
    this->model = model_args;
    this->Fn_front = model.params.lr / (model.params.lf + model.params.lr) * model.params.m * model.params.g;
    this->Fn_rear = model.params.lf / (model.params.lf + model.params.lr) * model.params.m * model.params.g;
}

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
    const double cross_track_limit_abs = 1.0 - safety_margin;
    const Vector2d lower_bound = Vector2d {-cross_track_limit_abs, -INF} - p_bar;
    const Vector2d upper_bound = Vector2d {cross_track_limit_abs, INF} - p_bar;

    return { J_p, lower_bound, upper_bound };
}

Constraint1D Constraints::getRearTireConstraint(const State &xk) const {
    // Unpack model params
    const double e_long = model.params.e_long;
    const double e_eps = model.params.e_eps;
    const double Dr = model.params.Dr;

    // Compute helper variables for computing the Jacobian
    TireForces F_rear = model.getForceRear(xk);
    TireForcesDerivatives dF_rear = model.getForceRearDerivatives(xk);
    double e_long_Fn_2 = pow(e_long / Fn_rear, 2);
    double Fn_inv_2 = pow(1.0/Fn_rear, 2);

    // TC (Tire Constraints) = (param_.e_long*(F_{rear,x}/Fn_rear))^2 + (F_{rear,y}/Fn_rear)^2
    // Check dynamic_bicycle.cpp to identify non-zero derivatives,
    // and include that in the partial derivatives below.
    const double dTC_dvx = e_long_Fn_2 * 2.0 * F_rear.Fx * dF_rear.dFx_vx +
                           Fn_inv_2 * 2.0 * F_rear.Fy * dF_rear.dFy_vx;
    const double dTC_dvy = Fn_inv_2 * 2.0 * F_rear.Fy * dF_rear.dFy_vy;
    const double dTC_dwz = Fn_inv_2 * 2.0 * F_rear.Fy * dF_rear.dFy_wz;
    const double dTC_daccel_D = e_long_Fn_2 * 2.0 * F_rear.Fx * dF_rear.dFx_accel_D;

    // Compute Jacobian
    C_i_MPC J_TC = C_i_MPC::Zero();
    J_TC(IndexMap.vx) = dTC_dvx;
    J_TC(IndexMap.vy) = dTC_dvy;
    J_TC(IndexMap.wz) = dTC_dwz;
    J_TC(IndexMap.accel_D) = dTC_daccel_D;

    // Compute constraint bounds using the following equation
    // 0 <= J_TC*(x - x0) + TC <= F_max
    // 0 <= J_TC*x + (-J_TC*x0 + TC) <= F_max
    // J_TC*x0 - TC <= J_TC*x <= F_max + J_TC*x0 - TC
    // where x is the decision variable and x0 is our linearisation point (xk in our definition).
    const double TC = pow(e_long*F_rear.Fx/Fn_rear, 2) + pow(F_rear.Fy/Fn_rear, 2);
    const double F_max = pow(e_eps * Dr, 2);
    const double lower_bound = J_TC*xk - TC;
    const double upper_bound = J_TC*xk + F_max - TC;

    return { J_TC, lower_bound, upper_bound };
}

Constraint1D Constraints::getFrontTireConstraint(const State &xk) const {
    // Unpack model params
    const double e_long = model.params.e_long;
    const double e_eps = model.params.e_eps;
    const double Df = model.params.Df;

    // Compute helper variables for computing the Jacobian
    TireForces F_front = model.getForceFront(xk);
    TireForcesDerivatives dF_front = model.getForceFrontDerivatives(xk);
    double Fn_inv_2 = pow(1.0/Fn_front, 2);

    // TC (Tire Constraints) = (param_.e_long*(F_{front,x}/Fn_rear))^2 + (F_{front,y}/Fn_rear)^2
    // Check dynamic_bicycle.cpp to identify non-zero derivatives,
    // and include that in the partial derivatives below.
    const double dTC_dvx = Fn_inv_2 * 2.0 * F_front.Fy * dF_front.dFy_vx;
    const double dTC_dvy = Fn_inv_2 * 2.0 * F_front.Fy * dF_front.dFy_vy;
    const double dTC_dwz = Fn_inv_2 * 2.0 * F_front.Fy * dF_front.dFy_wz;
    const double dTC_dsteering_angle = 2.0 * F_front.Fy * dF_front.dFy_steering_angle;

    // Compute Jacobian
    C_i_MPC J_TC = C_i_MPC::Zero();
    J_TC(IndexMap.vx) = dTC_dvx;
    J_TC(IndexMap.vy) = dTC_dvy;
    J_TC(IndexMap.wz) = dTC_dwz;
    J_TC(IndexMap.steering_angle) = dTC_dsteering_angle;

    // Compute constraint bounds using the following equation
    // 0 <= J_TC*(x - x0) + TC <= F_max
    // 0 <= J_TC*x + (-J_TC*x0 + TC) <= F_max
    // J_TC*x0 - TC <= J_TC*x <= F_max + J_TC*x0 - TC
    // where x is the decision variable and x0 is our linearisation point (xk in our definition).
    const double TC = pow(e_long * F_front.Fx / Fn_front, 2) + pow(F_front.Fy / Fn_front, 2);
    const double F_max = pow(e_eps * Df, 2);
    const double lower_bound = J_TC*xk - TC;
    const double upper_bound = J_TC*xk + F_max - TC;

    return { J_TC, lower_bound, upper_bound };
}
