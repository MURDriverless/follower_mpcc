//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#include "cost.h"

using Eigen::Vector2d;

Cost::Cost() = default;

Cost::Cost(const CostParams &costParams, const ModelParams &modelParams) {
    this->cost_params = costParams;
    this->model_params = modelParams;
}

CostTerm<Q_MPC, q_MPC> Cost::getRawInputCost() const {
    // Raw input cost is linear, so there are only quadratic terms
    Q_MPC Q = Q_MPC::Zero();
    Q(IndexMap.accel_D, IndexMap.accel_D) = cost_params.r_accel_D;
    Q(IndexMap.steering_angle, IndexMap.steering_angle) = cost_params.r_steering_angle;
    Q(IndexMap.virtual_input, IndexMap.virtual_input) = cost_params.r_virtual_input;
    return { Q, q_MPC::Zero() };
}

CostTerm<R_MPC, r_MPC> Cost::getInputChangeCost() const {
    // Input change cost is also linear, so there are only quadratic terms
    R_MPC R = R_MPC::Zero();
    R(IndexMap.d_accel_D, IndexMap.d_accel_D) = cost_params.r_d_accel_D;
    R(IndexMap.d_steering_angle, IndexMap.d_steering_angle) = cost_params.r_d_steering_angle;
    R(IndexMap.d_virtual_input, IndexMap.d_virtual_input) = cost_params.r_d_virtual_input;
    return { R, r_MPC::Zero() };
}

CostTerm<Z_MPC, z_MPC> Cost::getSoftConstraintsCost() const {
    Z_MPC Z_cost = Z_MPC::Zero();
    z_MPC z_cost = z_MPC::Zero();
    // Quadratic cost
    Z_cost(IndexMap.constraint_track, IndexMap.constraint_track) = cost_params.sc_quad_track;
    Z_cost(IndexMap.constraint_tire, IndexMap.constraint_tire) = cost_params.sc_quad_tire;
    Z_cost(IndexMap.constraint_alpha, IndexMap.constraint_alpha) = cost_params.sc_quad_alpha;
    // Linear cost
    z_cost(IndexMap.constraint_track) = cost_params.sc_lin_track;
    z_cost(IndexMap.constraint_tire) = cost_params.sc_lin_tire;
    z_cost(IndexMap.constraint_alpha) = cost_params.sc_lin_alpha;
    return { Z_cost, z_cost };
}

RefPoint Cost::getRefPoint(const CubicSpline2D &path, const State &xk) {
    const double theta_path = xk(IndexMap.virtual_state);
    const Vector2d x_y = path.getPosition(theta_path);
    const Vector2d dx_dy = path.getDerivative(theta_path);
    const Vector2d ddx_ddy = path.getSecondDerivative(theta_path);

    // Unpack variables
    const double x = x_y(0);
    const double y = x_y(1);
    const double dx = dx_dy(0);
    const double dy = dx_dy(1);
    const double yaw = atan2(dx_dy(1), dx_dy(0));

    // Note that dyaw is not angular rate of change of yaw, because dyaw is partial derivative (we need time
    // for angular rate of change). Instead, we now have the curvature of the path.
    // Formula for curvature = |dx*ddy + dy*ddx| / (dx^2 + dy^2)^(3/2)
    const double ddx = ddx_ddy(0);
    const double ddy = ddx_ddy(1);
    const double dyaw_numerator = dx*ddy + dy*ddx;
    const double dyaw_denominator = pow(dx*dx + dy*dy, 1.5);
    const double dyaw = dyaw_numerator / dyaw_denominator;

    return { x, y, dx, dy, yaw, dyaw };
}

ContouringError Cost::getContouringError(const CubicSpline2D &path, const State &xk) {
    const RefPoint ref = getRefPoint(path, xk);
    Matrix<double, 2, 1> error = Matrix<double, 2, 1>::Zero();
    Matrix<double, 2, NX> d_error = Matrix<double, 2, NX>::Zero();

    // Exact error
    error(0) = sin(ref.yaw)*(xk(IndexMap.X) - ref.x) - cos(ref.yaw)*(xk(IndexMap.Y) - ref.y);
    error(1) = -cos(ref.yaw)*(xk(IndexMap.X) - ref.x) - sin(ref.yaw)*(xk(IndexMap.Y) - ref.y);

    // d_contour_error and d_lag_error are partial derivatives of the errors with respect to the path parameter
    const double d_contour_error = ref.dyaw*cos(ref.yaw)*(xk(IndexMap.X) - ref.x) - ref.dx*sin(ref.yaw) +
                                   ref.dyaw*sin(ref.yaw)*(xk(IndexMap.Y) - ref.y) + ref.dy*cos(ref.yaw);

    const double d_lag_error = ref.dyaw*sin(ref.yaw)*(xk(IndexMap.X) - ref.x) + ref.dx*cos(ref.yaw) -
                               ref.dyaw*cos(ref.yaw)*(xk(IndexMap.Y) - ref.y) + ref.dy*sin(ref.yaw);

    // Jacobian of contouring error
    d_error(0, IndexMap.X) = sin(ref.yaw);
    d_error(0, IndexMap.Y) = -cos(ref.yaw);
    d_error(0, IndexMap.virtual_state) = d_contour_error;

    // Jacobian of lag error
    d_error(1, IndexMap.X) = -cos(ref.yaw);
    d_error(1, IndexMap.Y) = -sin(ref.yaw);
    d_error(1, IndexMap.virtual_state) = d_lag_error;

    return { error, d_error };
}

CostTerm<Q_MPC, q_MPC> Cost::getContouringCost(const CubicSpline2D &path, const State &xk) const {
    const ContouringError contour_error = getContouringError(path, xk);
    const Matrix<double, 2, 1> error_bar = contour_error.error - contour_error.d_error * xk;

    Matrix<double, 2, 2> contouring_weights = Matrix<double, 2, 2>::Zero();
    contouring_weights(0, 0) = cost_params.q_c;
    contouring_weights(1, 1) = cost_params.q_l;

    // TODO: Link Github issue or post in README on the error linearisation maths
    Q_MPC Q = contour_error.d_error.transpose() * contouring_weights * contour_error.d_error;
    q_MPC q = 2 * error_bar.transpose() * contouring_weights * contour_error.d_error;

    // Maximise progression via virtual input
    Q(IndexMap.virtual_input, IndexMap.virtual_input) = -cost_params.q_virtual_input;

    return { Q, q };
}

LinearisedVar Cost::getBetaKin(const State &xk, double lf, double lr) {
    const double delta = xk(IndexMap.steering_angle);

    // Exact value
    const double beta_kin = atan((lr*tan(delta)) / (lf*lf + lr*lr));

    // Partial derivatives with respect to steering angle (delta), computed symbollicaly using Matlab
    Matrix<double, 1, NX> d_beta_kin = Matrix<double, 1, NX>::Zero();
    // Define shorthand for a simpler formula
    const double l_ratio = lr / (lf + lr);
    // delta: l_ratio * (tan(delta)^2 + 1) / (l_ratio^2 * tan(delta)^2 + 1)
    d_beta_kin(0, IndexMap.steering_angle) = l_ratio*(pow(tan(delta), 2)+1.0) /
                                                (pow(l_ratio, 2)*pow(tan(delta), 2) + 1.0);

    const double beta_kin_bar = beta_kin - d_beta_kin*xk;

    return { beta_kin_bar, d_beta_kin };
}

LinearisedVar Cost::getBetaDyn(const State &xk) {
    const double vx = xk(IndexMap.vx);
    const double vy = xk(IndexMap.vy);

    // Exact value
    const double beta_dyn = atan(vy / vx);

    // Partial derivatives with respect to vx and vy, computed symbolically using Matlab
    Matrix<double, 1, NX> d_beta_dyn = Matrix<double, 1, NX>::Zero();
    // vx: -vy/(vx^2 + vy^2)
    d_beta_dyn(0, IndexMap.vx) = (-vy) / (vx*vx + vy*vy);
    // vy: vx/(vx^2 + vy^2)
    d_beta_dyn(0, IndexMap.vy) = (vx) / (vx*vx + vy*vy);

    // Setpoint value from linearisation
    const double beta_dyn_bar = beta_dyn - d_beta_dyn*xk;

    return { beta_dyn_bar, d_beta_dyn };
}

CostTerm<Q_MPC, q_MPC> Cost::getBetaCost(const State &xk) const {
    LinearisedVar beta_kin = getBetaKin(xk, model_params.lf, model_params.lr);
    LinearisedVar beta_dyn = getBetaDyn(xk);

    const double beta_bar = beta_kin.bar - beta_dyn.bar;
    const Matrix<double, 1, NX> d_beta_kin = beta_kin.jac;
    const Matrix<double, 1, NX> d_beta_dyn = beta_dyn.jac;
    const Matrix<double, 1, NX> d_beta = d_beta_kin - d_beta_dyn;

    Q_MPC Q = d_beta.transpose() * cost_params.q_beta * d_beta;
    q_MPC q = 2 * beta_bar * cost_params.q_beta * d_beta;

    return { Q, q };
}

CostMatrix Cost::getCost(const CubicSpline2D &path, const State &xk) {
    CostTerm<Q_MPC, q_MPC> contouring = getContouringCost(path, xk);
    CostTerm<Q_MPC, q_MPC> raw_input = getRawInputCost();
    CostTerm<R_MPC, r_MPC> input_change = getInputChangeCost();
    CostTerm<Z_MPC, z_MPC> soft_constraints = getSoftConstraintsCost();
//    CostTerm<Q_MPC, q_MPC> slip_angle = getBetaCost(xk);

    // Q
    Q_MPC Q_not_sym = contouring.quad_cost + raw_input.quad_cost;
    Q_not_sym = 2.0 * Q_not_sym;
    Q_MPC Q = 0.5 * (Q_not_sym.transpose() + Q_not_sym);

    // R
    R_MPC R = input_change.quad_cost;
    // Solver expects 0.5*uT*R*u
    R = 2.0 * R;

    // S (polytopic cost) will be 0 as it is not used
    S_MPC S = S_MPC::Zero();

    // q
    q_MPC q = contouring.lin_cost + raw_input.lin_cost;
    // Not sure about the line below, this was implemented in AL-MPCC
    q = q + (xk.adjoint()*Q).adjoint();

    // r, which should be zero
    r_MPC r = input_change.lin_cost;

    // Z
    Z_MPC Z = soft_constraints.quad_cost;
    // Similar to Q and R, quad cost has to be multiplied by 2.0
    Z = 2.0 * Z;

    // z
    z_MPC z = soft_constraints.lin_cost;

    return { Q, R, S, q, r, Z, z };
}
