//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#include "mpc.h"

using Eigen::Vector2d;

MPC::MPC() = default;

MPC::MPC(const ModelParams &modelParams_obj, IModel *model_obj, const Cost &cost_obj, const Track &track_obj, double Ts) {
    this->modelParams = modelParams_obj;
    this->prediction_model = model_obj;
    this->cost = cost_obj;
    this->track = track_obj;
    this->Ts = Ts;
}

void MPC::setTrack(const Track &new_track) {
    this->track = new_track;
}

OptVariablesHorizon MPC::updateInitialGuess(const OptVariablesHorizon &previous_guess,
                                            const State &xk, double Ts) const {
    OptVariablesHorizon next_guess;
    // All "future" guesses in previous_guess are moved one time-step back in next_guess
    for (int i = 1; i < N; i++) {
        next_guess[i-1] = previous_guess[i];
    }
    next_guess[0].xi = xk;
    next_guess[0].ui.setZero();

    next_guess[N-1].xi = previous_guess[N-2].xi;
    next_guess[N-1].ui.setZero();

    next_guess[N].xi = prediction_model->predictRK4(next_guess[N-1].xi, next_guess[N-1].ui, Ts);
    next_guess[N].ui.setZero();

    sanitiseInitialGuess(next_guess);
    return next_guess;
}

OptVariablesHorizon MPC::generateNewInitialGuess(const State &xk, double Ts) const {
    OptVariablesHorizon initial_guess;
    initial_guess[0].xi = xk;
    initial_guess[0].ui.setZero();
    for (int i = 1; i <= N; i++) {
        initial_guess[i].xi.setZero();
        initial_guess[i].ui.setZero();

        initial_guess[i].xi(IndexMap.virtual_state) = initial_guess[i-1].xi(IndexMap.virtual_state) +
                                                      Ts*modelParams.initial_velocity;
        Vector2d path_pos = track.path.getPosition(initial_guess[i].xi(IndexMap.virtual_state));
        Vector2d path_dpos = track.path.getDerivative(initial_guess[i].xi(IndexMap.virtual_state));
        initial_guess[i].xi(IndexMap.X) = path_pos(0);
        initial_guess[i].xi(IndexMap.Y) = path_pos(1);
        initial_guess[i].xi(IndexMap.yaw) = atan2(path_dpos(1), path_dpos(0));
        initial_guess[i].xi(IndexMap.vx) = modelParams.initial_velocity;
        initial_guess[i].xi(IndexMap.virtual_input) = modelParams.initial_velocity;
    }
    sanitiseInitialGuess(initial_guess);
    return initial_guess;
}

void MPC::sanitiseInitialGuess(OptVariablesHorizon &initial_guess) const {
    double L = track.path.getLength();
    for (int i = 1; i <= N; i++) {
        if ((initial_guess[i].xi(IndexMap.yaw) - initial_guess[i-1].xi(IndexMap.yaw)) < -M_PI) {
            initial_guess[i].xi(IndexMap.yaw) += 2.0 * M_PI;
        }
        if ((initial_guess[i].xi(IndexMap.yaw) - initial_guess[i-1].xi(IndexMap.yaw)) > M_PI) {
            initial_guess[i].xi(IndexMap.yaw) -= 2.0 * M_PI;
        }
        if ((initial_guess[i].xi(IndexMap.virtual_state) - initial_guess[i-1].xi(IndexMap.virtual_state)) > L/2.0) {
            initial_guess[i].xi(IndexMap.virtual_state) -= L;
        }
    }
}

OptVariablesHorizon MPC::sqpSolutionUpdate(const OptVariablesHorizon &last_solution,
                                           const OptVariablesHorizon &current_solution,
                                           double sqp_mix_ratio) {
    OptVariablesHorizon updated_solution;
    State updated_xi;
    Input updated_ui;
    for (int i = 0; i <= N; i++) {
        updated_xi = sqp_mix_ratio * current_solution[i].xi + (1 - sqp_mix_ratio) * last_solution[i].xi;
        updated_ui = sqp_mix_ratio * current_solution[i].ui + (1 - sqp_mix_ratio) * last_solution[i].ui;
        updated_solution[i].xi = updated_xi;
        updated_solution[i].ui = updated_ui;
    }
    return updated_solution;
}

MPCStageHorizon MPC::setStages(const OptVariablesHorizon &initial_guess, double Ts) const {
    int i, j;  // i = MPC stage index, j = state or input access index
    MPCStageHorizon problem;

    for (i = 0; i <= N; i++) {
        problem[i] = setStage(initial_guess[i].xi, initial_guess[i].ui, Ts, i);
    }

    return problem;
}

MPCStage MPC::setStage(const State &xk, const Input &uk, double Ts, int time_step) const {
    MPCStage stage;
    stage.nx = NX;
    stage.nu = NU;

    if (time_step == 0) {
        stage.ng = 0;
        stage.ns = 0;
    }
    else {
        stage.ng = NPC;
        stage.ns = NS;
    }

    // xk non zero vx, otherwise we get singularity when vx approx. to 0
    mpcc::vxNonZero(xk, modelParams.vx_zero);

    stage.cost_mat = cost.getCost(track.path, xk);
    stage.lin_model = prediction_model->lineariseExpm(xk, uk, Ts);
    stage.constraints_mat = constraints.getConstraints(track, xk);

    stage.x_lower = bounds.getLowerStateBounds(xk);
    stage.x_upper = bounds.getUpperStateBounds(xk);
    stage.u_lower = bounds.getLowerInputBounds(uk);
    stage.u_upper = bounds.getUpperInputBounds(uk);
    stage.s_lower = bounds.getLowerSoftBounds();
    stage.s_upper = bounds.getUpperSoftBounds();

    stage.x_lower(IndexMap.virtual_state) = -modelParams.s_trust_region;
    stage.x_upper(IndexMap.virtual_state) = modelParams.s_trust_region;

    return stage;
}

MPCSolution MPC::runMPC(State &xk) {
    int solver_status = -1;
    xk(IndexMap.virtual_state) = track.path.projectOnSpline(xk);
    mpcc::sanitiseState(xk, track.path.getLength());
    if (valid_initial_guess) {
        initial_guess = updateInitialGuess(initial_guess, xk, Ts);
    } else {
        initial_guess = generateNewInitialGuess(xk, Ts);
    }

    n_no_solves_sqp = 0;
    MPCStageHorizon stages;
    OptVariablesHorizon optimal_solution;
    for (int i = 0; i < n_sqp; i++) {
        stages = setStages(initial_guess, Ts);
        optimal_solution = solver->solveMPC(stages, xk, &solver_status);
        if (solver_status != 0) {
            n_no_solves_sqp++;
        }
        if (solver_status <= 1) {
            initial_guess = sqpSolutionUpdate(initial_guess, optimal_solution, sqp_mixing);
        }
    }

    const int max_error = std::max(n_sqp-1, 1);
    if (n_no_solves_sqp >= max_error) {
        n_non_solves++;
    }
    else {
        n_non_solves = 0;
    }

    

}
