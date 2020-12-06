//
// Created by Dennis Wirya (dwirya@student.unimelb.edu.au).
// Copyright (c) 2020 MUR Driverless. All rights reserved.
//
#ifndef FOLLOWER_MPCC_MPC_H
#define FOLLOWER_MPCC_MPC_H

#include <array>
#include <cmath>
#include <Eigen/Dense>
#include "../config.h"
#include "../types.h"
#include "../Cost/cost.h"
#include "../Models/model_interface.h"
#include "../Constraints/constraints.h"
#include "../Constraints/bounds.h"
#include "../Splines/track.h"
#include "../Params/params.h"
#include "../Solvers/solver_interface.h"

using std::array;

struct OptVariables {
    State xi;
    Input ui;
};

typedef array<OptVariables, N + 1> OptVariablesHorizon;

struct MPCStage {
    CostMatrix cost_mat;
    LinModelMatrix lin_model;
    ConstraintsMatrix constraints_mat;
    Bounds_x x_lower;
    Bounds_x x_upper;
    Bounds_u u_lower;
    Bounds_u u_upper;
    Bounds_s s_lower;
    Bounds_s s_upper;

    // nx    -> number of states
    // nu    -> number of inputs
    // nbx   -> number of bounds on x
    // nbu   -> number of bounds on u
    // ng    -> number of polytopic constraints
    // ns   -> number of soft constraints
    int nx, nu, nbx, nbu, ng, ns;
};

typedef array<MPCStage, N + 1> MPCStageHorizon;

struct MPCSolution {
    const Input u_sol;
    const array<OptVariables, N + 1> xi;
    const double exec_time;
};

class MPC {
public:
    MPC();
    MPC(const ModelParams &modelParams_obj, IModel *model_obj, const Cost &cost_obj, const Track &track_obj, double Ts);
    MPCSolution runMPC(State &xk);
    void setTrack(const Track &new_track);
    OptVariablesHorizon updateInitialGuess(const OptVariablesHorizon &previous_guess, const State &xk, double Ts) const;
    OptVariablesHorizon generateNewInitialGuess(const State &xk, double Ts) const;
    void sanitiseInitialGuess(OptVariablesHorizon &initial_guess) const;
    static OptVariablesHorizon sqpSolutionUpdate(const OptVariablesHorizon &last_solution,
                                                 const OptVariablesHorizon &current_solution,
                                                 double sqp_mix_ratio);
    MPCStageHorizon setStages(const OptVariablesHorizon &initial_guess, double Ts) const;
    MPCStage setStage(const State &xk, const Input &uk, double Ts, int time_step) const;
private:
    double Ts;
    bool valid_initial_guess;
    OptVariablesHorizon initial_guess;
    int n_sqp;
    double sqp_mixing;
    int n_non_solves;
    int n_no_solves_sqp;
    int n_reset;
    ISolver *solver;
    IModel *prediction_model;
    ModelParams modelParams;
    Cost cost;
    Constraints constraints;
    Bounds bounds;
    Track track;
};

#endif //FOLLOWER_MPCC_MPC_H
