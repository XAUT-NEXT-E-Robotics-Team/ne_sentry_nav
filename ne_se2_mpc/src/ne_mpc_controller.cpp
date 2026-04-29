/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-04-28 15:39:55
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-04-28 15:46:04
 * @FilePath: /ne_nav/src/ne_se2_mpc/src/ne_mpc_controller.cpp
 * @Description: 我永远喜欢雪之下雪乃
 *
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved.
 */
#include "ne_se2_mpc/ne_mpc_controller.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace mpc_slover
{
    namespace
    {
        constexpr int kStateDim = 6;
        constexpr int kControlDim = 3;

        template <typename T>
        void loadScalarIfPresent(const YAML::Node & node, const char * key, T * value)
        {
            if (node[key]) {
                *value = node[key].as<T>();
            }
        }

        template <int Dim>
        bool loadFixedVectorIfPresent(
            const YAML::Node & node,
            const char * key,
            Eigen::Matrix<double, Dim, 1> * value)
        {
            if (!node[key]) {
                return false;
            }

            const auto data = node[key].as<std::vector<double>>();
            if (static_cast<int>(data.size()) != Dim) {
                return false;
            }

            for (int i = 0; i < Dim; ++i) {
                (*value)(i) = data[static_cast<size_t>(i)];
            }
            return true;
        }
    }  // namespace

    MpcSlover::MpcSlover(const MpcConfig & config)
    {
        initialize(config);
    }

    bool MpcSlover::initialize(const MpcConfig & config)
    {
        if (config.dt <= 0.0 || config.horizon <= 0 || config.mass <= 0.0 || config.iz <= 0.0) {
            return false;
        }

        config_ = config;
        buildDiscreteModel();
        buildPredictionModel();
        buildCostMatrices();
        buildStaticConstraints();

        const int control_vars = kControlDim * config_.horizon;
        gradient_ = Eigen::VectorXd::Zero(control_vars);
        solution_ = Eigen::VectorXd::Zero(control_vars);
        qpoases_solver_ = std::make_unique<qpOASES::QProblem>(control_vars, total_constraints_);
        qpoases_solver_initialized_ = false;
        qp_initialized_ = true;
        return true;
    }

    bool MpcSlover::initializeFromYaml(const std::string & yaml_path, const std::string & root_key)
    {
        MpcConfig config;

        try {
            const YAML::Node yaml = YAML::LoadFile(yaml_path);
            const YAML::Node node = yaml[root_key];
            if (!node) {
                return initialize(config);
            }

            loadScalarIfPresent(node, "dt", &config.dt);
            loadScalarIfPresent(node, "horizon", &config.horizon);
            loadScalarIfPresent(node, "mass", &config.mass);
            loadScalarIfPresent(node, "iz", &config.iz);
            loadScalarIfPresent(node, "gravity", &config.gravity);
            loadScalarIfPresent(node, "max_preview_time", &config.max_preview_time);
            loadScalarIfPresent(node, "projection_sample_dt", &config.projection_sample_dt);
            loadScalarIfPresent(node, "vx_max", &config.vx_max);
            loadScalarIfPresent(node, "vy_max", &config.vy_max);
            loadScalarIfPresent(node, "omega_max", &config.omega_max);
            loadScalarIfPresent(node, "friction_coeff", &config.friction_coeff);

            static_cast<void>(loadFixedVectorIfPresent<kStateDim>(node, "q_diag", &config.q_diag));
            static_cast<void>(loadFixedVectorIfPresent<kControlDim>(node, "r_diag", &config.r_diag));
            static_cast<void>(loadFixedVectorIfPresent<kControlDim>(node, "u_min", &config.u_min));
            static_cast<void>(loadFixedVectorIfPresent<kControlDim>(node, "u_max", &config.u_max));
        } catch (const YAML::Exception &) {
            return false;
        }

        return initialize(config);
    }

    void MpcSlover::setTrajectoryOptimizer(const std::shared_ptr<ne_traj_opt::TrajectoryOptimize> & optimizer)
    {
        traj_wait_ = optimizer;
        direct_traj_.reset();
    }

    void MpcSlover::setReferenceTrajectory(const Trajectory2D & trajectory)
    {
        direct_traj_ = std::make_shared<Trajectory2D>(trajectory);
        traj_wait_.reset();
    }

    bool MpcSlover::getActiveTrajectory(Trajectory2D * trajectory) const
    {
        if (trajectory == nullptr) {
            return false;
        }

        if (direct_traj_ && direct_traj_->isInitialized()) {
            *trajectory = *direct_traj_;
            return true;
        }

        if (traj_wait_) {
            const auto optimizer_trajectory = traj_wait_->trajectorygetter();
            if (optimizer_trajectory.isInitialized()) {
                *trajectory = optimizer_trajectory;
                return true;
            }
        }

        return false;
    }

    mpc_state::state MpcSlover::sampleStateFromTrajectory(
        const Trajectory2D & trajectory,
        double t) const
    {
        const auto pos = trajectory.getPos(t);
        const auto vel = trajectory.getVel(t);
        const auto acc = trajectory.getAcc(t);

        double yaw = 0.0;
        if (vel.norm() > 1e-6) {
            yaw = std::atan2(vel.y(), vel.x());
        }

        double omega = 0.0;
        const double vel_sq_norm = vel.squaredNorm();
        if (vel_sq_norm > 1e-6) {
            omega = (vel.x() * acc.y() - vel.y() * acc.x()) / vel_sq_norm;
        }

        mpc_state::state state = mpc_state::state::Zero();
        state << pos.x(),
                 pos.y(),
                 yaw,
                 vel.x(),
                 vel.y(),
                 omega;
        return state;
    }

    bool MpcSlover::state_init()
    {
        Trajectory2D trajectory;
        if (!getActiveTrajectory(&trajectory)) {
            return false;
        }

        projected_time_ = trajectory.getStartTime();
        input_state = sampleStateFromTrajectory(trajectory, projected_time_);
        init_ = true;
        return true;
    }

    double MpcSlover::projectCurrentStateToTrajectory(double sample_dt, double search_window)
    {
        Trajectory2D trajectory;
        if (!getActiveTrajectory(&trajectory) || sample_dt <= 0.0 || search_window <= 0.0) {
            return 0.0;
        }

        const double start_t = trajectory.getStartTime();
        const double end_t = trajectory.getEndTime();
        const double search_start = init_ ? std::max(start_t, projected_time_ - 0.5 * search_window) : start_t;
        const double search_end = init_ ? std::min(end_t, projected_time_ + search_window) : end_t;

        const Eigen::Vector2d current_pos(input_state(0), input_state(1));
        double best_t = search_start;
        double best_dist_sq = std::numeric_limits<double>::max();

        for (double t = search_start; t <= search_end + 1e-9; t += sample_dt) {
            const Eigen::Vector2d pos = trajectory.getPos(t);
            const double dist_sq = (pos - current_pos).squaredNorm();
            if (dist_sq < best_dist_sq) {
                best_dist_sq = dist_sq;
                best_t = t;
            }
        }

        const Eigen::Vector2d best_pos = trajectory.getPos(best_t);
        const Eigen::Vector2d best_vel = trajectory.getVel(best_t);
        const double vel_sq_norm = best_vel.squaredNorm();
        if (vel_sq_norm > 1e-6) {
            const double dt_proj = (current_pos - best_pos).dot(best_vel) / vel_sq_norm;
            best_t = std::clamp(best_t + dt_proj, search_start, search_end);
        }

        projected_time_ = best_t;
        init_ = true;
        return projected_time_;
    }

    mpc_state::ref_sequence MpcSlover::generatePreviewReference(double step, double max_preview_time)
    {
        mpc_state::ref_sequence ref_traj;
        Trajectory2D trajectory;
        if (!getActiveTrajectory(&trajectory) || step <= 0.0 || max_preview_time <= 0.0) {
            return ref_traj;
        }

        const double start_t =
          projectCurrentStateToTrajectory(config_.projection_sample_dt, max_preview_time);
        const double end_t = std::min(trajectory.getEndTime(), start_t + max_preview_time);

        for (double t = start_t; t <= end_t + 1e-9; t += step) {
            ref_traj.push_back(sampleStateFromTrajectory(trajectory, t));
        }

        if (ref_traj.empty() || end_t - start_t < step) {
            ref_traj.push_back(sampleStateFromTrajectory(trajectory, end_t));
        } else {
            const double last_t = start_t + step * static_cast<double>(ref_traj.size() - 1);
            if (end_t - last_t > 1e-6) {
                ref_traj.push_back(sampleStateFromTrajectory(trajectory, end_t));
            }
        }

        return ref_traj;
    }

    void MpcSlover::buildDiscreteModel()
    {
        A = mpc_state::MatrixXd::Identity(kStateDim, kStateDim);
        B = mpc_state::MatrixXd::Zero(kStateDim, kControlDim);

        const double dt = config_.dt;
        const double m = config_.mass;
        const double iz = config_.iz;

        A(0, 3) = dt;
        A(1, 4) = dt;
        A(2, 5) = dt;

        B(3, 0) = dt / m;
        B(4, 1) = dt / m;
        B(5, 2) = dt / iz;
    }

    void MpcSlover::buildPredictionModel()
    {
        prediction_state_ = mpc_state::MatrixXd::Zero(kStateDim * config_.horizon, kStateDim);
        prediction_control_ =
          mpc_state::MatrixXd::Zero(kStateDim * config_.horizon, kControlDim * config_.horizon);

        std::vector<mpc_state::MatrixXd> powers(
          config_.horizon + 1,
          mpc_state::MatrixXd::Identity(kStateDim, kStateDim));
        for (int i = 1; i <= config_.horizon; ++i) {
            powers[i] = powers[i - 1] * A;
        }

        for (int i = 0; i < config_.horizon; ++i) {
            prediction_state_.block(i * kStateDim, 0, kStateDim, kStateDim) = powers[i + 1];
            for (int j = 0; j <= i; ++j) {
                prediction_control_.block(i * kStateDim, j * kControlDim, kStateDim, kControlDim) =
                  powers[i - j] * B;
            }
        }
    }

    void MpcSlover::buildCostMatrices()
    {
        Q = config_.q_diag.asDiagonal();
        R = config_.r_diag.asDiagonal();

        q_bar_ = mpc_state::MatrixXd::Zero(kStateDim * config_.horizon, kStateDim * config_.horizon);
        r_bar_ = mpc_state::MatrixXd::Zero(kControlDim * config_.horizon, kControlDim * config_.horizon);

        for (int i = 0; i < config_.horizon; ++i) {
            q_bar_.block(i * kStateDim, i * kStateDim, kStateDim, kStateDim) = Q;
            r_bar_.block(i * kControlDim, i * kControlDim, kControlDim, kControlDim) = R;
        }

        hessian_ = prediction_control_.transpose() * q_bar_ * prediction_control_ + r_bar_;
        hessian_ = 0.5 * (hessian_ + hessian_.transpose());
    }

    void MpcSlover::buildStaticConstraints()
    {
        const int control_vars = kControlDim * config_.horizon;
        const double inf = qpOASES::INFTY;

        lower_bound_ = Eigen::VectorXd::Constant(control_vars, -inf);
        upper_bound_ = Eigen::VectorXd::Constant(control_vars, inf);

        total_constraints_ = 2 * kControlDim * config_.horizon;
        constraint_matrix_ = mpc_state::MatrixXd::Zero(total_constraints_, control_vars);
        constraint_lower_bound_ = Eigen::VectorXd::Constant(total_constraints_, -inf);
        constraint_upper_bound_ = Eigen::VectorXd::Zero(total_constraints_);
        constraint_state_mapping_ = mpc_state::MatrixXd::Zero(0, kStateDim);

        const Eigen::Matrix<double, 2 * kControlDim, kControlDim> single_step_constraint =
          (Eigen::Matrix<double, 2 * kControlDim, kControlDim>() <<
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
           -1.0, 0.0, 0.0,
            0.0,-1.0, 0.0,
            0.0, 0.0,-1.0).finished();

        const Eigen::Matrix<double, 2 * kControlDim, 1> single_step_upper =
          (Eigen::Matrix<double, 2 * kControlDim, 1>() <<
            config_.u_max(0),
            config_.u_max(1),
            config_.u_max(2),
           -config_.u_min(0),
           -config_.u_min(1),
           -config_.u_min(2)).finished();

        for (int i = 0; i < config_.horizon; ++i) {
            const int row_offset = i * 2 * kControlDim;
            const int col_offset = i * kControlDim;
            constraint_matrix_.block(row_offset, col_offset, 2 * kControlDim, kControlDim) =
              single_step_constraint;
            constraint_upper_bound_.segment(row_offset, 2 * kControlDim) = single_step_upper;
        }
    }

    void MpcSlover::updateConstraintBounds()
    {
        // 当前约束严格按上传公式实现:
        // [I; -I] u_k <= [u_max; -u_min]
        // 因此这些约束与当前状态无关, 不需要在线更新.
    }

    Eigen::VectorXd MpcSlover::flattenReference(const mpc_state::ref_sequence & ref_traj) const
    {
        Eigen::VectorXd ref = Eigen::VectorXd::Zero(kStateDim * config_.horizon);
        if (ref_traj.empty()) {
            return ref;
        }

        for (int i = 0; i < config_.horizon; ++i) {
            const mpc_state::state & state =
              ref_traj[std::min(i, static_cast<int>(ref_traj.size()) - 1)];
            ref.segment(i * kStateDim, kStateDim) = state;
        }
        return ref;
    }

    bool MpcSlover::updateReferenceAndGradient()
    {
        if (!qp_initialized_) {
            return false;
        }

        const auto ref_traj = generatePreviewReference(config_.dt, config_.max_preview_time);
        if (ref_traj.empty()) {
            return false;
        }

        const Eigen::VectorXd ref = flattenReference(ref_traj);
        const Eigen::VectorXd state_error = prediction_state_ * input_state - ref;
        gradient_ = prediction_control_.transpose() * q_bar_ * state_error;
        updateConstraintBounds();
        return true;
    }

    Eigen::VectorXd MpcSlover::clampControlSequence(const Eigen::VectorXd & raw_solution) const
    {
        Eigen::VectorXd clamped = raw_solution;
        for (int i = 0; i < config_.horizon; ++i) {
            for (int j = 0; j < kControlDim; ++j) {
                const int idx = i * kControlDim + j;
                clamped(idx) = std::clamp(clamped(idx), config_.u_min(j), config_.u_max(j));
            }
        }
        return clamped;
    }

    mpc_state::control_sequence MpcSlover::vectorToControlSequence(const Eigen::VectorXd & solution) const
    {
        mpc_state::control_sequence sequence;
        sequence.reserve(config_.horizon);
        for (int i = 0; i < config_.horizon; ++i) {
            sequence.push_back(solution.segment(i * kControlDim, kControlDim));
        }
        return sequence;
    }

    bool MpcSlover::solve(mpc_state::control_sequence * optimal_sequence)
    {
        if (!qp_initialized_ || !updateReferenceAndGradient()) {
            return false;
        }

        if (!qpoases_solver_) {
            return false;
        }

        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = qpOASES::PL_NONE;
        qpoases_solver_->setOptions(options);

        const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> hessian_row_major = hessian_;
        const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> constraint_row_major =
          constraint_matrix_;
        const Eigen::VectorXd gradient_dense = gradient_;
        const Eigen::VectorXd lower_bound_dense = lower_bound_;
        const Eigen::VectorXd upper_bound_dense = upper_bound_;
        const Eigen::VectorXd constraint_lower_dense = constraint_lower_bound_;
        const Eigen::VectorXd constraint_upper_dense = constraint_upper_bound_;

        int n_wsr = std::max(50, config_.horizon * 10);
        qpOASES::returnValue status = qpOASES::SUCCESSFUL_RETURN;

        if (!qpoases_solver_initialized_) {
            status = qpoases_solver_->init(
              hessian_row_major.data(),
              gradient_dense.data(),
              constraint_row_major.data(),
              lower_bound_dense.data(),
              upper_bound_dense.data(),
              constraint_lower_dense.data(),
              constraint_upper_dense.data(),
              n_wsr,
              nullptr);
            qpoases_solver_initialized_ = (status == qpOASES::SUCCESSFUL_RETURN);
        } else {
            status = qpoases_solver_->hotstart(
              gradient_dense.data(),
              lower_bound_dense.data(),
              upper_bound_dense.data(),
              constraint_lower_dense.data(),
              constraint_upper_dense.data(),
              n_wsr,
              nullptr);

            if (status != qpOASES::SUCCESSFUL_RETURN) {
                qpoases_solver_->reset();
                status = qpoases_solver_->init(
                  hessian_row_major.data(),
                  gradient_dense.data(),
                  constraint_row_major.data(),
                  lower_bound_dense.data(),
                  upper_bound_dense.data(),
                  constraint_lower_dense.data(),
                  constraint_upper_dense.data(),
                  n_wsr,
                  nullptr);
                qpoases_solver_initialized_ = (status == qpOASES::SUCCESSFUL_RETURN);
            }
        }

        if (status != qpOASES::SUCCESSFUL_RETURN) {
            qpoases_solver_initialized_ = false;
            return false;
        }

        std::vector<qpOASES::real_t> primal_solution(static_cast<size_t>(solution_.size()), 0.0);
        status = qpoases_solver_->getPrimalSolution(primal_solution.data());
        if (status != qpOASES::SUCCESSFUL_RETURN) {
            qpoases_solver_initialized_ = false;
            return false;
        }

        for (int i = 0; i < solution_.size(); ++i) {
            solution_(i) = primal_solution[static_cast<size_t>(i)];
        }

        solution_ = clampControlSequence(solution_);
        control_state = solution_.segment(0, kControlDim);

        if (optimal_sequence != nullptr) {
            *optimal_sequence = vectorToControlSequence(solution_);
        }
        return true;
    }
}  // namespace mpc_slover
