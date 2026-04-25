/*
 * @Author: ei_code_bash && 3080152159@qq.com
 * @Date: 2026-04-19 16:14:51
 * @LastEditors: ei_code_bash && 3080152159@qq.com
 * @LastEditTime: 2026-04-25 22:27:46
 * @FilePath: /ne_nav/src/ne_global_planner/include/ne_global_planner/ne_traj_optimize.hpp
 * @Description: 我永远喜欢雪之下雪乃
 *
 * Copyright (c) 2026 by ei_code_bash, All Rights Reserved.
 */
#ifndef NE_TRAJ_OPTIMIZE
#define NE_TRAJ_OPTIMIZE

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "ne_global_planner/SplineTrajectory.hpp"
#include "ne_global_planner/lbfgs.hpp"
#include "ne_global_planner/ne_grid_search.hpp"
#include "ne_traj_param.hpp"

namespace ne_traj_opt
{

struct TrajectoryParams
{
    double total_len = 10.0;
    double total_time = 10.0;
    double piece_len = 2.0;
    double rho_v = 10000.0;           // Velocity penalty weight
    double rho_collision = 100000.0;  // Collision penalty weight
    double rho_T = 100.0;             // Time penalty weight
    double rho_energy = 500.0;        // Energy (smoothness) penalty weight
    double max_v = 2.0;               // Maximum velocity
    double safe_threshold = 0.5;      // Safety distance threshold

    int int_K = 32;                   // Integration sample points
    int mem_size = 256;               // L-BFGS memory size
    int past = 3;                     // L-BFGS parameter
    double g_epsilon = 1e-6;          // L-BFGS 的收敛阈值
    double min_step = 1e-32;          // Minimum step size
    double delta = 1e-5;              // Function change convergence threshold
    int max_iter = 10000;             // Maximum iterations
};

class TrajectoryOptimize
{
public:
    using Vector2d = Eigen::Vector2d;
    using MatrixXd = Eigen::MatrixXd;
    using VectorXd = Eigen::VectorXd;
    using PPoly2D = SplineTrajectory::PPolyND<2>;
    using CubicSpline2D = SplineTrajectory::CubicSpline2D;

    TrajectoryOptimize(
        std::shared_ptr<esdf_builder::ESDFMap2D> map,
        const std::vector<Vector2d> & jps_path,
        const TrajectoryParams & param = TrajectoryParams())
    : in_opt_(false),
      map_(std::move(map)),
      jps_path_(jps_path),
      param_(param)
    {
        preprocessRawPath();
    }

    struct TrajectoryMetrics
    {
        double max_v = 0.0;  // 最大速度
        double min_clearance = std::numeric_limits<double>::max();  // 轨迹距离障碍物最近的距离
        double total_time = 0.0;  // 总时间
        double trajectory_energy = 0.0;  // 执行轨迹的代价
        double path_deviation = 0.0;  // 相对 JPS 原始路径的平均偏差
    };

    static std::vector<Vector2d> convertJpsGridPath(
        const std::shared_ptr<esdf_builder::ESDFMap2D> & map,
        const std::vector<JPS::StatePtr> & grid_path)
    {
        std::vector<Vector2d> raw_path;
        if (!map) {
            return raw_path;
        }

        raw_path.reserve(grid_path.size());
        for (const auto & node : grid_path) {
            if (!node) {
                continue;
            }

            Vector2d pos;
            map->indexToPos(Eigen::Vector2i(node->x, node->y), pos);
            raw_path.push_back(pos);
        }
        return raw_path;
    }

    void setJpsPath(const std::vector<Vector2d> & jps_path)
    {
        jps_path_ = jps_path;
        preprocessRawPath();
    }

    const std::vector<Vector2d> & getJpsPath() const
    {
        return jps_path_;
    }

    TrajectoryMetrics evaluateTrajectory() const
    {
        TrajectoryMetrics metrics;
        if (!trajectory_.isInitialized()) {
            return metrics;
        }

        metrics.total_time = trajectory_.getDuration();
        const double dt = 0.01;
        double sample_count = 0.0;

        for (double t = 0.0; t < metrics.total_time; t += dt) {
            const Vector2d pos = trajectory_.getPos(t);
            const Vector2d vel = trajectory_.getVel(t);
            const Vector2d acc = trajectory_.getAcc(t);

            metrics.trajectory_energy += acc.squaredNorm() * dt;
            metrics.max_v = std::max(metrics.max_v, vel.norm());
            if (map_) {
                metrics.min_clearance = std::min(metrics.min_clearance, map_->getDistance(pos));
            }

            double min_dist = std::numeric_limits<double>::max();
            for (const auto & jps_pt : jps_path_) {
                min_dist = std::min(min_dist, (pos - jps_pt).norm());
            }
            if (min_dist < std::numeric_limits<double>::max()) {
                metrics.path_deviation += min_dist;
            }
            sample_count += 1.0;
        }

        if (sample_count > 0.0) {
            metrics.path_deviation /= sample_count;
        }
        return metrics;
    }

    std::vector<Vector2d> sampleTrajectory(double dt = 0.1) const
    {
        std::vector<Vector2d> path;
        if (!trajectory_.isInitialized() || dt <= 0.0) {
            return path;
        }

        const double total_time = trajectory_.getDuration();
        for (double t = 0.0; t < total_time; t += dt) {
            path.push_back(trajectory_.evaluate(t, 0));
        }

        const Vector2d end_pos = trajectory_.evaluate(total_time, 0);
        if (path.empty() || (path.back() - end_pos).norm() > 1e-6) {
            path.push_back(end_pos);
        }
        return path;
    }

    bool jps_plan()
    {
        preprocessRawPath();
        if (jps_path_.size() < 2) {
            return false;
        }

        MatrixXd init_state(2, 2);
        MatrixXd end_state(2, 2);
        init_state.setZero();
        end_state.setZero();
        init_state.col(0) = jps_path_.front();
        end_state.col(0) = jps_path_.back();
        init_state.col(1) = safeDirection(jps_path_[1] - jps_path_[0]) * 0.1;
        end_state.col(1) = safeDirection(jps_path_.back() - jps_path_[jps_path_.size() - 2]) * 0.1;

        const double total_len = calTotalPath(jps_path_);
        if (total_len <= 1e-6) {
            return false;
        }

        param_.total_len = total_len;
        int piece_num = static_cast<int>(std::ceil(total_len / std::max(param_.piece_len, 1e-3)));
        piece_num = std::max(piece_num, 2);
        param_.piece_len = total_len / static_cast<double>(piece_num);

        MatrixXd inner_pos = interpolateInnerPoints(piece_num, total_len);
        double total_time = param_.total_time;
        const int optimize_result = optimizeSE2Traj(init_state, inner_pos, end_state, total_time);
        std::cout << "求解的结果为:" << optimize_result << std::endl;
        return optimize_result >= 0;
    }

    int optimizeSE2Traj(
        const MatrixXd & init_state,
        MatrixXd & inner_pos,
        const MatrixXd & end_state,
        double & total_time)
    {
        in_opt_ = true;
        init_pos_ = init_state;
        end_pos_ = end_state;
        piece_pos_ = static_cast<int>(inner_pos.cols()) + 1;

        const int variable_num = 2 * (piece_pos_ - 1) + kDimT;
        VectorXd x(variable_num);
        x.setZero();

        double & tau = x(0);
        Eigen::Map<MatrixXd> Ppos(x.data() + kDimT, 2, piece_pos_ - 1);
        tau = logC2(total_time);
        Ppos = inner_pos;

        VectorXd Tpos(piece_pos_);
        calTfromTauUni(tau, Tpos);
        generateTrajectory(init_state, end_state, Ppos, Tpos);

        const auto metrics = evaluateTrajectory();
        std::cout << "Initial Trajectory:" << std::endl;
        std::cout << "Max velocity: " << metrics.max_v << " m/s" << std::endl;
        std::cout << "Min clearance: " << metrics.min_clearance << " m" << std::endl;
        std::cout << "Path deviation: " << metrics.path_deviation << " m" << std::endl;

        lbfgs::lbfgs_parameter_t lbfgs_params;
        lbfgs_params.mem_size = param_.mem_size;
        lbfgs_params.past = param_.past;
        lbfgs_params.g_epsilon = param_.g_epsilon;
        lbfgs_params.min_step = param_.min_step;
        lbfgs_params.delta = param_.delta;
        lbfgs_params.max_iterations = param_.max_iter;

        double final_cost = 0.0;
        const int result = lbfgs::lbfgs_optimize(
            x,
            final_cost,
            [](void * instance, const VectorXd & x, VectorXd & grad) {
                return static_cast<TrajectoryOptimize *>(instance)->costFunction(x, grad);
            },
            nullptr,
            nullptr,
            this,
            lbfgs_params);

        tau = x(0);
        Ppos = Eigen::Map<MatrixXd>(x.data() + kDimT, 2, piece_pos_ - 1);
        calTfromTauUni(tau, Tpos);
        generateTrajectory(init_pos_, end_pos_, Ppos, Tpos);
        total_time = Tpos.sum();

        std::cout << "Optimization finished with result: " << result << std::endl;
        std::cout << "Final cost: " << final_cost << std::endl;

        in_opt_ = false;
        return result;
    }

    double costFunction(const VectorXd & x, VectorXd & grad)
    {
        grad.setZero();

        const double & tau = x(0);
        double & grad_tau = grad(0);
        Eigen::Map<const MatrixXd> Ppos(x.data() + kDimT, 2, piece_pos_ - 1);
        Eigen::Map<MatrixXd> gradPpos(grad.data() + kDimT, 2, piece_pos_ - 1);

        VectorXd Tpos(piece_pos_);
        calTfromTauUni(tau, Tpos);
        generateTrajectory(init_pos_, end_pos_, Ppos, Tpos);

        double constrain_cost = 0.0;
        MatrixXd gdCpos_constrain;
        VectorXd gdTpos_constrain;
        calculateConstraintCostGrad(trajectory_, constrain_cost, gdCpos_constrain, gdTpos_constrain);

        MatrixXd gradPpos_constrain;
        VectorXd gradTpos_constrain;
        calGradCTtoQT(gdCpos_constrain, gdTpos_constrain, gradPpos_constrain, gradTpos_constrain);

        const double energy = cubic_spline_.getEnergy();
        const double energy_cost = param_.rho_energy * energy;
        const CubicSpline2D::MatrixType gradP_energy = cubic_spline_.getEnergyGradInnerP();
        const VectorXd gradT_energy = cubic_spline_.getEnergyGradTimes();

        gradPpos = gradPpos_constrain + param_.rho_energy * gradP_energy.transpose();
        const VectorXd gradTpos_total = gradTpos_constrain + param_.rho_energy * gradT_energy;

        const double tau_cost = param_.rho_T * expC2(tau);
        const double grad_Tsum = param_.rho_T + gradTpos_total.sum() / static_cast<double>(piece_pos_);
        grad_tau = grad_Tsum * getTtoTauGrad(tau);

        return constrain_cost + energy_cost + tau_cost;
    }

private:
    static constexpr int kDimT = 1;

    void calculateConstraintCostGrad(
        const PPoly2D & traj,
        double & cost,
        MatrixXd & gdCpos,
        VectorXd & gdTpos) const
    {
        cost = 0.0;
        double velocity_cost = 0.0;
        double collision_cost = 0.0;

        const int N = traj.getNumSegments();
        gdCpos = MatrixXd::Zero(4 * N, 2);
        gdTpos = VectorXd::Zero(N);

        const auto & breaks = traj.getBreakpoints();
        const auto & coeffs = traj.getCoefficients();

        for (int i = 0; i < N; ++i) {
            const Eigen::Matrix<double, 4, 2> c = coeffs.block<4, 2>(i * 4, 0);
            const double duration = breaks[i + 1] - breaks[i];
            if (duration <= 1e-9) {
                continue;
            }

            const int K = std::max(param_.int_K, 1);
            const double step = duration / static_cast<double>(K);
            for (int j = 0; j <= K; ++j) {
                const double alpha = static_cast<double>(j) / static_cast<double>(K);
                const double s1 = alpha * duration;
                const double s2 = s1 * s1;
                const double s3 = s2 * s1;
                const double weight = (j == 0 || j == K) ? 0.5 : 1.0;

                Eigen::Matrix<double, 4, 1> beta0;
                Eigen::Matrix<double, 4, 1> beta1;
                Eigen::Matrix<double, 4, 1> beta2;
                beta0 << 1.0, s1, s2, s3;
                beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2;
                beta2 << 0.0, 0.0, 2.0, 6.0 * s1;

                const Vector2d pos = c.transpose() * beta0;
                const Vector2d vel = c.transpose() * beta1;
                const Vector2d acc = c.transpose() * beta2;

                Vector2d grad_p = Vector2d::Zero();
                Vector2d grad_v = Vector2d::Zero();
                double grad_time = 0.0;

                const double vxy_snorm = vel.squaredNorm();
                const double v_viola = vxy_snorm - param_.max_v * param_.max_v;
                if (v_viola > 0.0) {
                    const double cost_v = param_.rho_v * v_viola * v_viola * v_viola;
                    grad_v += param_.rho_v * 6.0 * v_viola * v_viola * vel;
                    cost += cost_v * weight * step;
                    velocity_cost += cost_v * weight * step;
                    grad_time += weight * (cost_v / static_cast<double>(K) + step * alpha * grad_v.dot(acc));
                }

                if (map_) {
                    double sdf_value = 0.0;
                    Vector2d grad_sdf = Vector2d::Zero();
                    map_->getDistanceAndGradient(pos, sdf_value, grad_sdf);
                    const double c_viola = param_.safe_threshold - sdf_value;
                    if (c_viola > 0.0 && sdf_value < 5.0) {
                        const double penalty = c_viola * c_viola * c_viola;
                        const Vector2d grad_pc = -3.0 * c_viola * c_viola * grad_sdf;
                        const double cost_c = param_.rho_collision * penalty;
                        const Vector2d grad_pc_scaled = param_.rho_collision * grad_pc;

                        cost += cost_c * weight * step;
                        collision_cost += cost_c * weight * step;
                        grad_time += weight * (cost_c / static_cast<double>(K) + step * alpha * grad_pc_scaled.dot(vel));
                        grad_p += grad_pc_scaled;
                    }
                }

                gdCpos.block<4, 2>(i * 4, 0) +=
                    (beta0 * grad_p.transpose() + beta1 * grad_v.transpose()) * weight * step;
                gdTpos(i) += grad_time;
            }
        }

        if (in_opt_) {
            std::cout << "Cost breakdown - Vel:" << velocity_cost << ", Coll:" << collision_cost << std::endl;
        }
    }

    void calGradCTtoQT(
        const MatrixXd & gdCpos,
        const VectorXd & gdTpos,
        MatrixXd & gradPpos,
        VectorXd & gradTpos_out)
    {
        CubicSpline2D::MatrixType gdC_typed = gdCpos;
        CubicSpline2D::MatrixType gradByPoints;
        VectorXd gradByTimes;

        cubic_spline_.propagateGrad(gdC_typed, gdTpos, gradByPoints, gradByTimes);

        gradPpos = gradByPoints.transpose();
        gradTpos_out = gradByTimes;

        if (in_opt_) {
            std::cout << "Gradient norm - Ppos: " << gradPpos.norm()
                      << ", Tpos: " << gradTpos_out.norm() << std::endl;
        }
    }

    void calTfromTauUni(const double & tau, VectorXd & T) const
    {
        T.setConstant(expC2(tau) / static_cast<double>(T.size()));
    }

    double logC2(const double & T) const
    {
        return T > 1.0 ? (std::sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - std::sqrt(2.0 / T - 1.0));
    }

    double expC2(const double & tau) const
    {
        return tau > 0.0 ? ((0.5 * tau + 1.0) * tau + 1.0) : 1.0 / ((0.5 * tau - 1.0) * tau + 1.0);
    }

    double getTtoTauGrad(const double & tau) const
    {
        if (tau > 0.0) {
            return tau + 1.0;
        }

        const double den_sqrt = (0.5 * tau - 1.0) * tau + 1.0;
        return (1.0 - tau) / (den_sqrt * den_sqrt);
    }

    double calTotalPath(const std::vector<Vector2d> & jps_path) const
    {
        double total_length = 0.0;
        for (std::size_t i = 1; i < jps_path.size(); ++i) {
            total_length += (jps_path[i] - jps_path[i - 1]).norm();
        }
        return total_length;
    }

    MatrixXd interpolateInnerPoints(int piece_num, double total_len) const
    {
        MatrixXd inner_pos(2, piece_num - 1);
        if (piece_num <= 1) {
            return inner_pos;
        }

        const double step_len = total_len / static_cast<double>(piece_num);
        std::size_t current_segment = 0;
        double accumulated_len = 0.0;

        for (int i = 1; i < piece_num; ++i) {
            const double target_len = static_cast<double>(i) * step_len;
            while (current_segment + 1 < jps_path_.size()) {
                const Vector2d seg_vec = jps_path_[current_segment + 1] - jps_path_[current_segment];
                const double seg_len = seg_vec.norm();
                if (seg_len <= 1e-9) {
                    ++current_segment;
                    continue;
                }

                if (accumulated_len + seg_len >= target_len) {
                    const double ratio = (target_len - accumulated_len) / seg_len;
                    inner_pos.col(i - 1) = jps_path_[current_segment] + ratio * seg_vec;
                    break;
                }

                accumulated_len += seg_len;
                ++current_segment;
            }

            if (current_segment + 1 >= jps_path_.size()) {
                inner_pos.col(i - 1) = jps_path_.back();
            }
        }
        return inner_pos;
    }

    void generateTrajectory(
        const MatrixXd & init_pos,
        const MatrixXd & end_pos,
        const MatrixXd & inner_pos,
        const VectorXd & Tpos)
    {
        std::vector<double> times;
        times.reserve(static_cast<std::size_t>(Tpos.size()) + 1);
        double t = 0.0;
        times.push_back(t);
        for (int i = 0; i < Tpos.size(); ++i) {
            t += std::max(Tpos[i], 1e-6);
            times.push_back(t);
        }

        SplineTrajectory::SplineVector2D waypoints;
        waypoints.reserve(static_cast<std::size_t>(inner_pos.cols()) + 2);
        waypoints.push_back(init_pos.col(0));
        for (int i = 0; i < inner_pos.cols(); ++i) {
            waypoints.push_back(inner_pos.col(i));
        }
        waypoints.push_back(end_pos.col(0));

        SplineTrajectory::BoundaryConditions<2> bc(init_pos.col(1), end_pos.col(1));
        cubic_spline_.update(times, waypoints, bc);
        trajectory_ = cubic_spline_.getTrajectory();
    }

    void preprocessRawPath()
    {
        std::vector<Vector2d> filtered_path;
        filtered_path.reserve(jps_path_.size());
        for (const auto & pt : jps_path_) {
            if (filtered_path.empty() || (pt - filtered_path.back()).norm() > 1e-6) {
                filtered_path.push_back(pt);
            }
        }

        jps_path_.swap(filtered_path);
        const double raw_len = calTotalPath(jps_path_);
        if (raw_len > 1e-6) {
            param_.total_len = raw_len;
        }
    }

    Vector2d safeDirection(const Vector2d & vec) const
    {
        const double norm = vec.norm();
        if (norm <= 1e-9) {
            return Vector2d::Zero();
        }
        return vec / norm;
    }

    bool in_opt_{false};
    std::shared_ptr<esdf_builder::ESDFMap2D> map_;
    std::vector<Vector2d> jps_path_;
    MatrixXd init_pos_;
    MatrixXd end_pos_;
    int piece_pos_{0};
    PPoly2D trajectory_;
    TrajectoryParams param_;
    CubicSpline2D cubic_spline_;
};

}  // namespace ne_traj_opt

#endif
