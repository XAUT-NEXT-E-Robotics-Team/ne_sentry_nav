#ifndef NE_MPC_CONTROLLER
#define NE_MPC_CONTROLLER

#include <memory>

#include <qpOASES.hpp>

#include "ne_se2_mpc/common.hpp"
#include "ne_global_planner/ne_traj_optimize.hpp"

namespace mpc_slover
{
  struct MpcConfig
  {
    double dt = 0.05;
    int horizon = 40;
    double mass = 20.0;
    double iz = 5.0;
    double gravity = 9.81;
    double max_preview_time = 2.0;
    double projection_sample_dt = 0.02;
    double vx_max = -1.0;
    double vy_max = -1.0;
    double omega_max = -1.0;
    double friction_coeff = -1.0;

    mpc_state::state q_diag = (mpc_state::state() << 50.0, 50.0, 20.0, 5.0, 5.0, 2.0).finished();
    mpc_state::state_control r_diag =
      (mpc_state::state_control() << 0.2, 0.2, 0.1).finished();
    mpc_state::state_control u_min =
      (mpc_state::state_control() << -30.0, -30.0, -10.0).finished();
    mpc_state::state_control u_max =
      (mpc_state::state_control() << 30.0, 30.0, 10.0).finished();
  };

  class MpcSlover
  {
    public:
    using Trajectory2D = ne_traj_opt::TrajectoryOptimize::PPoly2D;

    MpcSlover() = default;
    explicit MpcSlover(const MpcConfig & config);
    ~MpcSlover() = default;

    bool initialize(const MpcConfig & config);
    bool initializeFromYaml(const std::string & yaml_path, const std::string & root_key = "mpc");
    bool state_init();
    void setTrajectoryOptimizer(const std::shared_ptr<ne_traj_opt::TrajectoryOptimize> & optimizer);
    void setReferenceTrajectory(const Trajectory2D & trajectory);
    void setCurrentState(const mpc_state::state & state) { input_state = state; }
    void setControlState(const mpc_state::state_control & control) { control_state = control; }
    double projectCurrentStateToTrajectory(double sample_dt = 0.02, double search_window = 2.0);
    mpc_state::ref_sequence generatePreviewReference(double step, double max_preview_time);
    bool updateReferenceAndGradient();
    bool solve(mpc_state::control_sequence * optimal_sequence = nullptr);

    const mpc_state::MatrixXd & getHessian() const { return hessian_; }
    const Eigen::VectorXd & getGradient() const { return gradient_; }
    const Eigen::VectorXd & getSolution() const { return solution_; }
    const MpcConfig & getConfig() const { return config_; }
    const mpc_state::state_control & getCurrentControl() const { return control_state; }
    double getProjectedTime() const { return projected_time_; }

    private:
    bool getActiveTrajectory(Trajectory2D * trajectory) const;
    mpc_state::state sampleStateFromTrajectory(
      const Trajectory2D & trajectory,
      double t) const;
    void buildDiscreteModel();
    void buildPredictionModel();
    void buildCostMatrices();
    void buildStaticConstraints();
    void updateConstraintBounds();
    Eigen::VectorXd flattenReference(const mpc_state::ref_sequence & ref_traj) const;
    Eigen::VectorXd clampControlSequence(const Eigen::VectorXd & raw_solution) const;
    mpc_state::control_sequence vectorToControlSequence(const Eigen::VectorXd & solution) const;

    MpcConfig config_;
    std::shared_ptr<ne_traj_opt::TrajectoryOptimize> traj_wait_;
    std::shared_ptr<Trajectory2D> direct_traj_;
    mpc_state::state input_state = mpc_state::state::Zero();
    mpc_state::state_control control_state = mpc_state::state_control::Zero();
    bool init_ = false;
    bool qp_initialized_ = false;
    bool qpoases_solver_initialized_ = false;
    double projected_time_ = 0.0;

    mpc_state::MatrixXd A;
    mpc_state::MatrixXd B;
    mpc_state::MatrixXd Q;
    mpc_state::MatrixXd R;
    mpc_state::MatrixXd prediction_state_;
    mpc_state::MatrixXd prediction_control_;
    mpc_state::MatrixXd q_bar_;
    mpc_state::MatrixXd r_bar_;
    mpc_state::MatrixXd hessian_;
    mpc_state::MatrixXd constraint_matrix_;
    mpc_state::MatrixXd constraint_state_mapping_;
    Eigen::VectorXd lower_bound_;
    Eigen::VectorXd upper_bound_;
    Eigen::VectorXd constraint_lower_bound_;
    Eigen::VectorXd constraint_upper_bound_;
    Eigen::VectorXd gradient_;
    Eigen::VectorXd solution_;
    std::unique_ptr<qpOASES::QProblem> qpoases_solver_;
    int total_constraints_ = 0;
  }; 
}
#endif
