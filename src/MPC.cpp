#include "MPC.h"
#include "constants.h"
#include "helper.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// The solver takes all the state variables and actuator variables in a single vector.
// Set a few constants to indicate where each variable starts to make code clearer.
  // State
std::size_t x_start     = 0;
std::size_t y_start     = x_start     + N;
std::size_t psi_start   = y_start     + N;
std::size_t v_start     = psi_start   + N;
std::size_t cte_start   = v_start     + N;
std::size_t epsi_start  = cte_start   + N;
  // Control Inputs
std::size_t delta_start = epsi_start  + N;
std::size_t a_start     = delta_start + N - 1;

class FG_eval {
 public:
  // Coefficients of the fitted polynomial and its derivative.
  Eigen::VectorXd coeffs;
  Eigen::VectorXd coeffs_p;
  FG_eval(Eigen::VectorXd coeffs) {
    this->coeffs = coeffs;
    this->coeffs_p = derivative(coeffs);
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    //
    // Setup cost.
    //

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Reference State Cost
    for (std::size_t t = 0; t < N; ++t) {
      // Add CTE and orientation error to the cost. Multiply by a large value
      // to prioritize keeping these values as low as possible.
      AD<double> cte  = vars[cte_start  + t];
      AD<double> epsi = vars[epsi_start + t];
      fg[0] += CppAD::pow(cte, 2);
      fg[0] += CppAD::pow(epsi, 2);
      // Enforce a reference velocity to prevent premature stopping.
      AD<double> v = vars[v_start + t];
      fg[0] += CppAD::pow(v - ref_v, 2);
    }

    // *** Note: The control inputs do not apply for the last time step
    // *** So the loop should terminate at N-1 instead of N.

    // Penalize large steering angle and throttle changes to make the ride smoother.
    for (std::size_t t = 0; t < N - 1; ++t) {
      AD<double> delta = vars[delta_start + t];
      AD<double> a     = vars[a_start     + t];
      fg[0] += CppAD::pow(delta, 2);
      fg[0] += CppAD::pow(a, 2);
    }

    // Add some temporal smoothness by penalizing a large steering angle and
    // throttle change rates. Stop the loop at N - 2 because we're looking
    // ahead one time step get get deltas and because we have just N - 1
    // control inputs.
    for (std::size_t t = 0; t < N - 2; ++t) {
      AD<double> delta0 = vars[delta_start + t];
      AD<double> delta1 = vars[delta_start + t+1];
      AD<double> a0     = vars[a_start     + t];
      AD<double> a1     = vars[a_start     + t+1];
      fg[0] += 350 * CppAD::pow(delta1 - delta0, 2);
        // At higher speeds it is necessary to reduce the tuning multiplier for
        // steering angle, delta, changes because the vehicle must be able to
        // respond quickly enough to make the corners.
      fg[0] += CppAD::pow(a1 - a0, 2);
    }

    //
    // Setup Constraints
    //

    // Initial constraints
    // (We add 1 to each of the starting indices since cost is stored at fg[0].)
    fg[1 + x_start]    = vars[x_start];
    fg[1 + y_start]    = vars[y_start];
    fg[1 + psi_start]  = vars[psi_start];
    fg[1 + v_start]    = vars[v_start];
    fg[1 + cte_start]  = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints - start at t=1 because t=0 is the initial state,
    // not calculated by the solver.
    for (std::size_t t = 1; t < N; ++t) {
      // Prior state values.
      AD<double> x0     = vars[x_start     + t-1];
      AD<double> y0     = vars[y_start     + t-1];
      AD<double> psi0   = vars[psi_start   + t-1];
      AD<double> v0     = vars[v_start     + t-1];
      AD<double> cte0   = vars[cte_start   + t-1];
      AD<double> epsi0  = vars[epsi_start  + t-1];
      // Prior actuator values. Go back two steps to account for actuator latency.
      AD<double> delta0 = vars[delta_start + t-1];
      AD<double> a0     = vars[a_start     + t-1];
      // Current state values.
      AD<double> x1     = vars[x_start    + t];
      AD<double> y1     = vars[y_start    + t];
      AD<double> psi1   = vars[psi_start  + t];
      AD<double> v1     = vars[v_start    + t];
      AD<double> cte1   = vars[cte_start  + t];
      AD<double> epsi1  = vars[epsi_start + t];

      AD<double> f_x0 = polyeval(coeffs, x0);
      AD<double> psi_des0 = CppAD::atan(polyeval(coeffs_p, x0));

      // Model constraints based on MPC state equations.
      fg[1 + x_start    + t] = x1    - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start    + t] = y1    - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start  + t] = psi1  - (psi0 + v0 / Lf * delta0 * dt);
      fg[1 + v_start    + t] = v1    - (v0 + a0 * dt);
      fg[1 + cte_start  + t] = cte1  - ((f_x0 - y0) + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psi_des0) + v0 / Lf * delta0 * dt);
    }
  }
};

//
// MPC class definition implementation.
//

MPC::MPC() { this->delta_limit = deg2rad(25); }
MPC::~MPC() {}

std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Split the state vector into named variables to aid in clarity.
  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).

  // N == # of timesteps
  // N - 1 == # of actuations
  // For MPC we have:
  //  6 model variables (x, y, psi==steering angle, v, cte, epsi)
  //  2 actuators (delta==steering angle change, a==acceleration)

  // Number of independent variables:
  //  one per model variable per time step
  //  one per actuator variable per actuation
  std::size_t n_vars = N * 6 + (N - 1) * 2;
  // We have one constraint per time step for each model variable.
  std::size_t n_constraints = N * 6;

  //
  // Initial value of the independent variables.
  //

  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (std::size_t i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }
  // Set the initial variable values (initial state).
  vars[x_start]    = x;
  vars[y_start]    = y;
  vars[psi_start]  = psi;
  vars[v_start]    = v;
  vars[cte_start]  = cte;
  vars[epsi_start] = epsi;

  //
  // Set lower and upper limits for variables.
  //

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (std::size_t i = 0; i < delta_start; ++i) {
    //vars_lowerbound[i] = std::numeric_limits<double>::min();
    //vars_upperbound[i] = std::numeric_limits<double>::max();
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (std::size_t i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = -this->delta_limit;//-0.436332;
    vars_upperbound[i] = this->delta_limit; //0.436332;
  }

  // Set a limit on acceleration.
  for (std::size_t i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  //
  // Set lower and upper limits for the constraints.
  //

  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (std::size_t i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Set initial state constraints.
  constraints_lowerbound[x_start]    = x;
  constraints_lowerbound[y_start]    = y;
  constraints_lowerbound[psi_start]  = psi;
  constraints_lowerbound[v_start]    = v;
  constraints_lowerbound[cte_start]  = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start]    = x;
  constraints_upperbound[y_start]    = y;
  constraints_upperbound[psi_start]  = psi;
  constraints_upperbound[v_start]    = v;
  constraints_upperbound[cte_start]  = cte;
  constraints_upperbound[epsi_start] = epsi;

  //
  // Setup solver options.
  //

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  //auto cost = solution.obj_value;
  //std::cout << "Cost " << cost << std::endl;

  // Return results.
  std::vector<double> result;

  // Return the calculated steering value and throttle value.
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  // Return predicted points.
  for (std::size_t t = 0; t < N - 1; ++t) {
    result.push_back(solution.x[x_start + t + 1]);
    result.push_back(solution.x[y_start + t + 1]);
  }

  return result;
}
