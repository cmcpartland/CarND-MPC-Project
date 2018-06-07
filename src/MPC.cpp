#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration. In total, the MPC looks 1 second into the future
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Define the set-point velocity
const double ref_v = 75.0;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lives easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // Implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
	
	fg[0] = 0; // 0th element of fg is reserved for the cost function value
	
	for (unsigned int t = 0; t < N; t++) {
		// Minimize the CTE, error in psi, and delta velocity by adding it to cost
		fg[0] += 4300*CppAD::pow(vars[cte_start+t], 2);
		fg[0] += 4300*CppAD::pow(vars[epsi_start+t], 2);
		fg[0] += CppAD::pow(vars[v_start+t]-ref_v, 2);
	}
	
	for (unsigned int t = 0; t < N-1; t++) {
		// Minimize the use of actuators, i.e. minimize the required steering input and acceleration inputs
		fg[0] += 5*CppAD::pow(vars[delta_start + t], 2);
		fg[0] += 5*CppAD::pow(vars[a_start + t], 2);		
	}
	
	for (unsigned int t = 0; t < N-2; t++) {
		// Minimize the value gap between sequential actuations.
		fg[0] += 1200*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
		fg[0] += 50*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
	}
	
	// SETUP CONSTRAINTS
	// Indexing is bumped by 1 since 0th element holds the cost function value
	fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
	
	// MODEL CONSTRAINTS
	for (unsigned int t = 1; t < N; t++) {
	  AD<double> x_t = vars[x_start + t];
	  AD<double> y_t = vars[y_start + t];
	  AD<double> psi_t = vars[psi_start + t];
	  AD<double> v_t = vars[v_start + t];
	  AD<double> cte_t = vars[cte_start + t];
	  AD<double> epsi_t = vars[epsi_start + t];

      AD<double> x_tm1 = vars[x_start + t - 1];
	  AD<double> y_tm1 = vars[y_start + t -1];
      AD<double> psi_tm1 = vars[psi_start + t - 1];
      AD<double> v_tm1 = vars[v_start + t - 1];
	  AD<double> delta_tm1 = vars[delta_start + t -1];
	  AD<double> epsi_tm1 = vars[epsi_start + t - 1];
	  AD<double> a_tm1 = vars[a_start + t - 1];
	  
	  AD<double> f_tm1 = coeffs[3]*pow(x_tm1,3) + coeffs[2]*pow(x_tm1,2) + coeffs[1]*x_tm1 + coeffs[0];
	  AD<double> psides_tm1 = CppAD::atan(3*coeffs[3]*pow(x_tm1,2) + 2*coeffs[2]*x_tm1 + coeffs[1]);
	  
	  AD<double> c1 = v_tm1*dt;
	  AD<double> c2 = c1*delta_tm1/Lf;
	  
	  fg[1 + x_start + t] = x_t - (x_tm1 + c1*CppAD::cos(psi_tm1));
	  fg[1 + y_start + t] = y_t - (y_tm1 + c1*CppAD::sin(psi_tm1));
	  fg[1 + psi_start + t] = psi_t - (psi_tm1 - c2);
	  fg[1 + v_start + t] = v_t - (v_tm1 + a_tm1*dt);
	  fg[1 + cte_start + t] = cte_t - (f_tm1 - y_tm1 + c1*CppAD::sin(epsi_tm1));
	  fg[1 + epsi_start + t] = epsi_t - (psi_tm1 - psides_tm1 - c2);
	}
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = N*6 + (N-1)*2;
  // Set the number of constraints
  size_t n_constraints = N*6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (unsigned int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  
  // Set lower and upper limits for variables.

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332*Lf;
    vars_upperbound[i] = 0.436332*Lf;
  }
  
  // Acceleration/deceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (unsigned int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
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
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result;
  
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  
  
  // Add all points in controller trajectory to results vector
  for (int t = 1; t < N; t++) {
	result.push_back(solution.x[x_start + t]);
	result.push_back(solution.x[y_start + t]);
  }
	
  return result;
};
