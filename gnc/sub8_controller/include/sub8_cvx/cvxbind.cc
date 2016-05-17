#include "solver.h"
#include <iostream>
#include <cstring>
#include <vector>

Vars vars;
Params params;
Workspace work;
Settings settings;


struct Dims {
  int rows;
  int columns;
};

void set_A(Params &params, int t, std::vector<double> &elements) {
  double dt = 0.1;
  double B = 0.1;
  // 6x6
  Dims dims;
  dims.rows = 6;
  dims.columns = dims.rows;

  double a[36] {
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
     dt, 0.0, 0.0,  -B, 0.0, 0.0,
    0.0,  dt, 0.0, 0.0,  -B, 0.0,
    0.0, 0.0,  dt, 0.0, 0.0,  -B,
  };

  std::memcpy(params.A[t], a, sizeof(double) * 36);
}

void set_B(Params &params, int t, std::vector<double> &elements) {
  double dt = 0.1;
  Dims dims;
  dims.rows = 6;
  dims.columns = 3;
  // 6x3

  double b[18] = {
    0.0, 0.0, 0.0, 1.0 * dt * dt, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0 * dt * dt, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0 * dt * dt,
  };

  std::memcpy(params.B[t], b, sizeof(double) * 18);
}

//void set_R(Params &params) {
//  double r[3] = {0.001, 0.001, 0.001};
//  std::memcpy(params.R, r, sizeof(double) * 3);
//}

void set_Q_final(Params &params) {
  // TODO: Make diagonal in cost fcn
  double q[6] = {
    1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0
  };
  std::memcpy(params.Q_final, q, sizeof(double) * 36);
}

void compute_solution() {

  set_defaults();
  setup_indexing();
  settings.verbose = 1;

  double x0[6] = {1.0, 0.1, 0.1, 1.1, 0.1, 0.1};
  std::memcpy(params.x_0, x0, sizeof(double) * 6);

  int n = 3;
  int m = 6;
  int T = 17;

  // set_R(params);
  set_Q_final(params);
  std::vector<double> blank;
  for (int t = 0; t <= T; t++) {
    set_B(params, t, blank);
    set_A(params, t, blank);
  }

  params.u_max[0] = 350.0;  // Newtons
  params.xdot_max[0] = 1.2;  // m/s
  // params.S = 1.0;        // Slew

  int num_iters = solve();
  if (work.converged == 1) {
    std::cout << "Converged" << std::endl;
  } else {
    std::cout << "Failed to converge" << std::endl;
  }

  for (int t = 0; t < T + 1; t++) {
    if (t > 0) {
      std::cout << "x: <" << vars.x[t][0] << ", " << vars.x[t][1] << ", "
                << vars.x[t][2] << ">" << std::endl;
      std::cout << "xdot: <" << vars.x[t][3] << ", " << vars.x[t][4] << ", "
                << vars.x[t][5] << ">" << std::endl;
    }
    if (t < T) {
      std::cout << "u: <" << vars.u[t][0] << ", " << vars.u[t][1] << ", "
                << vars.u[t][2] << ">" << std::endl;
    }
    std::cout << t << std::endl;
  }
}

int main() {
  compute_solution();
}
