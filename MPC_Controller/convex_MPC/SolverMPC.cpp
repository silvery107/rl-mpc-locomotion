#include <cmath>
#include <OsqpEigen/OsqpEigen.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

namespace py = pybind11;
using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::Quaternionf;
using std::cout;
using std::endl;

#define BIG_NUMBER 9e7
#define K_MAX_GAIT_SEGMENTS 40
// #define K_PRINT_EVERYTHING 1

struct ProblemSetup
{
    float dt;
    float mu;
    float f_max;
    int horizon;
};

struct UpdateData
{
    Matrix<float, 3, 1> p;
    Matrix<float, 3, 1> v;
    Matrix<float, 3, 1> w;
    Matrix<float, 4, 1> q;
    Matrix<float, 3, 4> r_feet;
    Matrix<float, 12, 1> weights;
    Matrix<float, 12 * K_MAX_GAIT_SEGMENTS, 1> traj;
    Matrix<float, K_MAX_GAIT_SEGMENTS, 1> gait;
    float yaw;
    float alpha;
    float x_drag;
};

Matrix<float, Dynamic, 13> A_qp;
Matrix<float, Dynamic, Dynamic> B_qp;
Matrix<float, 13, 12> Bdt;
Matrix<float, 13, 13> Adt;
Matrix<float, 25, 25> ABc, expmm;
Matrix<float, Dynamic, Dynamic> S;
Matrix<float, Dynamic, 1> X_d;
Matrix<double, Dynamic, 1> l_b;
Matrix<double, Dynamic, 1> U_b;
Matrix<float, Dynamic, Dynamic> fmat;

Matrix<float, Dynamic, Dynamic> qH;
Matrix<double, Dynamic, 1> qg;

Matrix<float, Dynamic, Dynamic> eye_12h;

Matrix<float, 13, 1> x_0;
Matrix<float, 3, 3> I_world;
Matrix<float, 13, 13> A_ct;
Matrix<float, 13, 12> B_ct_r;

// Eigen::VectorXd QPSolution;
Eigen::MatrixXd QPSolution;
UpdateData update;
ProblemSetup setup;
// double *q_soln;

int8_t near_zero(float a)
{
    return (a < 0.01 && a > -.01);
}
int8_t near_one(float a)
{
    return near_zero(a - 1);
}
void quat_to_rpy(Matrix<float, 4, 1> q, Matrix<float, 3, 1> &rpy)
{
    float as = fmin(-2. * (q(1) * q(3) - q(0) * q(2)), .99999);
    rpy(0) =
        atan2(2.f * (q(2) * q(3) + q(0) * q(1)), q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3));
    rpy(1) = asin(as);
    rpy(2) =
        atan2(2.f * (q(1) * q(2) + q(0) * q(3)), q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3));
}
void quat_to_rot(Matrix<float, 4, 1> q, Matrix<float, 3, 3> &R) {
  float e0 = q(0);
  float e1 = q(1);
  float e2 = q(2);
  float e3 = q(3);
  R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
      2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
      1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
      2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
      1 - 2 * (e1 * e1 + e2 * e2);
  R.transposeInPlace();
}
inline Matrix<float, 3, 3> cross_mat(Matrix<float, 3, 3> I_inv, Matrix<float, 3, 1> r)
{
    Matrix<float, 3, 3> cm;
    cm << 0.f, -r(2), r(1), r(2), 0.f, -r(0), -r(1), r(0), 0.f;
    return I_inv * cm;
}
// continuous time state space matrices.
void ct_ss_mats(Matrix<float, 3, 3> I_world, float m, Matrix<float, 3, 4> r_feet, Matrix<float, 3, 3> R_yaw,
                Matrix<float, 13, 13> &A, Matrix<float, 13, 12> &B, float x_drag)
{
    A.setZero();
    A(3, 9) = 1.f;
    A(11, 9) = x_drag;
    A(4, 10) = 1.f;
    A(5, 11) = 1.f;

    A(11, 12) = 1.f;
    A.block(0, 6, 3, 3) = R_yaw.transpose();

    B.setZero();
    Matrix<float, 3, 3> I_inv = I_world.inverse();

    for (int16_t b = 0; b < 4; b++)
    {
        B.block(6, b * 3, 3, 3) = cross_mat(I_inv, r_feet.col(b));
        B.block(9, b * 3, 3, 3) = Matrix<float, 3, 3>::Identity() / m;
    }
}
void resize_qp_mats(int16_t horizon)
{
    A_qp.resize(13 * horizon, Eigen::NoChange);
    B_qp.resize(13 * horizon, 12 * horizon);
    S.resize(13 * horizon, 13 * horizon);
    X_d.resize(13 * horizon, Eigen::NoChange);
    l_b.resize(20 * horizon, Eigen::NoChange);
    U_b.resize(20 * horizon, Eigen::NoChange);
    fmat.resize(20 * horizon, 12 * horizon);
    qH.resize(12 * horizon, 12 * horizon);
    qg.resize(12 * horizon, Eigen::NoChange);
    eye_12h.resize(12 * horizon, 12 * horizon);

    A_qp.setZero();
    B_qp.setZero();
    S.setZero();
    X_d.setZero();
    l_b.setZero();
    U_b.setZero();
    fmat.setZero();
    qH.setZero();
    eye_12h.setIdentity();
}
void c2qp(Matrix<float, 13, 13> Ac, Matrix<float, 13, 12> Bc, float dt, int16_t horizon)
{
    ABc.setZero();
    ABc.block(0, 0, 13, 13) = Ac;
    ABc.block(0, 13, 13, 12) = Bc;
    ABc = dt * ABc;
    expmm = ABc.exp();
    Adt = expmm.block(0, 0, 13, 13);
    Bdt = expmm.block(0, 13, 13, 12);

#ifdef K_PRINT_EVERYTHING
  cout<<"Adt: \n"<<Adt<<"\nBdt:\n"<<Bdt<<endl;
#endif
    if (horizon > 19)
    {
        throw std::runtime_error("horizon is too long!");
    }

    Matrix<float, 13, 13> powerMats[20];
    powerMats[0].setIdentity();
    for (int i = 1; i < horizon + 1; i++)
    {
        powerMats[i] = Adt * powerMats[i - 1];
    }

    for (int16_t r = 0; r < horizon; r++)
    {
        A_qp.block(13 * r, 0, 13, 13) = powerMats[r + 1]; // Adt.pow(r+1);
        for (int16_t c = 0; c < horizon; c++)
        {
            if (r >= c)
            {
                int16_t a_num = r - c;
                B_qp.block(13 * r, 12 * c, 13, 12) = powerMats[a_num] /*Adt.pow(a_num)*/ * Bdt;
            }
        }
    }
#ifdef K_PRINT_EVERYTHING
  cout<<"AQP:\n"<<A_qp<<"\nBQP:\n"<<B_qp<<endl;
#endif
}

int solve_mpc()
{
    // ! RobotState
    Matrix<float, 3, 3> R;
    quat_to_rot(update.q, R);
    Matrix<float, 3, 3> R_yaw;
    float yc = cos(update.yaw);
    float ys = sin(update.yaw);
    R_yaw <<  yc,  -ys,   0,
             ys,  yc,   0,
               0,   0,   1;
    Matrix<float,3,1> Id;
    Id << .07f, 0.26f, 0.242f;
    //Id << 0.3f, 2.1f, 2.1f; // DH
    Matrix<float, 3, 3> I_body;
    I_body.diagonal() = Id;
    float yaw;
    float m = 9;

#ifdef K_PRINT_EVERYTHING

    printf("-----------------\n");
    printf("   PROBLEM DATA  \n");
    printf("-----------------\n");
    print_problem_setup(setup);

    printf("-----------------\n");
    printf("    ROBOT DATA   \n");
    printf("-----------------\n");
    rs.print();
    print_update_data(update, setup.horizon);
#endif

    // roll pitch yaw
    Matrix<float, 3, 1> rpy;
    quat_to_rpy(update.q, rpy);

    // initial state (13 state representation)
    x_0 << rpy(0), rpy(1), rpy(2), update.p, update.w, update.v, -9.8f;
    I_world = R_yaw * I_body * R_yaw.transpose(); // original
    ct_ss_mats(I_world, m, update.r_feet, R_yaw, A_ct, B_ct_r, update.x_drag);

#ifdef K_PRINT_EVERYTHING
    cout << "Initial state: \n" << x_0 << endl;
    cout << "World Inertia: \n" << I_world << endl;
    cout << "A CT: \n" << A_ct << endl;
    cout << "B CT (simplified): \n" << B_ct_r << endl;
#endif
    // QP matrices
    c2qp(A_ct, B_ct_r, setup.dt, setup.horizon);

    // weights
    Matrix<float, 13, 1> full_weight;
    for (uint8_t i = 0; i < 12; i++)
        full_weight(i) = update.weights[i];
    full_weight(12) = 0.f;
    S.diagonal() = full_weight.replicate(setup.horizon, 1);

    // trajectory
    for (int16_t i = 0; i < setup.horizon; i++)
    {
        for (int16_t j = 0; j < 12; j++)
            X_d(13 * i + j, 0) = update.traj[12 * i + j];
    }
    // cout<<"XD:\n"<<X_d<<endl;

    // note - I'm not doing the shifting here.
    int16_t k = 0;
    for (int16_t i = 0; i < setup.horizon; i++)
    {
        for (int16_t j = 0; j < 4; j++)
        {
            U_b(5 * k + 0) = BIG_NUMBER;
            U_b(5 * k + 1) = BIG_NUMBER;
            U_b(5 * k + 2) = BIG_NUMBER;
            U_b(5 * k + 3) = BIG_NUMBER;
            U_b(5 * k + 4) = update.gait[i * 4 + j] * setup.f_max;
            k++;
        }
    }

    float mu = 1.f / setup.mu;
    Matrix<float, 5, 3> f_block;

    f_block << mu, 0, 1.f, -mu, 0, 1.f, 0, mu, 1.f, 0, -mu, 1.f, 0, 0, 1.f;

    for (int16_t i = 0; i < setup.horizon * 4; i++)
    {
        fmat.block(i * 5, i * 3, 5, 3) = f_block;
    }

    qH = 2 * (B_qp.transpose() * S * B_qp + update.alpha * eye_12h);
    qg = (2 * B_qp.transpose() * S * (A_qp * x_0 - X_d)).cast<double>();

    // instantiate the solver
    OsqpEigen::Solver solver;
    // settings
    // solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(12 * setup.horizon);
    solver.data()->setNumberOfConstraints(20 * setup.horizon);
    if(!solver.data()->setHessianMatrix((Eigen::SparseMatrix<double>)qH.sparseView().cast<double>())) return 1;
    if(!solver.data()->setGradient(qg)) return 1;
    if(!solver.data()->setLinearConstraintsMatrix((Eigen::SparseMatrix<double>)fmat.sparseView().cast<double>())) return 1;
    if(!solver.data()->setLowerBound(l_b)) return 1;
    if(!solver.data()->setUpperBound(U_b)) return 1;

    // instantiate the solver
    if(!solver.initSolver()) return 1;

    // solve the QP problem
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;

    // get the controller input
    QPSolution = solver.getSolution();


#ifdef K_PRINT_EVERYTHING
  cout<<"fmat:\n"<<fmat<<endl;
#endif
    return 0;
}
void setup_problem(double dt_, int horizon_, double mu_, double f_max_){
    setup.dt = dt_;
    setup.horizon = horizon_;
    setup.mu = mu_;
    setup.f_max = f_max_;
    resize_qp_mats(horizon_);
}

int update_problem_data(
    Matrix<float,3,1> p,
    Matrix<float, 3, 1> v,
    Matrix<float, 4, 1> q,
    Matrix<float, 3, 1> w,
    Matrix<float, 3, 4> r_feet,
    float yaw,
    Matrix<float, 12, 1> weights,
    Matrix<float, 12 * K_MAX_GAIT_SEGMENTS, 1> traj,
    float alpha,
    Matrix<float, K_MAX_GAIT_SEGMENTS, 1> gait)

{
    update.p = p;
    update.v = v;
    update.w = w;
    update.q = q;
    update.r_feet = r_feet;
    update.weights = weights;
    update.traj = traj;
    update.gait = gait;
    update.yaw = yaw;
    update.alpha = alpha;
    return solve_mpc();
}

void update_x_drag(float x_drag) {
  update.x_drag = x_drag;
}

double get_solution(int index)
{
  return QPSolution(index);
}

PYBIND11_MODULE(C_SolverMPC, m) {
    m.doc() = "pybind11 MPC Solver";
    m.def("setup_problem", &setup_problem,
          py::arg("dt"), 
          py::arg("horizon"), 
          py::arg("mu"), 
          py::arg("fmax")); // resize_qp_mats
    m.def("update_problem_data", &update_problem_data); // solve_mpc
    m.def("update_x_drag", &update_x_drag);
    m.def("get_solution", &get_solution);
}
