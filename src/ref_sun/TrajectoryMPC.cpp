#include "TrajectoryMPC.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <fstream>

void TrajectoryMPC::computeRbw()
{
    float psi = _rpy(2);
    float theta = _rpy(1);
    float phi = _rpy(0);

    float r11 = cos(psi) * cos(theta);
    float r12 = cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi);
    float r13 = sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta);
    float r21 = cos(theta) * sin(psi);
    float r22 = cos(phi) * cos(psi) + sin(phi) * sin(psi) * sin(theta);
    float r23 = cos(phi) * sin(psi) * sin(theta) - cos(psi) * sin(phi);
    float r31 = -sin(theta);
    float r32 = cos(theta) * sin(phi);
    float r33 = cos(phi) * cos(theta);

    _Rbw << r11, r12, r13, 
            r21, r22, r23, 
            r31, r32, r33;
}

void TrajectoryMPC::computeJomega()
{
    float psi = _rpy(2);
    float theta = _rpy(1);
    float phi = _rpy(0);

    float jw11 = cos(phi) / (cos(theta) * (cos(phi)*cos(psi)-pow(cos(psi),2)+1));
    float jw12 = sin(psi) / (cos(theta) * (cos(phi)*cos(psi)-pow(cos(psi),2)+1));
    float jw13 = 0;
    float jw21 = -sin(psi) / (cos(phi)*cos(psi) + pow(sin(psi),2));
    float jw22 = cos(psi) / (cos(phi)*cos(psi) + pow(sin(psi),2));
    float jw23 = 0;
    float jw31 = (cos(phi)*sin(theta)) / (cos(theta) * (cos(phi)*cos(psi)-pow(cos(psi),2)+1));
    float jw32 = (sin(psi)*sin(theta)) / (cos(theta) * (cos(phi)*cos(psi)-pow(cos(psi),2)+1));
    float jw33 = 1;

    _Jomega << jw11, jw12, jw13,
               jw21, jw22, jw23, 
               jw31, jw32, jw33;
}

void TrajectoryMPC::getOmegaSS()
{
    _omega_ss << 0, -_omega(2), _omega(1),
                 _omega(2), 0, -_omega(0),
                 -_omega(1), _omega(0), 0;
}

void TrajectoryMPC::continuousModel()
{
    _Ac.resize(13, 13);
    Eigen::Matrix3f zero33, eye3;
    Eigen::MatrixXf zero31(3, 1);
    Eigen::MatrixXf zero13(1, 3);
    zero33.setZero();
    zero31.setZero();
    zero13.setZero();
    eye3.setIdentity();
    Eigen::MatrixXf selectz(3, 1);
    selectz << 0, 0, 1;
    Eigen::Matrix3f Iw_inv;
    _Iworld = _Rbw * _Ibody * _Rbw.transpose();
    Iw_inv = _Iworld.inverse();

    _Ac << zero33, zero33, _Jomega, zero33, zero31,
           zero33, zero33, zero33,  eye3,   zero31,
           zero33, zero33, -Iw_inv*_omega_ss*_Iworld, zero33, zero31,
           zero33, zero33, zero33,  zero33, selectz,
           zero13, zero13, zero13,  zero13, 0;

    _Bc.resize(13, 12);
    Eigen::Matrix3f r1ss, r2ss, r3ss, r4ss;
    r1ss << 0, -_r_GRF(2), _r_GRF(1),
            _r_GRF(2), 0, -_r_GRF(0),
            -_r_GRF(1), _r_GRF(0), 0;
    r2ss << 0, -_r_GRF(5), _r_GRF(4),
            _r_GRF(5), 0, -_r_GRF(3),
            -_r_GRF(4), _r_GRF(3), 0;
    r3ss << 0, -_r_GRF(8), _r_GRF(7),
            _r_GRF(8), 0, -_r_GRF(6),
            -_r_GRF(7), _r_GRF(6), 0;
    r4ss << 0, -_r_GRF(11), _r_GRF(10),
            _r_GRF(11),  0, -_r_GRF(9),
            -_r_GRF(10), _r_GRF(9),  0;

    _Bc << zero33, zero33, zero33, zero33,
           zero33, zero33, zero33, zero33,
           Iw_inv*r1ss, Iw_inv*r2ss, Iw_inv*r3ss, Iw_inv*r4ss,
           eye3/_m, eye3/_m, eye3/_m, eye3/_m,
           zero13, zero13, zero13, zero13;
}

void TrajectoryMPC::discreteModel()
{
    // Not Accurate!!
    // Eigen::MatrixXf eye(13, 13);
    // eye.setIdentity();
    // _Ad.resize(13, 13);
    // _Ad = eye + _Ac * _T;
    // _Bd.resize(13, 12);
    // _Bd = _Bc * _T;

    // Google: discretization - wikipedia
    Eigen::MatrixXf ABc(25, 25);
    ABc.setZero();
    ABc.block(0, 0, 13, 13) = _Ac;                                                                                                                                                                                                                                                 
    ABc.block(0, 13, 13, 12) = _Bc;
    Eigen::MatrixXf ABd(25, 25);
    ABd = (ABc * _T).exp();
    _Ad.resize(13, 13);
    _Ad = ABd.block(0, 0, 13, 13);
    _Bd.resize(13, 12);
    _Bd = ABd.block(0, 13, 13, 12);
}

// Just for this program!
// Only compute positive-integer power of a matrix!
DMat<float> TrajectoryMPC::powerOfMatrix(const DMat<float> matrix, const int power)
{
    Eigen::MatrixXf result(matrix.rows(), matrix.cols());
    result = matrix;
    for (int i = 0; i < power-1; i++)
        result = result * matrix;
    return result;
}

void TrajectoryMPC::batchFormulation()
{
    // _Sx.resize(_horizon*_Ad.rows(), _Ad.cols());
    _Sx.resize(_horizon*13, 13);
    _Su.resize(_horizon*13, _horizon*12);
    Eigen::MatrixXf zero1312(13, 12);
    zero1312.setZero();
    for (int i = 0; i < _horizon; i++)
    {
        // i-th block-row of Sx
        _Sx.block(13*i ,0, 13, 13) = powerOfMatrix(_Ad, i+1);
        
        int power_of_A = i;
        // j-th block-row of Su
        for (int j = 0; j < _horizon; j++)
        {
            if (power_of_A > 0)
                _Su.block(13*i, 12*j, 13, 12) = powerOfMatrix(_Ad, power_of_A) * _Bd;
            else if (power_of_A == 0)
                _Su.block(13*i, 12*j, 13, 12) = _Bd;
            else
                _Su.block(13*i, 12*j, 13, 12) = zero1312;

            power_of_A -= 1;
        }
    }
}

void TrajectoryMPC::updateDynamicalSystem()
{
    TrajectoryMPC::computeRbw();
    TrajectoryMPC::computeJomega();
    TrajectoryMPC::getOmegaSS();
    TrajectoryMPC::continuousModel();
    TrajectoryMPC::discreteModel();
    TrajectoryMPC::batchFormulation();
}

void TrajectoryMPC::showDynamicalSystem() const
{
    std::cout << "=============== Dynamical System ===============" << std::endl;
    std::cout << "Ac = \n" << _Ac << std::endl;
    std::cout << "Bc = \n" << _Bc << std::endl;
    std::cout << "Ad = \n" << _Ad << std::endl;
    std::cout << "Bd = \n" << _Bd << std::endl;
    std::cout << "=============== Batch Formulation ===============" << std::endl;
    std::cout << "Sx = \n" << _Sx << std::endl;
    std::cout << "Su = \n" << _Su << std::endl;
}

TrajectoryMPC::TrajectoryMPC(RobotData &robot_data)
{
    _m = robot_data.m;
    _Ibody << robot_data.Ixx, 0, 0,
              0, robot_data.Iyy, 0,
              0, 0, robot_data.Izz;

    _mu = robot_data.mu;
    _gz = robot_data.gz;
    _T = 0.02;
    _t = 0;
}

TrajectoryMPC::~TrajectoryMPC()
{
    if (_qpSolver != nullptr)
        delete _qpSolver;
}

void TrajectoryMPC::setHorizon(const int horizon)
{
    _horizon = horizon;
}

void TrajectoryMPC::setInitState(RobotState &robot_state)
{
    _rpy = robot_state.rpy;
    _pos = robot_state.pos;
    _omega = robot_state.omega;
    _v = robot_state.v;
    _p_foot << 0.19, -0.049, 0.0, 0.19, 0.049, 0.0, -0.19, 0.049, 0.0, -0.19, -0.049, 0.0;
    _r_GRF << robot_state.pos(0) - _p_foot(0), robot_state.pos(1) - _p_foot(1), robot_state.pos(2) - _p_foot(2),
              robot_state.pos(0) - _p_foot(3), robot_state.pos(1) - _p_foot(4), robot_state.pos(2) - _p_foot(5),
              robot_state.pos(0) - _p_foot(6), robot_state.pos(1) - _p_foot(7), robot_state.pos(2) - _p_foot(8),
              robot_state.pos(0) - _p_foot(9), robot_state.pos(1) - _p_foot(10), robot_state.pos(2) - _p_foot(11);
    _xt.resize(13);
    _xt << _rpy, _pos, _omega, _v, _gz;
    
    std::ofstream fout;
    fout.open("/home/hans/lab/Cheetah-Software-master/user/Pingpong/data/robot_state.sv");
    fout << std::showpoint;
    fout.precision(6);
    fout << _xt(0) << "," << _xt(1) << "," << _xt(2) << "," 
         << _xt(3) << "," << _xt(4) << "," << _xt(5) << "," 
         << _xt(6) << "," << _xt(7) << "," << _xt(8) << "," 
         << _xt(9) << "," << _xt(10) << "," << _xt(11) << "," 
         << _xt(12) << std::endl;
    fout.close();
}

void TrajectoryMPC::updateRobotState(RobotState &robot_state)
{
    _rpy = robot_state.rpy;
    _pos = robot_state.pos;
    _omega = robot_state.omega;
    _v = robot_state.v;
    _r_GRF << robot_state.pos(0) - _p_foot(0), robot_state.pos(1) - _p_foot(1), robot_state.pos(2) - _p_foot(2),
              robot_state.pos(0) - _p_foot(3), robot_state.pos(1) - _p_foot(4), robot_state.pos(2) - _p_foot(5),
              robot_state.pos(0) - _p_foot(6), robot_state.pos(1) - _p_foot(7), robot_state.pos(2) - _p_foot(8),
              robot_state.pos(0) - _p_foot(9), robot_state.pos(1) - _p_foot(10), robot_state.pos(2) - _p_foot(11);
    _xt.resize(13);
    _xt << _rpy, _pos, _omega, _v, _gz;
}

void TrajectoryMPC::setControllerParas(ControllerSettings &controller_setting)
{
    _Q.resize(13, 13);
    _R.resize(12, 12);

    Eigen::Vector3f rpy_weight, pos_weight, omega_weight, v_weight;
    for (int i = 0; i < 3; i++)
    {
        rpy_weight(i) = controller_setting.rpy_weight;
        pos_weight(i) = controller_setting.pos_weight;
        omega_weight(i) = controller_setting.omega_weight;
        v_weight(i) = controller_setting.v_weight;
    }
    Eigen::DiagonalMatrix<float, 3> Qrpy = rpy_weight.asDiagonal();
    Eigen::DiagonalMatrix<float, 3> Qpos = pos_weight.asDiagonal();
    Eigen::DiagonalMatrix<float, 3> Qomega = omega_weight.asDiagonal();
    Eigen::DiagonalMatrix<float, 3> Qv = v_weight.asDiagonal();
    
    Eigen::Matrix3f zero33;
    Eigen::MatrixXf zero31(3, 1);
    Eigen::MatrixXf zero13(1, 3);
    zero33.setZero();
    zero31.setZero();
    zero13.setZero();
    
    _Q << (Eigen::Matrix3f)Qrpy,  zero33,  zero33, zero33, zero31,
          zero33,  (Eigen::Matrix3f)Qpos,  zero33, zero33, zero31,
          zero33, zero33, (Eigen::Matrix3f)Qomega, zero33, zero31,
          zero33, zero33,   zero33,   (Eigen::Matrix3f)Qv, zero31,
          zero13, zero13,   zero13,                zero13,    0.0;

    Eigen::VectorXf f_weight(12);
    for (int i = 0; i < 12; i++)
        f_weight(i) = controller_setting.f_weight;
    Eigen::DiagonalMatrix<float, 12> R = f_weight.asDiagonal();
    _R << (Eigen::MatrixXf)R;
        
    _f_min = controller_setting.f_min;
    _f_max = controller_setting.f_max;
}

void TrajectoryMPC::setDesiredTrajectory(const DMat<float> Xdes)
{
    _Xdes.resize(Xdes.rows(), Xdes.cols());
    _Xdes = Xdes;
}

void TrajectoryMPC::getXdesk()
{
    _Xdesk.resize(13*_horizon, 1);
    // _Xdesk = _Xdes.block(0, _t, 13, _horizon);
    for (int i = 0; i < _horizon; i++)
    {
        _Xdesk.block(13*i, 0, 13, 1) = _Xdes.block(0, _t+i, 13, 1);
    }
}

void TrajectoryMPC::setCostFunction()
{
    _Qbar.resize(13*_horizon, 13*_horizon);
    _Rbar.resize(12*_horizon, 12*_horizon);

    Eigen::MatrixXf zero1313(13, 13);
    Eigen::MatrixXf zero1212(12, 12);
    zero1313.setZero();
    zero1212.setZero();

    for (int i = 0; i < _horizon; i++)
    {
        for (int j = 0; j < _horizon; j++)
        {
            if (i==j)
            {
                _Qbar.block(13*i, 13*i, 13, 13) = _Q;
                _Rbar.block(12*i, 12*i, 12, 12) = _R;
            }
            else
            {
                _Qbar.block(13*i, 13*j, 13, 13) = zero1313;
                _Rbar.block(12*i, 12*j, 12, 12) = zero1212;
            }
        }
    }

    // std::cout << "Qbar = \n" << _Qbar << std::endl;
    // std::cout << "Rbar = \n" << _Rbar << std::endl;

    TrajectoryMPC::getXdesk();
    // std::cout << _Xdesk << std::endl;

    _H.resize(12*_horizon, 12*_horizon);
    _g.resize(12*_horizon, 1);
    _H = 2 * _Su.transpose() * _Qbar * _Su + _Rbar;
    _g = 2*_Su.transpose()*_Qbar*_Sx*_xt - 2*_Su.transpose()*_Qbar*_Xdesk;
}

void TrajectoryMPC::setForceConstraints()
{
    _A.resize(20*_horizon, 12*_horizon);
    _Alb.resize(20*_horizon, 1);
    _Aub.resize(20*_horizon, 1);

    Eigen::MatrixXf zero53(5, 3);
    zero53.setZero();
    Eigen::MatrixXf ci(5, 3);
    ci << 1, 0, _mu,
          1, 0, -_mu,
          0, 1, _mu,
          0, 1, -_mu,
          0, 0, 1;

    const double inf = std::numeric_limits<double>::infinity();
    Eigen::MatrixXf Alb(5, 1);
    Alb << 0, -inf, 0, -inf, _f_min;
    Eigen::MatrixXf Aub(5, 1);
    Aub << inf, 0, inf, 0 , _f_max;

    for (int i = 0; i < 4*_horizon; i++)
    {
        for (int j = 0; j < 4*_horizon; j++)
            _A.block(5*i, 3*j, 5, 3) = (i==j) ? ci : zero53;
        _Alb.block(5*i, 0, 5, 1) = Alb;
        _Aub.block(5*i, 0, 5, 1) = Aub;
    }
}

void TrajectoryMPC::showQPFormulation() const
{
    std::cout << "================ QP Formulation ================" << std::endl;
    std::cout << "The matrix H in cost function is: " << std::endl;
    std::cout << _H << std::endl;
    std::cout << "The matrix g in cost function is: " << std::endl;
    std::cout << _g << std::endl;
    std::cout << "The coeffcient matrix of the force constraints is: " << std::endl;
    std::cout << _A << std::endl; 
    std::cout << "The lower bound of the force constraints is: " << std::endl;
    std::cout << _Alb << std::endl;
    std::cout << "The upper bound of the force constraints is: " << std::endl;
    std::cout << _Aub << std::endl;
}

void TrajectoryMPC::setupSolver()
{
    _qpSolver = new QProblemEigen<float>(_g.size(), _A.rows(), HessianType::HST_POSDEF);
    
    _qpSolver->options.setToMPC();
    // _qpSolver->options.printLevel = PL_DEBUG_ITER;
    // _qpSolver->options.enableRegularisation = BT_TRUE;
    // // _qpSolver->options.enableEqualities = BT_TRUE;
    // _qpSolver->options.epsIterRef = 0.0001;
    // _qpSolver->options.enableRamping = BT_TRUE;
    // _qpSolver->options.enableCholeskyRefactorisation = 1;
    // _qpSolver->options.numRefinementSteps = 100;
    // _qpSolver->setOpt();

    Eigen::MatrixXf lb(0, 1);
    Eigen::MatrixXf ub(0, 1);

    _qpSolver->setup(_H, _g, _A, _Alb, _Aub, lb, ub);
}

bool TrajectoryMPC::getSolution(DVec<float> &solutions)
{
    bool hasSolution = _qpSolver->getSolution(solutions);
    if (hasSolution)
    {
        _cur_control.resize(12, 1);
        for (int i = 0; i < 12; i++)
            _cur_control(i, 0) = solutions(i);
    }
    else
        std::cout << "No solution!" << std::endl;

    return hasSolution;
}

void TrajectoryMPC::update()
{
    // update step
    _t += 1;
    // update state
    TrajectoryMPC::updateDynamicalSystem();
    _xt = _Ad * _xt + _Bd * _cur_control;
    std::cout << "Iteration " << _t-1 << std::endl;
    std::cout << "Current robot state: " 
              << "(rpy) " << _xt(0) << " " << _xt(1) << " " << _xt(2) << " " 
              << "(pos) " << _xt(3) << " " << _xt(4) << " " << _xt(5) << " " 
              << "(omega) " << _xt(6) << " " << _xt(7) << " " << _xt(8) << " " 
              << "(v) " << _xt(9) << " " << _xt(10) << " " << _xt(11) << " "
              << _xt(12) << std::endl;

    std::ofstream fout;
    fout.open("/home/hans/lab/Cheetah-Software-master/user/Pingpong/data/robot_state.csv", std::ios::app);
    fout << std::showpoint;
    fout.precision(6);
    fout << _xt(0) << "," << _xt(1) << "," << _xt(2) << "," 
         << _xt(3) << "," << _xt(4) << "," << _xt(5) << "," 
         << _xt(6) << "," << _xt(7) << "," << _xt(8) << "," 
         << _xt(9) << "," << _xt(10) << "," << _xt(11) << "," 
         << _xt(12) << std::endl;
    fout.close();

    // update cost function
    TrajectoryMPC::setCostFunction();
    // update qp solver
    TrajectoryMPC::setupSolver();
}