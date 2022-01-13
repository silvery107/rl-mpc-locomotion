#include "TrajectoryMPC.h"
#include <iostream>
#include <cmath>

# define PI 3.1415926

void generateTraj(DMat<float> &Xdes, const int points, const int horizon)
{
    Xdes.resize(13, points+horizon);
    
    // straight line
    // for (int i = 0; i < points; i++)
    // {
    //     Xdes(0, i) = 0;
    //     Xdes(1, i) = 0;
    //     Xdes(2, i) = 0;
    //     Xdes(3, i) = 0.002 * i;
    //     Xdes(4, i) = 0;
    //     Xdes(5, i) = 0.29;
    //     Xdes(6, i) = 0;
    //     Xdes(7, i) = 0;
    //     Xdes(8, i) = 0;
    //     Xdes(9, i) = 0.1;
    //     Xdes(10, i) = 0;
    //     Xdes(11, i) = 0;
    //     Xdes(12, i) = -9.8;
    // }

    // sine trajectory in xz-plane
    // cos trajectory for roll
    for (int i = 0; i < points; i++)
    {
        Xdes(0, i) = 0.15 * cos(20*PI*0.002*i);
        Xdes(1, i) = 0;
        Xdes(2, i) = 0;
        Xdes(3, i) = 0.002 * i;
        Xdes(4, i) = 0;
        Xdes(5, i) = 0.29 + 0.05*sin(20*PI*0.002*i);
        Xdes(6, i) = -0.3*PI*sin(20*PI*0.002*i);
        Xdes(7, i) = 0;
        Xdes(8, i) = 0;
        Xdes(9, i) = 0.1;
        Xdes(10, i) = 0;
        Xdes(11, i) = 0.1*PI*cos(20*PI*0.002*i);
        Xdes(12, i) = -9.8;
    }
    for (int i = points; i < points+horizon; i++)
    {
        Xdes.block(0, i, 13, 1) = Xdes.block(0, points-1, 13, 1);
    }
}

int main()
{
    int horizon = 10;
    Eigen::MatrixXf Xdes;
    generateTraj(Xdes, 50, horizon);
    // std::cout << Xdes << std::endl;

    RobotData data;
    data.m = 9;
    data.Ixx = 0.025;
    data.Iyy = 0.15;
    data.Izz = 0.18;
    data.mu = 0.6;
    data.gz = -9.8;
    data.tau_max = 250;

    TrajectoryMPC test = TrajectoryMPC(data);
    test.setHorizon(horizon);

    RobotState state;
    Eigen::Vector3f rpy, pos, omega, v;
    rpy << 0.21, 0.19, -0.18;
    pos << 0.0, 0.0, 0.29;
    omega << 0.0, 0.0, 0.0;
    v << 0.0, 0.0, 0.0;
    state.rpy = rpy;
    state.pos = pos;
    state.omega = omega;
    state.v = v;
    test.setInitState(state);
    
    ControllerSettings settings;
    settings.rpy_weight = 30.0;
    settings.pos_weight = 50.0;
    settings.omega_weight = 1.0;
    settings.v_weight = 50.0;
    settings.f_weight = 1.0E-6;
    settings.f_max = 150.0;
    settings.f_min = 5.0;
    test.setControllerParas(settings);
    
    test.updateDynamicalSystem();
    // test.showDynamicalSystem();

    test.setDesiredTrajectory(Xdes);
    test.setCostFunction();
    test.setForceConstraints();
    // test.showQPFormulation();

    DVec<float> opt_solution;
    test.setupSolver();
    // test.getSolution(opt_solution);
    // std::cout << opt_solution << std::endl;

    int iterator = 0;
    while (iterator < 50)
    {
        test.getSolution(opt_solution);
        test.update();
        iterator += 1;
    }

    return 0;
}