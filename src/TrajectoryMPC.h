#ifndef TRAJECTORY_MPC_H_
#define TRAJECTORY_MPC_H_

#include "cppTypes.h"
#include "QProblemEigen.h"

struct RobotData
{
    int   m;
    float Ixx;
    float Iyy;
    float Izz;
    float mu;
    float gz;
    float tau_max;
};

struct ControllerSettings
{
    float rpy_weight;
    float pos_weight;
    float omega_weight;
    float v_weight;
    float f_weight;
    float f_min;
    float f_max;
};

struct RobotState
{
    Vec3<float> rpy;
    Vec3<float> pos;
    Vec3<float> omega;
    Vec3<float> v;
};

class TrajectoryMPC
{
private:

    // parameters of controller
    int _horizon;
    int _t;       /* suppose our trajectory has m points,
                    _t will record we are at t-th step
                    now. (0 <= t <= m) */
    float _T;
    float _gz;

    // robot data
    int _m;
    Mat3<float> _Jomega;
    Mat3<float> _Ibody;
    Mat3<float> _omega_ss;
    
    // robot state
    Vec3<float> _rpy;
    Vec3<float> _pos;
    Vec3<float> _omega;
    Vec3<float> _v;
    Vec12<float> _p_foot;
    Vec12<float> _r_GRF;
    DVec<float> _xt;

    Mat3<float> _Iworld;
    RotMat<float> _Rbw;

    // State space modeltty
    DMat<float> _Ac;
    DMat<float> _Bc;
    DMat<float> _Ad;
    DMat<float> _Bd;
    DMat<float> _Sx;
    DMat<float> _Su;

    // Force constraints
    float _mu;
    float _f_min;
    float _f_max;

    // Desired trajectory
    DMat<float> _Xdes;
    DMat<float> _Xdesk;

    // QP formulation
    DMat<float> _Q;
    DMat<float> _R;
    DMat<float> _Qbar;
    DMat<float> _Rbar;
    DMat<float> _H;
    DMat<float> _g;
    DMat<float> _Aub;
    DMat<float> _Alb;
    DMat<float> _A;

    // QP solver
    QProblemEigen<float> *_qpSolver;
    DMat<float> _cur_control;

    void computeRbw();
    void computeJomega();
    void getOmegaSS();
    void continuousModel();
    void discreteModel();
    void batchFormulation();
    void getXdesk();
    DMat<float> powerOfMatrix(const DMat<float> matrix, const int power);

public:

    TrajectoryMPC(RobotData &robot_data);
    ~TrajectoryMPC();
    
    void setHorizon(const int horizon); 
    void setInitState(RobotState &robot_state);
    void updateRobotState(RobotState &robot_state);
    void setControllerParas(ControllerSettings &controller_setting);
    
    void updateDynamicalSystem();
    void showDynamicalSystem() const;
    void setDesiredTrajectory(const DMat<float> Xdes);
    void setCostFunction();  
    void setForceConstraints();
    void showQPFormulation() const;
    
    void setupSolver();
    bool getSolution(DVec<float> &solutions);
    void update();
};

#endif