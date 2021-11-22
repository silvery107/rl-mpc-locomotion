#ifndef _QPROBLEMEIGEN_H
#define _QPROBLEMEIGEN_H

#include <qpOASES.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

using namespace qpOASES;

template <typename T>
class QProblemEigen
{
public:
    QProblemEigen(int nVars, int nConstrs, HessianType hst_type = HessianType::HST_POSDEF);

    bool setup(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &H,
               const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &g,
               const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &A,
               const Eigen::Matrix<T, Eigen::Dynamic, 1> &Alb,
               const Eigen::Matrix<T, Eigen::Dynamic, 1> &Aub,
               const Eigen::Matrix<T, Eigen::Dynamic, 1> &lb,
               const Eigen::Matrix<T, Eigen::Dynamic, 1> &ub,
               bool isWarmStart = false);

    bool getSolution(Eigen::Matrix<T, Eigen::Dynamic, 1> &opt_solution);

    void setHessianType(HessianType hessianType)
    {
        _QProblem->setHessianType(hessianType);
    }

    void setOpt()
    {
        _QProblem->reset();
        _QProblem->setOptions(options);
        // _QProblem->setPrintLevel(qpOASES::PrintLevel::PL_NONE);
    }

    qpOASES::Options options;

    int_t nWSR = 1000;

private:
    void setConstant(qpOASES::real_t *array, int size, qpOASES::real_t num);

    void MatToOASESArray(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &mat, qpOASES::real_t *array);

    int _nVars, _nConstrs;
    qpOASES::real_t *qp_H, *qp_g, *qp_A, *qp_Aub, *qp_Alb, *qp_ub, *qp_lb;
    qpOASES::QProblem *_QProblem;

    qpOASES::real_t big_num = (qpOASES::real_t)1.0e13;
};

#endif