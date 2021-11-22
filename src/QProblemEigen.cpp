#include "QProblemEigen.h"
#include <iostream>

template <typename T>
QProblemEigen<T>::QProblemEigen(int nVars, int nConstrs, HessianType hst_type) : _nVars(nVars), _nConstrs(nConstrs)
{
    assert(nVars > 0 && nConstrs >= 0);
    _QProblem = new QProblem(nVars, nConstrs, hst_type);
    qp_H = new qpOASES::real_t[nVars * nVars];
    qp_g = new qpOASES::real_t[nVars * 1];
    qp_A = new qpOASES::real_t[nConstrs * nVars];
    qp_Aub = new qpOASES::real_t[nConstrs * 1];
    qp_Alb = new qpOASES::real_t[nConstrs * 1];
    qp_ub = new qpOASES::real_t[nVars * 1];
    qp_lb = new qpOASES::real_t[nVars * 1];

    options.printLevel = PrintLevel::PL_NONE;
    _QProblem->setOptions(options);
}

template <typename T>
bool QProblemEigen<T>::setup(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &H,
                             const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &g,
                             const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &A,
                             const Eigen::Matrix<T, Eigen::Dynamic, 1> &Alb,
                             const Eigen::Matrix<T, Eigen::Dynamic, 1> &Aub,
                             const Eigen::Matrix<T, Eigen::Dynamic, 1> &lb,
                             const Eigen::Matrix<T, Eigen::Dynamic, 1> &ub,
                             bool isWarmStart)
{
    assert(H.cols() == _nVars && H.rows() == _nVars);
    MatToOASESArray(H, qp_H);

    assert(g.cols() == 1 && g.rows() == _nVars);
    MatToOASESArray(g, qp_g);

    if (A.size() == 0)
    {
        delete[] qp_A;
        qp_A = NULL;
    }
    else
    {
        assert(A.cols() == _nVars && A.rows() == _nConstrs);
        MatToOASESArray(A, qp_A);
    }

    if (Alb.size() == 0)
    {
        delete[] qp_Alb;
        qp_Alb = NULL;
    }
    else
    {
        assert(Alb.cols() == 1 && Alb.rows() == _nConstrs);
        MatToOASESArray(Alb, qp_Alb);
    }

    if (Aub.size() == 0)
    {
        delete[] qp_Aub;
        qp_Aub = NULL;
    }
    else
    {
        assert(Aub.cols() == 1 && Aub.rows() == _nConstrs);
        MatToOASESArray(Aub, qp_Aub);
    }

    if (lb.size() == 0)
    {
        delete[] qp_lb;
        qp_lb = NULL;
    }
    else
    {
        assert(lb.cols() == 1 && lb.rows() == _nVars);
        MatToOASESArray(lb, qp_lb);
    }

    if (ub.size() == 0)
    {
        delete qp_ub;
        qp_ub = NULL;
    }
    else
    {
        assert(ub.cols() == 1 && ub.rows() == _nVars);
        MatToOASESArray(ub, qp_ub);
    }

    // setOpt();

    real_t *_old_opt_solution = new real_t[_nVars];
    if (_QProblem->isSolved() && isWarmStart)
    {
        _QProblem->getPrimalSolution(_old_opt_solution);
        returnValue retVar = _QProblem->init(qp_H, qp_g, qp_A, qp_lb, qp_ub, qp_Alb, qp_Aub, nWSR, 0, _old_opt_solution);
        if (retVar != SUCCESSFUL_RETURN)
        {
            THROWERROR(retVar);
            return false;
        }
    }
    else
    {
        returnValue retVar = _QProblem->init(qp_H, qp_g, qp_A, qp_lb, qp_ub, qp_Alb, qp_Aub, nWSR, 0);
        if (retVar != SUCCESSFUL_RETURN)
        {
            THROWERROR(retVar);
            return false;
        }
    }
    return true;
}

template <typename T>
void QProblemEigen<T>::setConstant(qpOASES::real_t *array, int size, qpOASES::real_t num)
{
    for (int i = 0; i < size; i++)
    {
        array[i] = (qpOASES::real_t)num;
    }
}

template <typename T>
void QProblemEigen<T>::MatToOASESArray(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &mat, qpOASES::real_t *array)
{
    for (int row = 0; row < mat.rows(); row++)
    {
        for (int col = 0; col < mat.cols(); col++)
        {
            array[row * mat.cols() + col] = (qpOASES::real_t)mat(row, col);
        }
    }
}

template <typename T>
bool QProblemEigen<T>::getSolution(Eigen::Matrix<T, Eigen::Dynamic, 1> &opt_solution)
{
    if (_QProblem->isSolved() == BT_TRUE)
    {
        real_t *_opt_solution = new real_t[_nVars];
        _QProblem->getPrimalSolution(_opt_solution);

        opt_solution.resize(_nVars, 1);
        for (int i = 0; i < _nVars; i++)
        {
            opt_solution(i) = (float)_opt_solution[i];
        }
        return true;
    }
    else
    {
        // throw std::runtime_error("QP got no solution");
        return false;
    }
    // real_t *_opt_solution = new real_t[_nVars];
    // _QProblem->getPrimalSolution(_opt_solution);

    // opt_solution.resize(_nVars, 1);
    // for (int i = 0; i < _nVars; i++)
    // {
    //     opt_solution(i, 0) = (float)_opt_solution[i];
    // }
    // std::cout << "-----------------------------------------------------\n"
    //           << "opt: \n"
    //           << opt_solution.transpose() << std::endl;
}

template class QProblemEigen<double>;
template class QProblemEigen<float>;
