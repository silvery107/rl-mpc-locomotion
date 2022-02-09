# QP Solver Installation

**Content**:
- OSQP
- OSQP Eigen
- CVXOPT
- MOSEK

## Dependencies
- Eigen3
- CMake

## [OSQP](https://github.com/osqp/osqp)
- Python

```bash
conda install -c conda-forge osqp
```
- C++

```bash
git clone --recursive https://github.com/osqp/osqp

cd osqp
mkdir build &&
cd build

cmake -G "Unix Makefiles" .. # Linux
cmake --build . --target install # build to /usr/local/include and /usr/local/lib
```

## [OSQP Eigen](https://github.com/robotology/osqp-eigen)

```bash
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen/
mkdir build && cd build
cmake ..
make -j 4
sudo make install # default to /usr/local/include  and /usr/local/lib
```

## CVXOPT

- Python
<!-- TODO -->

## MOSEK
- Licenses
- Python
- C++
<!-- TODO -->