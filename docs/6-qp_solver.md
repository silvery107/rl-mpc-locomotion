# QP Solver Installation

**Content**:
- [OSQP](https://github.com/osqp/osqp)
- [OSQP Eigen](https://github.com/robotology/osqp-eigen)
- [CVXOPT](https://github.com/cvxopt/cvxopt)
- [MOSEK](https://www.mosek.com/)

## Dependencies
- Eigen3
- CMake

## OSQP
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

## OSQP Eigen

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
  - Installing via conda
    ```bash
    conda install -c conda-forge cvxopt
    ```

  - Installing via pip
    ```bash
    pip install cvxopt
    ```

## MOSEK
### License Activation

> To activate an ordered product:
> 1. Determine the hostname and hostid of your designated machine(s) according to the instructions in the [licensing guide](http://docs.mosek.com/latest/licensing/hostid-hostname.html).
> 2. E-mail those details to license@mosek.com.
> 3. You will receive license file(s) which can be installed according to the instructions for [floating license](http://docs.mosek.com/latest/licensing/floating-license.html) or [server license](http://docs.mosek.com/latest/licensing/client-setup.html).

After activated the MOSEK license, a `mosek.lic` file will be sent to your email. The license file should be placed inside a folder called "mosek" under the user's home directory. For example:

- Windows:
  `c:\users\_userid_\mosek\mosek.lic`

- Linux/Mac OS:
  `/home/_userid_/mosek/mosek.lic`

  Where `_userid_` is your User ID on the computer.

### Software Installation

Download default installers in the [download page](https://www.mosek.com/downloads/) according to your systems.

- Python
  - Installing via conda
    ```bash
    conda install -c mosek mosek
    ```

  - Installing via pip
    ```bash
    pip install Mosek 
    ```
