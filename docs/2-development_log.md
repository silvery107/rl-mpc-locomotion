# Development Log: Cheetah Software in Python

## How to imitate a C/C++ STRUCT in Python?

For example, we have a `RobotState` struct, and we want to transfer it into a python structure.
```c++
struct RobotState
{
    Vec3<float> pos;
    Vec3<float> vel;
    Vec3<float> omega;
    Mat3<float> R;
    RobotData<float> *data;
};
```

The nature idea is using a `class` in python.
```python
class RobotState:
    pos = np.zeros((3,1), dtype=np.float32)
    vel = np.zeros((3,1), dtype=np.float32)
    omega = np.zeros((3,1), dtype=np.float32)
    R = np.zeros((3,3), dtype=np.float32)
    data = RobotData()
```

In this way, variables in  `RobotState` are all belong to the class, not its instance, which means if we have two instances of `RobotState`, they share the same data. Since we want multiple instances with different data region, we can move atributes of `RobotState` to its `__init__()` function. Problem solved? Not yet.
```python
class RobotState:
    def __init__(self):
        self.pos = np.zeros((3,1), dtype=np.float32)
        self.vel = np.zeros((3,1), dtype=np.float32)
        self.omega = np.zeros((3,1), dtype=np.float32)
        self.R = np.zeros((3,3), dtype=np.float32)
        self.data = RobotData()
```

What about the `data` atribute? It is a pointer in struct but an instance in python class now. We may not want `RobotData()` to be called inside `__init__()` in `RobotState` class, so just declear its type ahead with `:` before `=`.

```python
self.data:RobotData = None
```
Note that this type hints will not initialize `data` but can help IDE auto-complete engine like `data.m -> float`.


## How to copy data from list or ndarray into a Python struct?

Till now we have a `RobotState` class in python looks like a struct, but does it work like a struct? Not yet. As we know, a struct is a user defined data structure and used as a container, but assignment statements in Python do not copy objects, they create bindings between a target and an object. The declearation of member variables like `R = np.zeros((3,3), dtype=np.float32)` will assign them to some initialized data, but how to change their values without replacing their origional data? 

We should use shallow copy or deepcopy operations depending on the situation, and I will take `list` and `np.ndarray` as examples. 

In a non-nested **list**, which means it contains no other objects except numbers can be completely copyed using `copy.copy()`.

```python
original_list = [1, 2, 3, 4]
copied_list = copy.copy(original_list)
```

If a list contains other objects (include other lists) like `[1, 2, [3, 4]]`, 
 `copy.deepcopy()` is needed for deep copying. `copy.deepcopy()` calls `copy.copy()` recursively inside the origional.

```python
original_list = [1, 2, [3, 4]]
copied_list = copy.deepcopy(original_list)
```

In a non-nested **ndarray**, I recommand using `np.copyto()` (numpy >=1.7.1) function for data copying.

```python
np.copyto(dst, src, casting='same_kind')
```

Options for `casting` are `{‘no’, ‘equiv’, ‘safe’, ‘same_kind’, ‘unsafe’}`

- `no` means the data types should not be cast at all.

- `equiv` means only byte-order changes are allowed.

- `safe` means only casts which can preserve values are allowed.

- `same_kind` means only safe casts or casts within a kind, like float64 to float32, are allowed.

- `unsafe` means any data conversions may be done.

## Matrix multiplication and exponential

### Product

|Method|Description|
|-|-|
|`*`|matrix multiplication for `np.mat`; dot product for `np.ndarray`|
|`@`, `np.matmul`| matrix multiplication without scalars|
|`ndarray.dot`, `np.dot`| matrix multiplication|
|`np.multiply`| dot product only|

### Exponential
The matrix exponential of M is defined by
$$
exp(M) = \sum^{\infty}_{k=0}\frac{M^k}{k!}
$$

`scipy.linalg.expm(M)`

Compute the matrix exponential using Pade approximation.





## Quadratic programming with CVXOPT

Convex optimization problems of the form
$$
min \quad \frac{1}{2}x^TPx + q^Tx\\
s.t. \qquad Gx\leq h\\
$$


`cvxopt.solvers.qp(P, Q, G, h, solver="mosek")`


The function `qp` solves the pair of primal and dual convex quadratic programs, which also provides the option of using solver from MOSEK.
