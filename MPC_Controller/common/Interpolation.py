# Utility functions to interpolate between two values
# y0 (3,1)
# yf (3,1)
# x float
import numpy as np

#  Cubic bezier interpolation between y0 and yf.  x is between 0 and 1
def cubicBezier(y0:np.ndarray, yf:np.ndarray, x:float):
    """x must use floating point value"""
    assert x >= 0 and x <= 1
    yDiff = yf - y0
    bezier = x * x * x + 3.0 * (x * x * (1.0 - x))
    return y0 + bezier * yDiff

# Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and 1
def cubicBezierFirstDerivative(y0:np.ndarray, yf:np.ndarray, x:float):
    """x must use floating point value"""
    assert x >= 0 and x <= 1
    yDiff = yf - y0
    bezier = 6.0 * x * (1.0 - x)
    return bezier * yDiff

# Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and 1
def cubicBezierSecondDerivative(y0:np.ndarray, yf:np.ndarray, x:float):
    """x must use floating point value"""
    assert x >= 0 and x <= 1
    yDiff = yf - y0
    bezier = 6.0 - 12.0 * x
    return bezier * yDiff