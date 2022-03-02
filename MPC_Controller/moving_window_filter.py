"""Moving window filter to smooth out sensor readings."""

import collections

import numpy as np


class MovingWindowFilter(object):
  """A stable O(1) moving filter for incoming data streams.

  We implement the Neumaier's algorithm to calculate the moving window average,
  which is numerically stable.

  """
  def __init__(self, window_size: int, dim: int = 3):
    """Initializes the class.

    Args:
      window_size: The moving window size.
    """
    assert window_size > 0
    self._window_size = window_size
    self._value_deque = collections.deque(maxlen=window_size)
    # The moving window sum.
    self._sum = np.zeros(dim)
    # The correction term to compensate numerical precision loss during
    # calculation.
    self._correction = np.zeros(dim)

  def _neumaier_sum(self, value: np.ndarray):
    """Update the moving window sum using Neumaier's algorithm.

    For more details please refer to:
    https://en.wikipedia.org/wiki/Kahan_summation_algorithm#Further_enhancements

    Args:
      value: The new value to be added to the window.
    """

    new_sum = self._sum + value
    self._correction = np.where(
        np.abs(self._sum) >= np.abs(value),
        self._correction + (self._sum - new_sum) + value,
        self._correction + (value - new_sum) + self._sum)
    self._sum = new_sum

  def calculate_average(self, new_value: np.ndarray) -> np.ndarray:
    """Computes the moving window average in O(1) time.

    Args:
      new_value: The new value to enter the moving window.

    Returns:
      The average of the values in the window.

    """
    deque_len = len(self._value_deque)
    if deque_len < self._value_deque.maxlen:
      pass
    else:
      # The left most value to be subtracted from the moving sum.
      self._neumaier_sum(-self._value_deque[0])

    self._neumaier_sum(new_value)
    self._value_deque.append(new_value)

    return (self._sum + self._correction) / self._window_size
