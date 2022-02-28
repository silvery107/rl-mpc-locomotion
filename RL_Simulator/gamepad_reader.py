from absl import app
from absl import flags
import inputs
from inputs import get_gamepad
import threading
import time

FLAGS = flags.FLAGS
MAX_ABS_VAL = 32768


def _interpolate(raw_reading, max_raw_reading, new_scale):
  return raw_reading / max_raw_reading * new_scale


class Gamepad:
  """Interface for reading commands from xbox Gamepad.

  The control works as following:
  1) Press LB+RB at any time for emergency stop
  2) Use the left joystick for forward/backward/left/right walking.
  3) Use the right joystick for rotation around the z-axis.
  """
  def __init__(self, 
               vel_scale_x: float=.5, 
               vel_scale_y: float=.5, 
               vel_scale_rot: float=1.,
               max_acc: float = .5):
    """Initialize the gamepad controller.
    Args:
      vel_scale_x: maximum absolute x-velocity command.
      vel_scale_y: maximum absolute y-velocity command.
      vel_scale_rot: maximum absolute yaw-dot command.
    """
    if not inputs.devices.gamepads:
      raise Exception("Please plug in your xbox gamepad!")

    self.gamepad = inputs.devices.gamepads[0]
    self._vel_scale_x = vel_scale_x
    self._vel_scale_y = vel_scale_y
    self._vel_scale_rot = vel_scale_rot
    self._max_acc = max_acc
    self._lb_pressed = False
    self._rb_pressed = False
    self._lj_pressed = False

    # Controller states
    self.vx, self.vy, self.wz = 0., 0., 0.
    self._gait_number = 0
    self._FSM_switch = False
    self.estop_flagged = False
    self.is_running = True

    self.read_thread = threading.Thread(target=self.read_loop)
    self.read_thread.start()

  def read_loop(self):
    """The read loop for events.

    This funnction should be executed in a separate thread for continuous
    event recording.
    """
    while self.is_running:# and not self.estop_flagged:
      try:
        events = self.gamepad.read()
        for event in events:
          # print(event.ev_type, event.code, event.state)
          self.update_command(event)
      except Exception as e:
        pass

  def update_command(self, event):
    """Update command based on event readings."""
    if event.ev_type == 'Key' and event.code == 'BTN_TL':
      self._lb_pressed = bool(event.state)
      self._FSM_switch = self._lb_pressed
    elif event.ev_type == 'Key' and event.code == 'BTN_TR':
      self._rb_pressed = bool(event.state)
    elif event.ev_type == 'Key' and event.code == 'BTN_THUMBL':
      self._lj_pressed = bool(event.state)
    elif event.ev_type == 'Absolute' and event.code == 'ABS_HAT0X':
      self._gait_number += event.state
    elif event.ev_type == 'Absolute' and event.code == 'ABS_X':
      # Left Joystick L/R axis
      self.vy = _interpolate(-event.state, MAX_ABS_VAL, self._vel_scale_y)
    elif event.ev_type == 'Absolute' and event.code == 'ABS_Y':
      # Left Joystick F/B axis; need to flip sign for consistency
      self.vx = _interpolate(-event.state, MAX_ABS_VAL, self._vel_scale_x)
    elif event.ev_type == 'Absolute' and event.code == 'ABS_RX':
      self.wz = _interpolate(-event.state, MAX_ABS_VAL, self._vel_scale_rot)
    
    if self.estop_flagged and self._lj_pressed:
      self.estop_flagged = False
      print("Estop Released.")

    if self._lb_pressed and self._rb_pressed:
      print("EStop Flagged, press LEFT joystick to release.")
      self.estop_flagged = True
      self.vx, self.vy, self.wz = 0., 0., 0.

  def get_command(self):
    return (self.vx, self.vy, 0), self.wz, self.estop_flagged

  def get_gait(self):
    if self._gait_number < 0:
      self._gait_number = 0
    return self._gait_number

  def get_FSM_switch(self):
    return self._FSM_switch

  def stop(self):
    self.is_running = False


def main(_):
  gamepad = Gamepad()
  while True:
    # print("Vx: {}, Vy: {}, Wz: {}, Estop: {}".format(gamepad.vx, gamepad.vy,
    #                                                  gamepad.wz,
    #                                                  gamepad.estop_flagged))
    time.sleep(0.1)


if __name__ == "__main__":
  app.run(main)
