import os
import pickle
from datetime import datetime

class Logger:
    def __init__(self, logdir="logs/") -> None:
        self._logdir = logdir
        if not os.path.exists(self._logdir):
            os.makedirs(self._logdir)
        self.filename = 'log_{}.pkl'.format(datetime.now().strftime('%Y_%m_%d_%H_%M_%S'))

    def start_logging(self):
        self._logs = []

    def update_logging(self, frame):
        self._logs.append(frame)

    def flush_logging(self):
        with open(os.path.join(self._logdir, self.filename), 'wb') as logfile:
            pickle.dump(self._logs, logfile)

        print("Data logged to: {}".format(os.path.join(self._logdir, self.filename)))
