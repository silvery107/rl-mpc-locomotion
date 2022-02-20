import numpy as np

class OffsetDurationGait:
    """
    trotting, bounding, pronking
    jumping, galloping, standing
    trotRunning, walking, walking2
    pacing
    """
    def __init__(self, nSegment:int, offset:np.ndarray, durations:np.ndarray, name:str):

        # offset in mpc segments
        self.__offsets = offset.flatten()
        # duration of step in mpc segments
        self.__durations = durations.flatten()
        # offsets in phase (0 to 1)
        self.__offsetsFloat = offset / nSegment
        # durations in phase (0 to 1)
        self.__durationsFloat = durations / nSegment
        self.__nIterations = nSegment
        self.__name = name
        self.__stance = durations[0]
        self.__swing = nSegment - durations[0]
        self.__mpc_table = [0 for _ in range(nSegment*4)]

    def setIterations(self, iterationsPerMPC:int, currentIteration:int):
        self.__iteration = (currentIteration / iterationsPerMPC) % self.__nIterations
        self.__phase = float(currentIteration % (iterationsPerMPC * self.__nIterations)) / float(iterationsPerMPC * self.__nIterations)

    def getContactState(self):
        progress = self.__phase - self.__offsetsFloat

        for i in range(4):
            if progress[i] < 0:
             progress[i] += 1.0

            if progress[i] > self.__durationsFloat[i]:
                progress[i] = 0.0
            else:
                progress[i] = progress[i] / self.__durationsFloat[i]
            
        # print("contact state: %.3f %.3f %.3f %.3f"%(progress[0], progress[1], progress[2], progress[3]))
        return progress[:, None] # convert to matrix

    def getSwingState(self):
        swing_offset = self.__offsetsFloat + self.__durationsFloat
        for i in range(4):
            if swing_offset[i] > 1:
                swing_offset[i] -= 1.0
        swing_duration = np.ones_like(self.__durationsFloat) - self.__durationsFloat

        progress = self.__phase - swing_offset

        for i in range(4):
            if progress[i] < 0:
                progress[i] += 1.0

            if progress[i] > swing_duration[i]:
                progress[i] = 0.0
            else:
                if swing_duration[i] == 0.0:
                    progress[i] = 0.0
                else:
                    progress[i] = progress[i] / swing_duration[i]

        # print("swing state: %.3f %.3f %.3f %.3f"%(progress[0], progress[1], progress[2], progress[3]))
        return progress[:,None]

    def getMpcTable(self):
        # print("MPC table:")
        for i in range(self.__nIterations):
    
            iter = (i + self.__iteration + 1) % self.__nIterations
            progress = iter - self.__offsets
            for j in range(4):
                if progress[j] < 0:
                    progress[j] += self.__nIterations
                if progress[j] < self.__durations[j]:
                    self.__mpc_table[i * 4 + j] = 1
                else:
                    self.__mpc_table[i * 4 + j] = 0
            # print("%d "% self.__mpc_table[i*4 + j], end="")
        
        return self.__mpc_table

    def getCurrentGaitPhase(self):
        return self.__iteration

    def getCurrentSwingTime(self, dtMPC:float, leg:int):
        return dtMPC * self.__swing

    def getCurrentStanceTime(self, dtMPC:float, leg:int):
        return dtMPC * self.__stance