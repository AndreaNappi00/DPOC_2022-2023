import modulefinder
import numpy as np
import scipy
from Constants import *




def ComputeTerminalStateIndex(stateSpace, map_world):
    """
    Computes the index of the terminal state in the stateSpace matrix

    @type  stateSpace: (K x 4)-matrix. K is the number of possible combinations of states
    @param stateSpace: Matrix where the i-th row represents the i-th
          element of the state space.

    @type  map_world: (M x N)-matrix
    @param  map_world:      A matrix describing the terrain.
          With values: FREE OBSTACLE PORTAL ALIEN MINE
          LAB BASE

    @return stateIndex: An integer that is the index of the terminal state in the
              stateSpace matrix

    """
    m,n = np.where(map_world == Constants.LAB)

    return np.where((stateSpace == np.array([m,n,1,0])).all(axis=1))[0][0]
