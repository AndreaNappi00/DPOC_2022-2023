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
#     for i in range(0,M):
#       for j in range(0,N):
#             if map_world[i][j] == LAB:
#                   m = i
#                   n = j


    m,n = np.where(map_world == Constants.LAB)
    dim = np.size(stateSpace,0)
    for i in range(0,dim):
      if stateSpace[i][0]==m and stateSpace[i][1]==n and stateSpace[i][2]==1 and stateSpace[i][3]==0:
            stateIndex = i
    return stateIndex
