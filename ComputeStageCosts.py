import numpy as np
import scipy
from Constants import *

def ComputeStageCosts(stateSpace, map_world, K):
    """
      Computes the stage costs
      for all states in the state space  for all control inputs.

    @type  stateSpace: (K x 4)-matrix
    @param stateSpace: Matrix where the i-th row represents the i-th
          element of the state space.

    @type  map_world: (M x N)-matrix
    @param map_world: A matrix describing the terrain.
              With values: FREE OBSTACLE PORTAL ALIEN MINE
              LAB BASE

    @type  K: integer
    @param K: Index representing the terminal state

    @return G:    A (K x L)-matrix containing the stage costs of all states in
                  the state space for all control inputs. The entry G(i, l)
                  represents the expected stage cost if we are in state i and
                  apply control input l.
    """
    
    G =np.ones((K,Constants.L))


    
    return G
