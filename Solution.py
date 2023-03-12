import numpy as np
import scipy.optimize as opt
from Constants import *
from ComputeTransitionProbabilities import P_nonzero

def Solution(P, G, K, TERMINAL_STATE_INDEX):

    """
    Solve the stochastic shortest path problem by Value Iteration, Policy iteration or Linear programming
    
    Computes the optimal cost and
    the optimal control input for each state of the state space.
    @param P: A (K x K x L)-matrix containing the transition probabilities
                          between all states in the state space for all control inputs.
                          The entry P(i, j, l) represents the transition probability
                          from state i to state j if control input l is applied.
    @param G: A (K x L)-matrix containing the stage costs of all states in
                          the state space for all control inputs. The entry G(i, l)
                          represents the cost if we are in state i and apply control
                          input l.
    @param K: An integer representing the total number of states in the state space
    @param TERMINAL_STATE_INDEX: An integer representing the index of the terminal state in the state space

    @return J_opt:
                A (K x 1)-matrix containing the optimal cost-to-go for each
                element of the state space.

    @return u_opt_ind:
                A (K x 1)-matrix containing the index of the optimal control
                input for each element of the state space. Mapping of the
                terminal state is arbitrary (for example: STAY).

    """

    value_func = np.zeros((K,1))
    policy = 4*np.ones((K,1), dtype=int)

    #LINEAR PROGRAM
    c = [-1]*K
    c[TERMINAL_STATE_INDEX] = 0
    bound = [(0,np.inf)] * K
    bound[TERMINAL_STATE_INDEX] = (0,0)
    
    A = []
    b = []
    for i in range(K):
        if i != TERMINAL_STATE_INDEX:
            for action in range(5):
                ub = G[i,action]
                if ub != np.inf:
                    a = [0] * K
                    a[i] = 1
                    b.append(ub)
                    for j in P_nonzero[(i,action)]:
                        a[j] = a[j] - P[i,j,action]
                    A.append(a)    
    res = opt.linprog(c=c, A_ub=A, b_ub=b,bounds = bound,integrality=[2]*K)
    value_func = res.x
    for i in range(K):
        if i != TERMINAL_STATE_INDEX:
            vect = np.zeros((5,1))
            for action in range(5):
                cost = G[i,action]
                sum_prob = 0
                if cost == np.inf:
                    vect[action] = np.inf
                    continue
                for j in P_nonzero[(i,action)]:
                    sum_prob = sum_prob + P[i,j,action]*value_func[j]
                vect[action] = cost+sum_prob
            policy[i] = np.argmin(vect)

    return value_func, policy
