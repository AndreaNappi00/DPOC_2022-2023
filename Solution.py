import numpy as np
import scipy
from Constants import *

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
    
    #Do you need to do something with the terminal state before solving the problem?

    value_func = np.zeros((K,1))
    policy = 4*np.ones((K,1), dtype=int)
    equal = False

    epsilon = 0.00001
    it = 0

    while not equal and it<500:
        old_value = np.copy(value_func)
        equal = True
        it = it + 1
        #maximum = 0
        print("indx:" + str(it))
        for i in range(K):
            if i != TERMINAL_STATE_INDEX:
                vect = np.zeros((5,1))
                for action in range(5):
                    my_cost = G[i,action]
                    if my_cost == np.inf:
                        vect[action]=np.inf
                        continue
                    for j in Constants.P_nonzero[(i,action)]:
                        vect[action] = vect[action] + P[i,j,action]*value_func[j]
                    vect[action] = vect[action] + my_cost
                policy[i] = np.argmin(vect)
                value_func[i] = vect[policy[i]] 
                diff = np.abs(value_func[i] - old_value[i])
                #maximum = np.max([maximum, diff])
                if diff > epsilon:
                    equal = False

    # mu_star = np.zeros((5,1))
    # P_star = np.zeros((K,K,1))
    # G_star = np.zeros((K,1))
    # for i in range(K):
    #     mu_star[policy[i]] = 1
    #     P_star[i] = np.matmul(P[i],mu_star)
    #     G_star[i] = G[policy[i]]
    # new_value = np.matmul(P_star,old_value) + G_star
    # diff = np.abs(new_value-old_value)
    # indx_tot = 0

    # while not equal:
    #     indx_tot = indx_tot + 1
    #     indx_evaluation = 0
    #     print("index_total:" + str(indx_tot))
    #     while True:         #policy evaluation
    #         indx_evaluation = indx_evaluation + 1
    #         print("indx_evaluate:" +  str(indx_evaluation))
    #         maximum = 0
    #         diff = 0
    #         old_value = np.copy(value_func)
    #         for i in range(K):
    #             current_value = 0
    #             action = policy[i][0]
    #             cost = G[i,action]
    #             for j in range(K):
    #                 prob = P[i,j,action]
    #                 current_value = current_value + prob*old_value[j]
    #                 #print(j)
    #             current_value = current_value + cost
    #             diff = np.abs(current_value - old_value[i])
    #             value_func[i] = current_value
    #         # mu_star = np.zeros((5,1))
    #         # P_star = np.zeros((K,K,1))
    #         # G_star = np.zeros((K,1))
    #         # old_value = np.copy(value_func)
    #         # for i in range(K):
    #         #     mu_star[policy[i]] = 1
    #         #     P_star[i] = np.matmul(P[i],mu_star)
    #         #     G_star[i] = G[i,policy[i]]
    #         # new_value = np.matmul(P_star,old_value) + G_star
    #         # diff = np.max(np.abs(new_value-old_value))
    #             if diff > maximum:
    #                 maximum = diff  
    #         if maximum < epsilon:
    #             break 

    #     old_policy = np.copy(policy)
    #     equal = True

        # for i in range(K):
        #     vect = np.zeros((5,1))
        #     for action in range(5):
        #         my_cost = G[i,action]
        #         if my_cost == np.inf:
        #             vect[action]=np.inf
        #             continue
        #         for j in range(K):
        #             if P[i,j,action] != 0:
        #                 vect[action] = vect[action] + P[i,j,action]*value_func[j]
        #         vect[action] = vect[action] + my_cost
        #     policy[i] = np.argmin(vect)   
        #     if policy[i] != old_policy[i]:
        #         equal = False

    return value_func, policy