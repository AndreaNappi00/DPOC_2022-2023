import numpy as np
import scipy
from Constants import *
  
def check_if_portal(portal_pos,m1,n1):
      return (m1,n1) in portal_pos
def possible_to_walk_up_upper(m,n, psi):
      if (m+n)%2==0:
            if not psi:
                  return False
            else:
                  return True
      if not psi:
            return True
      return False
def near_alien(m,n,m1,n1,psi):
      num=0
      if not psi: #if we are in the upper world
            return 0
      a = possible_to_walk_up_upper(m1, n1, 1)
      for i in range(len(m)):
            if a:
                  if n1+1 == n[i] and m1 == m[i]:
                        num=num+1
            else:
                  if n1-1 == n[i] and m1 == m[i]:
                        num=num+1
            if m1+1 == m[i] and n1 == n[i]:
                  num=num+1
            if m1-1 == m[i] and n1 == n[i]:
                  num=num+1
            if m1==m[i] and n1==n[i]:
                  num=num+1
      return num
def in_mine(mine_pos, m1, n1, psi1):
      if psi1 == Constants.LOWER:
            return (m1,n1) in mine_pos
      return False
def not_accessible(d,m2,n2):
      # if m2 == Constants.M or m2 == -1 or n2 == Constants.N or n2 == -1:
      #       return True
      # return check_if_obstacle(m,n,m2,n2)
      if not ((m2,n2,0,0) in d.keys()):
            return True
      return False

def ComputeTransitionProbabilities(stateSpace, map_world, K):
    """
    Computes the transition probabilities between all states in the state space for
    all control inputs.
    @type  stateSpace: (K x 4)-matrix
    @param stateSpace: Matrix where the i-th row represents the i-th
          element of the state space.
    @type  map_world: (M x N)-matrix
    @param  map_world:      A matrix describing the terrain.
          With values: FREE OBSTACLE PORTAL ALIEN MINE
          LAB BASE
    @type  K: integer
    @param K: An integer representing the total number of states in the state space
    @return P:
              A (K x K x L)-matrix containing the transition probabilities
              between all states in the state space for all control inputs.
              The entry P(i, j, l) represents the transition probability
              from state i to state j if control input l is applied.
    """
    def stateSpace_to_dict():
      state = tuple(map(tuple,stateSpace))
      ind = np.arange(K)
      dictionary = dict(zip(state,ind))
      return dictionary
    
    m_n_lab = np.where(map_world == Constants.LAB)
    lab_tuple = tuple(zip(m_n_lab[0], m_n_lab[1]))
    m_n_base = np.where(map_world == Constants.BASE)
    base_tuple = tuple(zip(m_n_base[0], m_n_base[1]))
    m_n_portal = np.where(map_world == Constants.PORTAL) 
    portal_tuple = tuple(zip(m_n_portal[0], m_n_portal[1]))
    m_n_mine = np.where(map_world == Constants.MINE)        
    mine_tuple = tuple(zip(m_n_mine[0], m_n_mine[1]))
    state_dict = stateSpace_to_dict()
    m_alien, n_alien = np.where(map_world == Constants.ALIEN)
   # i_terminal = find_state(m_obstacle,n_obstacle, m_lab,n_lab,1,0)
    i_terminal = state_dict[(lab_tuple[0][0],lab_tuple[0][1],1,0)]
    P_DISTURBED = Constants.P_DISTURBED
    P_PROTECTED = Constants.P_PROTECTED
    S = Constants.S
    L = Constants.L
    N = Constants.N
    M = Constants.M
    P = np.zeros((K,K,L))
    P[i_terminal,i_terminal,4] = 1
    Constants.cost_dict[(i_terminal,4)] = 0

    def move_disturbed(m_arriv, n_arriv, psi_arriv, phi_arriv, azione, p_prec, fought):
      Prb = {}
      north = possible_to_walk_up_upper(m_arriv,n_arriv,psi_arriv)
      #j_base = find_state(m_obstacle,n_obstacle,m_base,n_base,0,0)
      j_base = state_dict[(base_tuple[0][0],base_tuple[0][1],0,0)]
      Prb[(i, j_base ,azione)] = 0

      if north:
            if not not_accessible(state_dict, m_arriv, n_arriv+1):    
                  port = check_if_portal(portal_tuple, m_arriv, n_arriv+1)        #1a
                  psi_end = psi_arriv*(1-port) + (1-psi_arriv)*port
                  phi_end = phi_arriv
                  phi_lotta = - 1

                  numb_alien = near_alien(m_alien, n_alien, m_arriv, n_arriv+1, psi_end)     #1b
                  if numb_alien:
                        phi_lotta = phi_end-1
                        if not fought:
                              Constants.cost_dict[(i,azione)] = Constants.cost_dict[(i,azione)] + Constants.N_a*numb_alien*(1-(1-S)*psi_end)*P_DISTURBED/3

                  mine = in_mine(mine_tuple, m_arriv, n_arriv+1, psi_end)          #1c
                  if mine:
                        phi_end = 1
                        phi_lotta = -1

                  j_up = state_dict[(m_arriv,n_arriv+1,phi_end,psi_end)]
                  Prb[(i,j_up,azione)] = ((1-(1-S)*psi_arriv)*P_DISTURBED/3)*((P_PROTECTED**numb_alien)*(1+phi_lotta) - (2+phi_lotta)*phi_lotta)*p_prec
                  if phi_lotta != -1:
                        j_fight = state_dict[(m_arriv,n_arriv+1,phi_end,psi_end)]
                        Prb[(i,j_fight,azione)] = ((1-(1-S)*psi_arriv)*P_DISTURBED/3)*(1-P_PROTECTED**numb_alien)*p_prec
            else:       #nord leads to collision and so to start
                  Prb[(i,j_base,azione)] = Prb[(i,j_base,azione)] + ((1-(1-S)*psi_arriv)*P_DISTURBED/3)*p_prec
                  Constants.cost_dict[(i,azione)] = Constants.cost_dict[(i,azione)] + Constants.N_b*(1-(1-S)*psi_arriv)*P_DISTURBED/3

      if not north:
            if not not_accessible(state_dict, m_arriv, n_arriv-1):    #south
                  port = check_if_portal(portal_tuple, m_arriv, n_arriv-1)        #1a
                  psi_end = psi_arriv*(1-port) + (1-psi_arriv)*port
                  phi_end = phi_arriv
                  phi_lotta = - 1

                  numb_alien = near_alien(m_alien, n_alien, m_arriv, n_arriv-1, psi_end)     #1b
                  if numb_alien:
                        phi_lotta = phi_end-1
                        if not fought:
                              Constants.cost_dict[(i,azione)] = Constants.cost_dict[(i,azione)] + Constants.N_a*numb_alien*(1-(1-S)*psi_end)*P_DISTURBED/3

                  mine = in_mine(mine_tuple, m_arriv, n_arriv-1, psi_end)          #1c
                  if mine:
                        phi_end = 1
                        phi_lotta = -1
                  j_down = state_dict[(m_arriv,n_arriv-1,phi_end,psi_end)]

                  Prb[(i,j_down,azione)] = ((1-(1-S)*psi_arriv)*P_DISTURBED/3)*((P_PROTECTED**numb_alien)*(1+phi_lotta) - (2+phi_lotta)*phi_lotta)*p_prec
                  if phi_lotta != -1:
                        j_fight = state_dict[(m_arriv,n_arriv-1,phi_lotta,psi_end)]
                        Prb[(i,j_fight,azione)] = ((1-(1-S)*psi_arriv)*P_DISTURBED/3)*(1-P_PROTECTED**numb_alien)*p_prec
            else:       #south leads to collision and so to start
                  Prb[(i,j_base,azione)] = Prb[(i,j_base,azione)] + ((1-(1-S)*psi_arriv)*P_DISTURBED/3)*p_prec
                  if not fought:
                        Constants.cost_dict[(i,azione)] = Constants.cost_dict[(i,azione)] + Constants.N_b*(1-(1-S)*psi_arriv)*P_DISTURBED/3
            
      
      if not not_accessible(state_dict, m_arriv+1, n_arriv):    #right
            port = check_if_portal(portal_tuple, m_arriv+1, n_arriv)        #1a
            psi_end = psi_arriv*(1-port) + (1-psi_arriv)*port
            phi_end = phi_arriv
            phi_lotta = - 1

            numb_alien = near_alien(m_alien, n_alien, m_arriv+1, n_arriv, psi_end)     #1b
            if numb_alien:
                  phi_lotta = phi_end-1
                  if not fought:
                        Constants.cost_dict[(i,azione)] = Constants.cost_dict[(i,azione)] + Constants.N_a*numb_alien*(1-(1-S)*psi_end)*P_DISTURBED/3

            mine = in_mine(mine_tuple, m_arriv+1, n_arriv, psi_end)          #1c
            if mine:
                  phi_end = 1
                  phi_lotta = -1
            j_right = state_dict[(m_arriv+1,n_arriv,phi_end,psi_end)]

            Prb[(i,j_right,azione)] = ((1-(1-S)*psi_arriv)*P_DISTURBED/3)*((P_PROTECTED**numb_alien)*(1+phi_lotta) - (2+phi_lotta)*phi_lotta)*p_prec
            if phi_lotta != -1:
                  j_fight = state_dict[(m_arriv+1,n_arriv,phi_lotta,psi_end)]
                  Prb[(i,j_fight,azione)] = ((1-(1-S)*psi_arriv)*P_DISTURBED/3)*(1-P_PROTECTED**numb_alien)*p_prec
      else:       #right leads to collision and so to start
            Prb[(i,j_base,azione)] = Prb[(i,j_base,azione)] + ((1-(1-S)*psi_arriv)*P_DISTURBED/3)*p_prec
            if not fought:
                  Constants.cost_dict[(i,azione)] = Constants.cost_dict[(i,azione)] + Constants.N_b*(1-(1-S)*psi_arriv)*P_DISTURBED/3

      if not not_accessible(state_dict, m_arriv-1, n_arriv):      #left
            port = check_if_portal(portal_tuple, m_arriv-1, n_arriv)        #1a
            psi_end = psi_arriv*(1-port) + (1-psi_arriv)*port
            phi_end = phi_arriv
            phi_lotta = - 1

            numb_alien = near_alien(m_alien, n_alien, m_arriv-1, n_arriv, psi_end)     #1b
            if numb_alien:
                  phi_lotta = phi_end-1
                  if not fought:
                        Constants.cost_dict[(i,azione)] = Constants.cost_dict[(i,azione)] + Constants.N_a*numb_alien*(1-(1-S)*psi_end)*P_DISTURBED/3

            mine = in_mine(mine_tuple, m_arriv-1, n_arriv, psi_end)          #1c
            if mine:
                  phi_end = 1
                  phi_lotta = -1
            j_left = state_dict[(m_arriv-1,n_arriv,phi_end,psi_end)]

            Prb[(i,j_left,azione)] = ((1-(1-S)*psi_arriv)*P_DISTURBED/3)*((P_PROTECTED**numb_alien)*(1+phi_lotta) - (2+phi_lotta)*phi_lotta)*p_prec
            if phi_lotta != -1:
                  j_fight = state_dict[(m_arriv-1,n_arriv,phi_lotta,psi_end)]
                  Prb[(i,j_fight,azione)] = ((1-(1-S)*psi_arriv)*P_DISTURBED/3)*(1-P_PROTECTED**numb_alien)*p_prec
      else:       #left leads to collision and so to start
            Prb[(i,j_base,azione)] = Prb[(i,j_base,azione)] + ((1-(1-S)*psi_arriv)*P_DISTURBED/3)*p_prec
            if not fought:
                  Constants.cost_dict[(i,azione)] = Constants.cost_dict[(i,azione)] + (Constants.N_b*(1-(1-S)*psi_arriv)*P_DISTURBED/3)

      return Prb

    def move_algorithm(m_b,n_b,act,psi_a,phi_a):
      Prob = {}
      phi_b = phi_a
      phi2_lotta = -1

      #1
      portal = check_if_portal(portal_tuple, m_b, n_b)        #1a
      psi_b = psi_a*(1-portal) + (1-psi_a)*portal
      Constants.cost_dict[(i,act)] = (1-(1-S)*psi_b)*P_DISTURBED + 1

      num_alien = near_alien(m_alien, n_alien, m_b, n_b, psi_b)     #1b
      if num_alien:
            phi2_lotta = phi_b-1
            Constants.cost_dict[(i,act)] = Constants.cost_dict[(i,act)] + Constants.N_a*num_alien
      mine = in_mine(mine_tuple, m_b, n_b, psi_b)          #1c
      if mine:
            phi_b = 1
            phi2_lotta = -1

      #2
      if phi2_lotta != -1:
            j_lotta = state_dict[(m_b,n_b,phi2_lotta,psi_b)]
            Prob[(i,j_lotta,act)] = (1-(1-(1-S)*psi_b)*P_DISTURBED)*(1-P_PROTECTED**num_alien)
            Prob_dist_lotta = move_disturbed(m_b, n_b, psi_b, phi2_lotta, act, (1-P_PROTECTED**num_alien), True)
            for key in Prob_dist_lotta.keys():
                  if key not in Prob.keys():
                        Prob[key] = Prob_dist_lotta[key]
                  else:
                        Prob[key] = Prob[key] + Prob_dist_lotta[key]
      j = state_dict[(m_b,n_b,phi_b,psi_b)]
      Prob[(i,j,act)] = (1-(1-(1-S)*psi_b)*P_DISTURBED)*((P_PROTECTED**num_alien)*(1+phi2_lotta) - (2+phi2_lotta)*phi2_lotta)   #prob of fighting or not and not disturbed
      Prob_dist = move_disturbed(m_b, n_b, psi_b, phi_b, act, ((P_PROTECTED**num_alien)*(1+phi2_lotta) - (2+phi2_lotta)*phi2_lotta),False)
      for key in Prob_dist.keys():
            if key not in Prob.keys():
                  Prob[key] = Prob_dist[key]
            else:
                  Prob[key] = Prob[key] + Prob_dist[key]

      return Prob

    for i in range(K):
      m1=stateSpace[i][0]
      n1=stateSpace[i][1]
      phi1=stateSpace[i][2]
      psi1=stateSpace[i][3]
      up = possible_to_walk_up_upper(m1,n1,psi1)

      if i != i_terminal:           # if not in terminal state then execute
            #South
            action = Constants.SOUTH

            if not up and not not_accessible(state_dict, m1, n1-1):
                  m2 = m1
                  n2 = n1-1 
                  Prob_move = move_algorithm(m2,n2,action,psi1,phi1)
                  for key in Prob_move.keys():
                        P[key[0], key[1], key[2]] = P[key[0], key[1], key[2]] + Prob_move[key]

            action = Constants.NORTH
            if up and not not_accessible(state_dict, m1, n1+1):
                  m2 = m1
                  n2 = n1+1
                  Prob_move = move_algorithm(m2,n2,action,psi1,phi1)
                  for key in Prob_move.keys():
                        P[key[0], key[1], key[2]] = P[key[0], key[1], key[2]] + Prob_move[key]

            action = Constants.EAST
            if not not_accessible(state_dict, m1+1, n1):
                  m2 = m1+1
                  n2 = n1
                  Prob_move = move_algorithm(m2,n2,action,psi1,phi1)
                  for key in Prob_move.keys():
                        P[key[0], key[1], key[2]] = P[key[0], key[1], key[2]] + Prob_move[key]

            action = Constants.WEST
            if not not_accessible(state_dict, m1-1, n1):
                  m2 = m1-1
                  n2 = n1
                  Prob_move = move_algorithm(m2,n2,action,psi1,phi1)
                  for key in Prob_move.keys():
                        P[key[0], key[1], key[2]] = P[key[0], key[1], key[2]] + Prob_move[key]
            action = Constants.STAY
            Prob_move =move_algorithm(m1,n1,action,psi1,phi1)
            for key in Prob_move.keys():
                  P[key[0], key[1], key[2]] = P[key[0], key[1], key[2]] + Prob_move[key]

    return P