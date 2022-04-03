from states import *
import copy

def passthroughStateEstimator(self, observation):
    self.states = observation["states"]
    self.submap = observation["submap"]

def passthroughMDPBUMStateEstimator(self, observation):
    states_prime = observation["states"]
    state_prime_index = enumerateState(states_prime, self.state_dimensions)
    for i in range(self.num_models):
        outcomes, outcome_prob = self.T(self.states, self.chosen_action, self.constants, i)
        num_outcomes = len(outcomes)
        outcome_indices = [0] * num_outcomes
        #T_prob = 0.001
        for j in range(num_outcomes):
            outcome_indices[j] = enumerateState(outcomes[j], self.state_dimensions)
            if outcome_indices[j] == state_prime_index:
                T_prob = outcome_prob[j]
                print("matching T_prob: {0}".format(T_prob))
                break
        self.bum[i] *= T_prob
    self.bum /= np.sum(self.bum)
    print("bum: {0}".format(self.bum))
    self.states = copy.deepcopy(states_prime)
    self.submap = observation["submap"]

# DEPRACATED
#def passthroughMDPStateEstimator(self, observation):
#    self.states = observation["states"]
