import copy
from submap_utils import *

def fullyAccurateAndCertainObservationModel(states, submap, constants):
    observation = {"states" : states, "submap" : submap}
    return observation

# DEPCRACATED
#def fullyAccurateAndCertainMDPObservationModel(states,  constants):
#    observation = {"states" : states}
#    return observation
