def passthroughStateEstimator(self, observation):
    self.states = observation["states"]
    self.submap = observation["submap"]

def passthroughMDPStateEstimator(self, observation):
    self.states = observation["states"]
