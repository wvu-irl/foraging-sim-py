import sys
from importlib.machinery import SourceFileLoader

sim_params = SourceFileLoader("sim_params", sys.argv[1]).load_module()
print(sim_params.human_description)
