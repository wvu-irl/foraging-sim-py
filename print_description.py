import sys
from importlib.machinery import SourceFileLoader
import re

num_files = len(sys.argv)
for i in range(1, num_files):
    sim_params = SourceFileLoader("sim_params", sys.argv[i]).load_module()
    filename_string_split = re.split("/|\.", sys.argv[i])
    regex_pattern = re.compile("foraging*")
    matches = list(filter(regex_pattern.match, filename_string_split))
    if len(matches) != 1:
        raise RuntimeError("Filepath does not contain single 'foraging' string")
    output_prefix = matches[0]
    output_string = output_prefix + ": " + sim_params.human_description
    print(output_string)
