import sys
import subprocess

start = int(sys.argv[1])
end = int(sys.argv[2])

for i in range(start, end+1):
    filename = "foraging" + str(i) + ".py"
    sed_cmd = "s/foraging.*\.npz/foraging" + str(i) + "\.npz/g"
    subprocess.call(["sed", "-i", "-e", sed_cmd, filename])
