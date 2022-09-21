import shutil
import sys

start = int(sys.argv[1])
end = int(sys.argv[2])
offset = int(sys.argv[3])

for i in range(start, end+1):
    old_filename = "foraging" + str(i) + ".py"
    new_filename = "foraging" + str(i + offset) + ".py"
    shutil.copyfile(old_filename, new_filename)
