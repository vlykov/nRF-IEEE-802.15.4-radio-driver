import sys
import os.path

f = open(sys.argv[1], 'r')
f2 = open(os.path.dirname(sys.argv[1])+"/../../framework/doxygen/buildfiles/release_notes/"+os.path.basename(sys.argv[1]), "w")

for line in f:
    if line.find("=====")==0:
        break
    else:
        f2.write(line)

f.close()
f2.close()
