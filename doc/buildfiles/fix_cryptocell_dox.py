import fileinput
import ntpath
import re
for line in fileinput.input():
    line = line.replace(" ::CRYS_RND_SetGenerateVectorFunc"," CRYS_RND_SetGenerateVectorFunc")
    line = line.replace("@param[in/out]","@param[in,out]")
    print(line.rstrip('\n'))

print("/** @} */")
