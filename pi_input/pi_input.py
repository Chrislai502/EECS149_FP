#!/usr/bin/env python
import time

kobuki_input = 0
f = open("kobuki-input.txt", "w")
f.write(str(kobuki_input))
f.close()
time.sleep(1)

kobuki_input = 150
f = open("kobuki-input.txt", "w")
f.write(str(kobuki_input))
f.close()
time.sleep(3)

kobuki_input = 1
f = open("kobuki-input.txt", "w")
f.write(str(kobuki_input))
f.close()

time.sleep(3)
kobuki_input = -1
f = open("kobuki-input.txt", "w")
f.write(str(kobuki_input))
f.close()
#print("wrote " + str(kobuki_input) + " and closed file")