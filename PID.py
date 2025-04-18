"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-oneshot-labs

File Name: template.py << [Modify with your own file name!]

Title: [PLACEHOLDER] << [Modify with your own title]

Author: [PLACEHOLDER] << [Write your name or team name here]

Purpose: [PLACEHOLDER] << [Write the purpose of the script here]

Expected Outcome: [PLACEHOLDER] << [Write what you expect will happen when you run
the script.]
"""

########################################################################################
# Imports
########################################################################################

import sys
import math

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(0, '../library')
import racecar_core
import simulation.lidar_sim as ldr

########################################################################################
# Global variables
########################################################################################

rc = racecar_core.create_racecar()

#The sections of the lidar's measurements
#sector 6 is the left side and sector 2 is the right side
sections = {
    "s1": (45, 135),
    "s2": (135, 225),
    "s3": (225, 315),
    "s4": (315, 405),
    "s5": (405, 495),
    "s6": (495, 585),
    "s7": (585, 675),
    "s8": (-45, 45)
}
leftSide = sections["s6"]
rightSide = sections["s2"]

lidarSim = ldr.LidarSim(rc)

########################################################################################
# Functions
########################################################################################

def getMedian(lst)-> float:
    sortedLst = sorted(lst)
    high = math.ceil(len(lst)/2)
    low = math.floor(len(lst)/2)
    return (sortedLst[high] + sortedLst[low])/2

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    rc.drive.stop()    


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    pass


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    samples = lidarSim.get_samples()
    medianLeft = getMedian(samples[leftSide[0]:leftSide[1]])
    medianRight = getMedian(samples[rightSide[0]:rightSide[1]])
    print("Left distance: " + str(medianLeft))
    print("Right distance: " + str(medianRight))

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
