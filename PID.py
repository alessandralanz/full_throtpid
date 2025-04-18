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
import random
import numpy

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(0, 'library')
import racecar_core
import simulation.lidar_sim as ldr

########################################################################################
# Global variables
########################################################################################

kp = 4
kd = 1

rc = racecar_core.create_racecar()
max_speed = desired_speed = .5

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
leftSide = sections["s7"]
rightSide = sections["s1"]

lastError = 0

lidarSim = ldr.LidarSim(rc)

########################################################################################
# Functions
########################################################################################

def quickselect(l, k, pivot_fn):
    """
    Select the kth element in l (0 based)
    :param l: List of numerics
    :param k: Index
    :param pivot_fn: Function to choose a pivot, defaults to random.choice
    :return: The kth element of l
    """
    if len(l) == 1:
        assert k == 0
        return l[0]

    pivot = pivot_fn(l)

    lows = [el for el in l if el < pivot]
    highs = [el for el in l if el > pivot]
    pivots = [el for el in l if el == pivot]

    if k < len(lows):
        return quickselect(lows, k, pivot_fn)
    elif k < len(lows) + len(pivots):
        # We got lucky and guessed the median
        return pivots[0]
    else:
        return quickselect(highs, k - len(lows) - len(pivots), pivot_fn)

def getMedian(l, pivot_fn=random.choice)-> float:
    if len(l) == 0:
        return 0.0 #if slice ever empties
    if len(l) % 2 == 1:
        return quickselect(l, len(l) // 2, pivot_fn)
    else:
        return 0.5 * (quickselect(l, len(l) / 2 - 1, pivot_fn) +
                      quickselect(l, len(l) / 2, pivot_fn))

#wraps start/end indices and returns that slices as a list
def get_sector_samples(samples, sector):
    start, end = sector
    length = len(samples)
    start = start % length
    end = end % length
    if start < end:
        return samples[start:end]
    else:
        #wraps around the end of the list
        return samples[start:] + samples[:end]
        
def scale(raw_min, raw_max, to_min, to_max, val):
    return (val - raw_min) * (to_max - to_min) / (raw_max - raw_min) + to_min

def pdControl(left, right)-> float:
    global lastError
    normalized_val = (right-left)/(right + left)
    newError = normalized_val - lastError
    lastError = newError
    return normalized_val*kp + newError*kd

#racecar callbacks

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global max_speed
    rc.drive.set_max_speed(max_speed)
    rc.drive.stop()    


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    samples = lidarSim.get_samples_async()
    left_vals = get_sector_samples(samples, leftSide)
    right_vals = get_sector_samples(samples, rightSide)
    
    medianLeft = getMedian(left_vals)
    medianRight = getMedian(right_vals)



    pdCtrl = pdControl(medianLeft, medianRight)
    #angle = max(-1.0, min(1.0, pdCtrl))
    rc.drive.set_speed_angle(max_speed, max(min(pdCtrl, 1),-1))
    


# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
def update_slow():
    samples = lidarSim.get_samples_async()
    #medianLeft = getMedian(samples[leftSide[0]:leftSide[1]])
    #medianRight = getMedian(samples[rightSide[0]:rightSide[1]])
    left_vals  = get_sector_samples(samples, leftSide)
    right_vals = get_sector_samples(samples, rightSide)
    medianLeft = getMedian(left_vals)
    medianRight = getMedian(right_vals)
    print("Left distance: " + str(medianLeft))
    print("Right distance: " + str(medianRight))

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
