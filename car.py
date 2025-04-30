"""
`MIT BWSI Autonomous RACECAR
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
import numpy as np

# If this file is nested inside a folder in the labs folder, the relative path should
# be [1, ../../library] instead.
sys.path.insert(1, "../../library")
import racecar_core
import racecar_utils as rc_utils

########################################################################################
# Global variables
########################################################################################

#kp = 0.08
kp = 0.02
ki = 0.00001
#kd = 0.037
kd = 0.06

rc = racecar_core.create_racecar(False)
max_speed = 0.2
desired_speed = 0.125

sector_width_degree = 40
half_sw = sector_width_degree / 2
center_left = 90
center_right = 270

rightSide = (270, 300)
leftSide = (60, 90)

lastError = 0.0
integralError = 0.0

########################################################################################
# Functions
########################################################################################
def update_lidar():
    raw = rc.lidar.get_samples()
    if len(raw) == 0:
        return None

    clipped = np.clip(raw, None, 3000)

    scan_length = len(clipped)
    per_degree = (scan_length - 1) / 270.0 #~4 raw samples per degree

    scan360 = []
    for deg in range(360):
        #map output degree to signed angle in [-180, 180)
        raw_angle = ((deg + 180) % 360) - 180

        #if outside the sensor's field of view
        if abs(raw_angle) > 135:
            #padding: how we fill in degrees where the lidar doesn’t measure anything
            #padding value is the placeholder distance we want the control logic
            #to believe exists where the lidar has no real data
            #choosing it large (e.g. 3000) means “treat blind spots as open”
            #choosing it small means “treat them as close obstacles"
            scan360.append(3000)
            continue

        idx_f = (135 - raw_angle) * per_degree
        #average +/- window size raw samples around that index
        #window size for each degree bin is 8
        #because we have ~4 raw samples per degree (1081 samples over 270 degrees)
        #a window size of 8 averages across +/- 8 raw sample binds which is about +/- 2 degrees of field of view
        #integer slice bounds
        low = int(math.floor(idx_f - 8))
        high = int(math.ceil(idx_f + 8))
        #clamp to valid array range
        low = max(low, 0)
        high = min(high, len(clipped) - 1)
        #average the integer slice
        scan360.append(float(np.mean(raw[low:high + 1])))

    return np.array(scan360)


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

def pidControl(left, right)-> float:
    global lastError, integralError, ki, kp, kd
    # 1) compute the normalized lateral error
    error = (right - left)
    # 2) integral term (simple sum; dt = 1 per frame)
    integralError += error
    # 3) derivative term
    derivative = error - lastError
    # 4) remember current error for next frame's derivative
    lastError = error
    # 5) PID output
    return kp * error + ki * integralError + kd * derivative
#racecar callbacks

# [FUNCTION] The start function is run once every time the start button is pressed
def start():
    global lastError, integralError, max_speed
    rc.drive.set_max_speed(max_speed)
    rc.drive.stop()
    #clear out any leftover integral or derivative history
    lastError = 0.0
    integralError = 0.0


# [FUNCTION] After start() is run, this function is run once every frame (ideally at
# 60 frames per second or slower depending on processing speed) until the back button
# is pressed  
def update():
    global desired_speed
    scan360 = update_lidar()
    if scan360 is None:
        return
    #get left/right sector readings
    left_vals  = get_sector_samples(scan360, leftSide)
    right_vals = get_sector_samples(scan360, rightSide)
    #find median distances
    m_left  = getMedian(left_vals)
    m_right = getMedian(right_vals)
    steer = pidControl(m_left, m_right) + 0.2
    steer = max(min(steer, 1.0), -1.0)
    #drive
    rc.drive.set_speed_angle(desired_speed, steer)

# [FUNCTION] update_slow() is similar to update() but is called once per second by
# default. It is especially useful for printing debug messages, since printing a 
# message every frame in update is computationally expensive and creates clutter
count = 0
def update_slow():
    global count
    if count == 2:
        count = 0
        scan360 = update_lidar()
        if scan360 is None:
            return
        left_vals  = get_sector_samples(scan360, leftSide)
        right_vals = get_sector_samples(scan360, rightSide)
        print(f"Left dist: {getMedian(left_vals.tolist()):.2f}, Right dist: {getMedian(right_vals.tolist()):.2f}")
    else:
        count += 1

########################################################################################
# DO NOT MODIFY: Register start and update and begin execution
########################################################################################

if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()