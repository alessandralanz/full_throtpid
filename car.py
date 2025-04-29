"""
MIT BWSI Autonomous RACECAR
MIT License
racecar-neo-oneshot-labs

File Name: template.py
Title: Wall-Following Across Sim & Real
Author: [Your Name]
Purpose: Follow a wall on either the sim (360° LiDAR) or real car (270° LiDAR, Hokuyo UST-10LX)
Expected Outcome: The car will drive along the wall, keeping a constant lateral distance.
"""

########################################################################################
# Imports
########################################################################################
import sys
import math
import random
import numpy

sys.path.insert(1, "../../library")
import racecar_core
import simulation.lidar_sim as ldr

########################################################################################
# PID gains & globals
########################################################################################
kp = 4
ki = 0
kd = 1

rc = racecar_core.create_racecar()
max_speed = desired_speed = 0.5

# degree-based wall-following sectors (relative to front = 0°)
sections = {
    "s1": ( 45, 135),
    "s2": (135, 225),
    "s3": (225, 315),
    "s4": (315, 405),
    "s5": (405, 495),
    "s6": (495, 585),
    "s7": (585, 675),
    "s8": (-45,  45),
}

# initialize to sim defaults
leftSide = sections["s7"]  # sim default
rightSide = sections["s1"]  # sim default

lastError = 0.0
integralError = 0.0

lidarSim = ldr.LidarSim(rc)
use_sim = True  # will detect at runtime

########################################################################################
# Utility functions
########################################################################################
def quickselect(l, k, pivot_fn):
    if len(l) == 1:
        assert k == 0
        return l[0]
    pivot = pivot_fn(l)
    lows   = [el for el in l if el < pivot]
    highs  = [el for el in l if el > pivot]
    pivots = [el for el in l if el == pivot]
    if k < len(lows):
        return quickselect(lows, k, pivot_fn)
    elif k < len(lows) + len(pivots):
        return pivots[0]
    else:
        return quickselect(highs, k - len(lows) - len(pivots), pivot_fn)

def getMedian(l, pivot_fn=random.choice) -> float:
    if len(l) == 0:
        return 0.0
    n = len(l)
    if n % 2 == 1:
        return quickselect(l, n // 2, pivot_fn)
    else:
        return 0.5 * (quickselect(l, n//2 - 1, pivot_fn)
                    + quickselect(l, n//2, pivot_fn))

def get_sector_samples(samples, sector):
    start, end = sector
    n = len(samples)
    start %= n
    end   %= n
    if start < end:
        return samples[start:end]
    else:
        return samples[start:] + samples[:end]

########################################################################################
# PID controller
########################################################################################
def pidControl(left, right) -> float:
    global lastError, integralError
    if (left + right) == 0:
        error = 0.0
    else:
        error = (right - left) / (right + left)
    integralError += error
    derivative = error - lastError
    lastError = error
    return kp * error + ki * integralError + kd * derivative

########################################################################################
# Calibration for Hokuyo UST-10LX forward offset
########################################################################################
def calibrate_forward_offset(n, start_angle, resolution, num_reads=5):
    # Collect multiple scans to average out noise
    acc = [0.0] * n
    for _ in range(num_reads):
        s = rc.lidar.get_samples()
        for i, d in enumerate(s):
            acc[i] += d
    mean_dist = [x / num_reads for x in acc]
    # Measured forward direction = index with max distance
    measured_idx = max(range(n), key=lambda i: mean_dist[i])
    # Nominal forward index in ideal scan: angle=0° 
    nominal_idx = int(round((0.0 - start_angle) / resolution))
    # Compute offset between measured and nominal
    return (measured_idx - nominal_idx) % n

########################################################################################
# Racecar callbacks
########################################################################################
def start():
    global leftSide, rightSide, use_sim, lastError, integralError

    # set up drive
    rc.drive.set_max_speed(max_speed)
    rc.drive.stop()
    lastError = integralError = 0.0

    # --- Detect sim vs real by trying sim API ---
    try:
        scan = lidarSim.get_samples_async()
        use_sim = True
    except Exception:
        # fallback to real LiDAR
        use_sim = False
        scan = rc.lidar.get_samples()
        while len(scan) == 0:
            scan = rc.lidar.get_samples()

    n = len(scan)

    if use_sim:
        # simulation: use original degree windows directly
        leftSide = sections["s7"]
        rightSide = sections["s1"]
    else:
        # real: Hokuyo UST-10LX spec mapping
        fov = 270.0
        start_angle = -135.0  # initial assumed start
        resolution = fov / (n - 1)
        # calibrate actual forward offset
        offset_idx = calibrate_forward_offset(n, start_angle, resolution)
        # build sectors with offset
        idx_sectors = {}
        for name, (d0, d1) in sections.items():
            # nominal indices
            nom0 = int(round((d0 - start_angle) / resolution))
            nom1 = int(round((d1 - start_angle) / resolution))
            # apply offset
            i0 = (nom0 + offset_idx) % n
            i1 = (nom1 + offset_idx) % n
            idx_sectors[name] = (i0, i1)
        leftSide  = idx_sectors["s7"]
        rightSide = idx_sectors["s1"]

    # end of start setup


def update():
    # branch between sim and real scan
    samples = (lidarSim.get_samples_async() if use_sim else rc.lidar.get_samples())
    left_vals  = get_sector_samples(samples, leftSide)
    right_vals = get_sector_samples(samples, rightSide)
    mL = getMedian(left_vals)
    mR = getMedian(right_vals)
    ctrl = pidControl(mL, mR)
    angle = max(-1.0, min(1.0, ctrl))
    rc.drive.set_speed_angle(max_speed, angle)


def update_slow():
    samples = (lidarSim.get_samples_async() if use_sim else rc.lidar.get_samples())
    left_vals  = get_sector_samples(samples, leftSide)
    right_vals = get_sector_samples(samples, rightSide)
    print(f"Left distance:  {getMedian(left_vals):.3f}")
    print(f"Right distance: {getMedian(right_vals):.3f}")

########################################################################################
# DO NOT MODIFY: Register start/update and begin execution
########################################################################################
if __name__ == "__main__":
    rc.set_start_update(start, update, update_slow)
    rc.go()
