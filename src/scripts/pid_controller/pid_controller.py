#!/usr/bin/env python3
"""
PID Controller
"""
import rospy

KP = 0.1#rospy.get_param("/gains/proportional")  # propotional gain
KD = 0.1 #rospy.get_param("/gains/differential")  # differential gain
KI = 0.1 #rospy.get_param("/gains/integral")  # integral gain
DT = 0.1 # rospy.get_param("/time_step")  # [s] time step
MAXSPEED = 15 # rospy.get_param("/max_speed")  # [m/s] max speed
MINSPEED = 2 #rospy.get_param("/min_speed")  # [m/s] min speed
TARGETSPEED = 10 #rospy.get_param("/target_speed")  # [m/s] target speed

def targetSpeedCalc(delta: float) -> float:
    """
    Calculating Target Speed for PID controller
    """
    targetSpeed: float = (TARGETSPEED) / (abs(delta) * 4)  # [m/s]
    # waypoints.update(pose)
    if targetSpeed <= MINSPEED: #15 / 3.6:  # min speed
        targetSpeed = MINSPEED #15 / 3.6
    if targetSpeed >= MAXSPEED: #60 / 3.6:  # max speed
        targetSpeed = MAXSPEED #60 / 3.6
    return targetSpeed


def proportionalControl(
    targetSpeed: float, currentSpeed: float
) -> float:  # longitudinal controller
    """
    PID Controller
    """
    acc: float = (
        KP * (targetSpeed - currentSpeed)
        + KD * ((targetSpeed - currentSpeed) / DT)
        + KI * (targetSpeed - currentSpeed) * DT
    )
    return acc
