#!/usr/bin/env python3
import math
import rospy
import numpy as np
from geometry_msgs.msg import Pose
import matplotlib.pyplot as plt
from ackermann_msgs.msg import AckermannDriveStamped
import asurt_msgs.msg as asurt_msgs

# from std_msgs.msg import Path
from nav_msgs.msg import Path

message = AckermannDriveStamped()
message.drive.steering_angle = 0
DT= 0.1
BaseWidth = 2.9
gainLH = 0.5
LOOKAHEADCONSTANT = 2.0
class State:
    """
    doc string
    """

    def __init__(self, x: float, y: float, yaw: float, currentSpeed: float = 0.0) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw
        self.currentSpeed = currentSpeed
        self.rearX = self.x - ((BaseWidth / 2) * math.cos(self.yaw))
        self.rearY = self.y - ((BaseWidth / 2) * math.sin(self.yaw))
        self.oldX = ()
        self.oldY = ()

        pass

    def update(self, currentState: Pose) -> None:
        self.x = currentState.position.x
        self.y = currentState.position.y
        self.yaw = currentState.orientation.z
        self.currentSpeed = math.sqrt((math.pow(self.x-self.oldX,2)+math.pow(self.y-self.oldY, 2)))/DT #or directly from SLAM if it is possible?
        self.rearX = self.x - ((BaseWidth / 2) * math.cos(self.yaw))
        self.rearY = self.y - ((BaseWidth / 2) * math.sin(self.yaw))
        self.oldX = self.x
        self.oldY = self.y
        pass
    def calcDistance(self, point_x:float, point_y:float) -> float:
        """
        
        calculate the distance between the vehicle and the target point
        
        """
        dx = self.rearX - point_x
        dy = self.rearY - point_y
        return math.hypot(dx, dy)    

class States:
    def __init__(self) -> None:
        self.states = []
        self.Y = []
        self.X = []
        self.yaws = []
        self.currentSpeeds = []
        self.old_nearest_point_index = None
        pass

    def add(self, state: State) -> None:
        self.X.append(state.x)
        self.Y.append(state.y)
        self.yaws.append(state.yaw)
        self.currentSpeeds.append(state.currentSpeed)

        pass





class WayPoints:
    def __init__(self) -> None:
        self.waypoints = Path()
        self.X = []
        self.Y = []
        self.old_nearest_point_index = None
        
        pass

    def add(self, waypoint: Pose) -> None:
        self.waypoints = waypoint

        self.X.append(self.waypoints.position.x)                                     #poses[-1].position.x)
        self.Y.append(self.waypoints.position.y)                                     #poses[-1].position.y)
        pass

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rearX - icx for icx in self.X]
            dy = [state.rearY - icy for icy in self.Y]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind], self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1], self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lookahead = gainLH * state.currentSpeed + LOOKAHEADCONSTANT  # update look ahead distance
        
        # search look ahead target point index
        while Lookahead > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lookahead

   


def purepursuitSteercontrol(state: State, trajectory: WayPoints, pind: int):
    """
    pure pursuit steering control
    """
    ind, Lf = trajectory.search_target_index(state)
    trajX = ty = 0
    if pind >= ind:
        ind = pind

    if ind < len(trajectory.X_coordinates):
        trajX = trajectory.X_coordinates[ind]
        ty = trajectory.Y_coordinates[ind]
    else:  # toward goal
        trajX = trajectory.X_coordinates[-1]
        ty = trajectory.Y_coordinates[-1]
        ind = len(trajectory.X_coordinates) - 1

    alpha = math.atan2(ty - state.rearY, trajX - state.rearX) - state.yaw

    delta = math.atan2(2.0 * BaseWidth * math.sin(alpha) / Lf, 1.0)

    return delta, ind
