#!/usr/bin/env python3
"""
Initilization Pure Pursuit node for vehicle control
"""
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
# from purepursuit import State, WayPoints, States, purepursuitSteercontrol, proportionalControl
#from pure_pursuit import purepursuitSteercontrol
#from asurt_msgs.msg import NodeStatus 
#import vehicle
#from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Pose
#from pid_controller import proportionalControl, targetSpeedCalc
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
import time
#from pure_pursuit.src.pure_pursuit.purepursuit.pure_pursuit import purepursuitSteercontrol
# from pure_pursuit.pid_controller.pid_controller import proportionalControl
#from src.ros.purepursuit import purepursuitSteercontrol
#from ros.pid_controller.pid_controller import proportionalControl, targetSpeedCalc
#from pure_pursuit import *   #purepursuitSteercontrol, WayPoints, State
#from src.scripts import purepursuitSteercontrol, proportionalControl, targetSpeedCalc, WayPoints, State
BaseWidth = 2.9
gainLH = 0.1
KP = 0.1#rospy.get_param("/gains/proportional")  # propotional gain
KD = 0.1 #rospy.get_param("/gains/differential")  # differential gain
KI = 0.1 #rospy.get_param("/gains/integral")  # integral gain
DT = 0.1 # rospy.get_param("/time_step")  # [s] time step
MAXSPEED = 15 # rospy.get_param("/max_speed")  # [m/s] max speed
MINSPEED = 2 #rospy.get_param("/min_speed")  # [m/s] min speed
TARGETSPEED = 10 #rospy.get_param("/target_speed")  # [m/s] target speed
LOOKAHEADCONSTANT = 2.0
class State:
    """
    doc string
    """

    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float =0.0, currentSpeed: float = 0.0) -> None:
        self.x = x
        self.y = y
        self.yaw = yaw
        self.currentSpeed = currentSpeed
        self.rearX = self.x - ((BaseWidth / 2) * math.cos(self.yaw))
        self.rearY = self.y - ((BaseWidth / 2) * math.sin(self.yaw))
        self.oldX = self.x
        self.oldY = self.y
        self.state_sub = rospy.Subscriber("/odometry_node/trajectory" , Path , self.update) 

        pass

    def update(self, currentState: Path) -> None:
        #global states,delta
        currentTime = time.time()
        self.x = currentState.poses[-1].pose.position.x
        self.y = currentState.poses[-1].pose.position.y
        #self.currentSpeed = ((self.x - self.oldX[-1])**2 + (self.y - self.oldY[-1])**2)/ (currentTime-self.time)
        #self.yaw += self.currentSpeed / BaseWidth * math.tan(delta) * dt   
        quat_msg = currentState.poses[-1].pose.orientation
        quat_list = [quat_msg.x,quat_msg.y,quat_msg.z,quat_msg.w]
        (_, _, self.yaw) = euler_from_quaternion(quat_list)
        # self.x = currentState.position.x
        # self.y = currentState.position.y
        #self.yaw = currentState.orientation.z
        self.currentSpeed = math.sqrt((math.pow(self.x-self.oldX,2)+math.pow(self.y-self.oldY, 2)))/DT #or directly from SLAM if it is possible?
        self.rearX = self.x - ((BaseWidth / 2) * math.cos(self.yaw))
        self.rearY = self.y - ((BaseWidth / 2) * math.sin(self.yaw))
        self.oldX = self.x
        self.oldY = self.y
        self.time = currentTime
        rospy.loginfo(self.x)
        rospy.loginfo(self.y)
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
            if len(d) > 0:
                ind = np.argmin(d)
                self.old_nearest_point_index = ind
            else:
                ind = 0
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calcDistance(self.X[ind], self.Y[ind])
            while True:
                distance_next_index = state.calcDistance(self.X[ind + 1], self.Y[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.X) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lookahead = gainLH * state.currentSpeed + LOOKAHEADCONSTANT  # update look ahead distance
        
        # search look ahead target point index
        while Lookahead > state.calcDistance(self.X[ind], self.Y[ind]):
            if (ind + 1) >= len(self.X):
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

    if ind < len(trajectory.X):
        trajX = trajectory.X[ind]
        ty = trajectory.Y[ind]
    else:  # toward goal
        trajX = trajectory.X[-1]
        ty = trajectory.Y[-1]
        ind = len(trajectory.X) - 1

    alpha = math.atan2(ty - state.rearY, trajX - state.rearX) - state.yaw

    delta = math.atan2(2.0 * BaseWidth * math.sin(alpha) / Lf, 1.0)

    return delta, ind





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

def main():
    """
    main function for pure pursuit vehicle control
    """
    rospy.init_node('purepursuit_controller', anonymous=True)
    
    #message = NodeStatus()
    #message.status = #starting
    #rospy.wait_for_message("/waypoints", Pose)
    #rospy.wait_for_message("/state", Pose)

    controlActionPub = rospy.Publisher("/sub_control_actions", Odometry, queue_size = 10)
    waypoints = WayPoints()
    state = State()
    #rospy.Subscriber("/state", Pose, callback=state.update)
    rospy.Subscriber("/waypoints", Pose ,callback=waypoints.add)
   
    
    controlAction = Odometry()
    #message.status = #ready
    rate= rospy.Rate(10)
    while not rospy.is_shutdown(): # and using_pure_pursuit = true
        targetInd, _ =   waypoints.search_target_index(state)
        delta, targetInd = purepursuitSteercontrol(state, waypoints, targetInd) #lateral controller
        targetSpeed = targetSpeedCalc(delta)

        # acc = proportionalControl(targetSpeed, state.currentSpeed)  #longitudinal controller
        
        controlAction.twist.twist.linear.x = targetSpeed
        controlAction.twist.twist.linear.z = delta
        controlAction.twist.twist.linear.y = state.currentSpeed
        # controlAction.acceleration = acc #proportionalControl(targetSpeed, state.currentSpeed)
        # controlAction.steering_angle = delta
       
        # clearance = state.calcDistance(waypoints.X[-1], waypoints.Y[-1])
        # if clearance <= 1.4:
        #     # goal_reached = True
        #     print("Goal Reached")
        #     controlAction.acceleration = 0.0
        #     controlAction.steering_angle = 0.0
        
        controlActionPub.publish(controlAction)
        
        #message.status = #running
            
        rate.sleep()
    

    # assert lastIndex >= targetInd, "Cannot reach goal"
    


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
  


# target_course = WayPoints(X, Y, Z_coordinates) 
#     targetInd, _ = target_course.search_target_index(state)
#<rosparam file="$(find pure_pursuit)/tests/parameters.yaml" />
