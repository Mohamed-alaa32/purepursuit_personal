"""
Initilization Pure Pursuit node for vehicle control
"""
import rospy
# from purepursuit import State, WayPoints, States, purepursuitSteercontrol, proportionalControl
#from pure_pursuit import purepursuitSteercontrol
from asurt_msgs.msg import NodeStatus 
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Pose
#from pid_controller import proportionalControl, targetSpeedCalc
from nav_msgs.msg import Path
#from pure_pursuit.src.pure_pursuit.purepursuit.pure_pursuit import purepursuitSteercontrol
# from pure_pursuit.pid_controller.pid_controller import proportionalControl
#from src.ros.purepursuit import purepursuitSteercontrol
#from ros.pid_controller.pid_controller import proportionalControl, targetSpeedCalc
from pure_pursuit import *   #purepursuitSteercontrol, WayPoints, State
#from src.scripts import purepursuitSteercontrol, proportionalControl, targetSpeedCalc, WayPoints, State


def main():
    """
    main function for pure pursuit vehicle control
    """
    rospy.init_node('purepursuit_controller', anonymous=True)
    
    #message = NodeStatus()
    #message.status = #starting
    rospy.wait_for_message("/waypoints", Pose)
    rospy.wait_for_message("/state", Pose)

    controlActionPub = rospy.Publisher("/control_actions", AckermannDrive, queue_size = 10)
    waypoints = WayPoints()
    state = State()
    rospy.Subscriber("/state", Pose, callback=state.update)
    rospy.Subscriber("/waypoints", Pose ,callback=waypoints.add)
   
    
    controlAction = AckermannDrive()
    #message.status = #ready
    rate= rospy.Rate(10)
    while not rospy.is_shutdown(): # and using_pure_pursuit = true
        targetInd, _ =   waypoints.search_target_index(state)
        delta, targetInd = purepursuitSteercontrol(state, waypoints, targetInd) #lateral controller
        targetSpeed = targetSpeedCalc(delta)

        acc = proportionalControl(targetSpeed, state.currentSpeed)  #longitudinal controller
        
        
        
        controlAction.acceleration = acc #proportionalControl(targetSpeed, state.currentSpeed)
        controlAction.steering_angle = delta
       
        # clearance = state.calcDistance(waypoints.X_coordinates[-1], waypoints.Y_coordinates[-1])
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
  


# target_course = WayPoints(X_coordinates, Y_coordinates, Z_coordinates) 
#     targetInd, _ = target_course.search_target_index(state)
#<rosparam file="$(find pure_pursuit)/tests/parameters.yaml" />