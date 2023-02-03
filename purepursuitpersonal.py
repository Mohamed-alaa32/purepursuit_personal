import rospy
import math
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, Pose
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt

#parameters
gainLH =  0.3  # look forward gain rospy.get_param()

LOOKAHEADCONSTANT =  2.0  # [m] look-ahead distance
Kp =  1.0  # speed proportional gain
Kd = 0.1
dt =  0.1  # [s] time tick
BaseWidth = 2.9  # [m] wheel base of vehicle
show_animation = True
#getting car state from SLAM
class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, currentSpeed=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.currentSpeed = currentSpeed
        self.rear_x = self.x - ((BaseWidth / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((BaseWidth / 2) * math.sin(self.yaw))
    def update(self, acc, delta):
        # my_pose = rospy.Subscriber("my_pose", PoseStamped, callback=self)
        # my_pose = PoseStamped()
        self.x += self.currentSpeed * math.cos(self.yaw) * dt       #my_pose.pose.position.x
        self.y += self.currentSpeed * math.sin(self.yaw) * dt #my_pose.pose.position.y
        self.yaw += self.currentSpeed / BaseWidth * math.tan(delta) * dt   #my_pose.pose.orientation.z
        self.currentSpeed += acc * dt  #2.0 +  time #my_pose.twist.linear.x
        self.rear_x = self.x - ((BaseWidth / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((BaseWidth / 2) * math.sin(self.yaw))
    def calc_distance(self, point_x, point_y):  #1
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)
#state= State()
#storing the states of the vehicle gotten from SLAM
class States:
    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.currentSpeed = []
        self.time = []
        
    def update(self, time, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.currentSpeed.append(state.currentSpeed)
        self.time.append(self.time) #de lsa hashof hagebha mnen
        

#class for calculating distance between car and next point
class WayPoints:
    def __init__(self, X_coordinates, Y_Coordinates):
        self.X_coordinates : list = X_coordinates  #rospy.Subscriber("X coordinate", PoseStamped, callback=self.callback)
        self.Y_coordinates : list = Y_Coordinates #rospy.Subscriber("y coordinate", Float64MultiArray, callback=self.callback)
        self.old_nearest_point_index = None
    
    def update(self, X_coordinates, Y_coordinates):
        self.X_coordinates.append(X_coordinates)
        self.Y_coordinates.append(Y_coordinates)

    def search_target_index(self, state): #lazm t5osh fe while loop 8aleban kol iteration

        # To speed up nearest point search, doing it at only first time. , de msh hynf3 3shan msh hyb2a m3aya elpoints kolaha men awel mra (to be optimized)
        if self.old_nearest_point_index is None:
            # search nearest point index 
            self.dx = [state.rear_x - iX_coordinates for iX_coordinates in self.X_coordinates]
            self.dy = [state.rear_y - iY_coordinates for iY_coordinates in self.Y_coordinates]
            self.d = np.hypot(self.dx, self.dy)
            ind = np.argmin(self.d) #bey7sb a2al distance ben elcar w kol elpoints elmawgoda w ygeb index elpoint de fe ellist bta3t elpoints
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.X_coordinates[ind],self.Y_coordinates[ind])
            if distance_this_index > 1.0:
                rospy.sleep(0.02)
            while True: #bydwar 3al index elsah 
                
                self.distance_next_index = state.calc_distance(self.X_coordinates[ind + 1],self.Y_coordinates[ind + 1])
                if distance_this_index < self.distance_next_index:
                    break
                if (ind + 1) < len(self.X_coordinates):
                    ind = ind + 1  
                else :
                    ind = ind
                distance_this_index = self.distance_next_index
            self.old_nearest_point_index = ind

        # hases en mkanha msh hena > Lf = k * state.v + Lfc  # update look ahead distance
        Lookahead = gainLH * state.currentSpeed + LOOKAHEADCONSTANT  # update look ahead distance
        # search look ahead target point index , bey3od yzawed fel index lehad ma yla2y awel point m3adeya el lookahead distance circle (mmkn ya5od ely ablaha)
        while Lookahead > state.calc_distance(self.X_coordinates[ind], self.Y_coordinates[ind]):
            if (ind + 1) >= len(self.X_coordinates):
                break  # not exceed goal
            ind += 1

        return ind, Lookahead    #beytl3 index elpoint ely hangeb 3ndha el steering angle
        

def pure_pursuit_steer_control(state:State, trajectory:WayPoints, pind): #3 #limitation of delta [-pi/3, pi/3]
    ind, Lf = trajectory.search_target_index(state)
    tx = ty = 0
    if pind >= ind:
        ind = pind

    if ind < len(trajectory.X_coordinates):
        tx = trajectory.X_coordinates[ind]
        ty = trajectory.Y_coordinates[ind]
    else:  # toward goal
        tx = trajectory.X_coordinates[-1]
        ty = trajectory.Y_coordinates[-1]
        ind = len(trajectory.X_coordinates) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    delta = math.atan2(2.0 * BaseWidth * math.sin(alpha) / Lf, 1.0)

    return delta, ind
# BaseWidth = 3  #distance between rear and front wheel
# dt = 0.1  #time step
# gainLH= 0.1  #lookahead gain
# velocity = 1  
# LOOKAHEADCONSTANT = 2 #lookahead constant

# Lookahead distance is increasing with increased velocity to avoid rough curvatures
# Lookahead = gainLH * velocity + LOOKAHEADCONSTANT

def proportional_control(targetSpeed, currentSpeed):  #longitudinal controller
    
    acc = Kp*(targetSpeed - currentSpeed) + Kd*((targetSpeed-currentSpeed)/dt )
    return acc
    
def main():
    #  target course , de mkan el waypoints
    rospy.init_node ('purepursuit_controller', anonymous=True)
    X_coordinates = np.arange(0, 100, 0.5)
    Y_coordinates = [math.sin(ix / 5.0) * ix / 2.0 for ix in X_coordinates] #[5*math.sin(ix/6.0) for ix in X_coordinates]                                    #[math.sin(ix / 5.0) * ix / 2.0 for ix in X_coordinates]                         #[math.sin(ix / 5.0) *(0*ix +  2.0)  for ix in X_coordinates]
    #pid
    target_speed = (20.0 / 3.6) #- time * 0.1 # [m/s]
    
    T = 100.0  # max simulation time
    
    # initial state
    state = State(x=-0.0, y=-3.0, yaw=0.0, currentSpeed=0.0)
    
    lastIndex = len(X_coordinates) - 1
    time = 0.0
    states = States()
    states.update(time, state)
    
    target_course = WayPoints(X_coordinates, Y_coordinates) #elwaypoints dol eltrajectory
    target_ind, _ = target_course.search_target_index(state)
    
    
    while T >= time or lastIndex > target_ind:
        # da elcontroller kolo longitudinal be pure pursuit
        # Calc control input
        acc = proportional_control(target_speed, state.currentSpeed) #dol eletnen functions bto3 el controller
        di, target_ind = pure_pursuit_steer_control(state, target_course, target_ind)
        state.update(acc, di)  # Control vehicle , da beyb3at steering angle w acceleration
        # rospy.Rate(1)
        rospy.sleep(0.02)
        # target_ind += 1
        target_speed = (20.0/ 3.6)/(abs(di) *4)  # [m/s]
        if target_speed <= 10/3.6:  # min speed
            target_speed = 10/3.6
        if target_speed >= 40/3.6:   #max speed
            target_speed = 40/3.6
        time += dt
        print("target_x:",round(target_course.X_coordinates[target_ind],2),"my_x:",round(state.x,2),
         "target_y:" ,round(target_course.Y_coordinates[target_ind],2), "my_y:", round(state.y,2),"steer:",di,"speed:", round(state.currentSpeed,2),"ind:", target_ind, "target_Speed:", target_speed)
        #print(acc, state.currentSpeed, state.x, state.y, time)
        #x.pose = time
        
        states.update(time, state)
        # plt.plot(state.x, state.y, "-r", label="course")
        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            
            plt.plot(X_coordinates, Y_coordinates, "-r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.plot(X_coordinates[target_ind], Y_coordinates[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.currentSpeed * 3.6)[:4])
            plt.pause(0.001)

    assert lastIndex >= target_ind, "Cannot goal"
    
    if show_animation:  # pragma: no cover
        plt.cla()
        plt.plot(X_coordinates, Y_coordinates, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()

publisher = rospy.Publisher('cmd_vel', Pose, queue_size=10)
# x = PoseStamped()
cmd_vel = Pose()
# cmd_vel.position.x = 1.0
cmd_vel.position.y = 4.0
X_coordinates = np.arange(0, 50, 0.5)
working = 38
timer = 0

while True:
    main()
    
    # print("working")
    rospy.Rate(1)
    # cmd_vel.position.x = 5.0
    publisher.publish(cmd_vel)
    # plt.show()
    # plt.xlabel("Time[s]")
    # plt.ylabel("Speed[km/h]")
    # plt.grid(True)
    # for i in range(10):
    #     timer += 1
    #     rospy.sleep(1)
    # if timer >=9:
    #     break   
    # print()
    # rospy.spin() 