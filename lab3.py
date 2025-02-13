"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module.
import math
from controller import Robot, Motor, DistanceSensor, Supervisor
import numpy as np

pose_x = 0
pose_y = 0
pose_theta = 0

# create the Robot instance.
robot = Supervisor()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.1257 # ePuck wheel speed in m/s
MAX_SPEED = 6.28

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10): robot.step(SIM_TIMESTEP)  

vL = 0
vR = 0

#xr = 0
#yr = 0
#theta = 0

# Initialize gps and compass for odometry
gps = robot.getDevice("gps")
gps.enable(SIM_TIMESTEP)
compass = robot.getDevice("compass")
compass.enable(SIM_TIMESTEP)

# TODO: Find waypoints to navigate around the arena while avoiding obstacles
waypoints = [[-0.3, -0.42], [0.3, -0.42], [0.3, -0.24], [0.04, -0.02], [0.36, 0.25], [0.135, 0.42], [-0.31, 0.42], [-0.43, 0.28], [-0.43, 0.0]]
# Index indicating which waypoint the robot is reaching next
index = 0

# Get ping pong ball marker that marks the next waypoint the robot is reaching
marker = robot.getFromDef("marker").getField("translation")

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:
    # Set the position of the marker
    marker.setSFVec3f([waypoints[index][0], waypoints[index][1], 0.01])
    
    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    # Read pose_x, pose_y, pose_theta from gps and compass
    pose_x = gps.getValues()[0]
    pose_y = gps.getValues()[1]
    pose_theta = np.arctan2(compass.getValues()[0], compass.getValues()[1])
    
    fov = math.pi / 16
    close_enough = 0.03

    waypoint_dist_x = waypoints[index][0] - pose_x
    waypoint_dist_y = waypoints[index][1] - pose_y
    waypoint_dist = math.sqrt(waypoint_dist_x**2 + waypoint_dist_y**2)
    waypoint_theta = math.atan(waypoint_dist_y / waypoint_dist_x)

    if waypoint_dist_x < 0:
        waypoint_theta = waypoint_theta + (math.pi)

    angle_error = pose_theta - waypoint_theta
    if angle_error >= math.pi: angle_error = angle_error - (2 * math.pi)
    if angle_error <= -math.pi: angle_error = angle_error + (2 * math.pi)

    if (close_enough > waypoint_dist):
        index = index + 1
        if index >= len(waypoints): index = 0

    #Full speed
    if (abs(angle_error) <= fov):
        vL = MAX_SPEED
        vR = MAX_SPEED

    #Angle slightly
    elif angle_error < 0 and angle_error > fov * -2:
        vL = MAX_SPEED / 4
        vR = MAX_SPEED
    elif angle_error > 0 and angle_error < fov * 2:
        vL = MAX_SPEED
        vR = MAX_SPEED / 4

    #Angle sharply
    elif angle_error < 0:
        vL = -MAX_SPEED
        vR = MAX_SPEED
    elif angle_error > 0:
        vL = MAX_SPEED
        vR = -MAX_SPEED

    #Be confused
    else:
        vL = 0
        vR = 0

    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    print("waypoint #: %1f, dist: %5f, waypoint_theta: %5f, angle_error: %5f" % (index, waypoint_dist, waypoint_theta, angle_error))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
