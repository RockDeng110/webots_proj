"""line_following controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

MAX_SPEED = 6.28
r = 0.0201 # radius of wheels
d = 0.052  # distance between two wheels
delta_x = 0
delta_omegaz = 0
delta_time = timestep/1000  # loop duration in while in seconds
delta_distance = 0
xw = 0  # displacement along the X-axis of the world 
yw = 0.028  # displacement along the Y-axis of the world
omegaz = 1.57  # Rotations along the Z-axis

# init motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


  
# enable sensors
gs = []
gsNames = [
    'gs0', 'gs1', 'gs2'
]
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)
            
phildot, phirdot = MAX_SPEED, MAX_SPEED # speeds of left and right wheels           
stop_count = 0  
 

# use the ground sensor's values to determine if the robot should turn left, turn right or stop
def determine_direction(gsensors):
    # 1. rank sensor values from max to mid to min, and find out their corresponding index.
    min_value = min(gsensors)
    max_value = max(gsensors)
    min_index = gsensors.index(min_value)
    max_index = gsensors.index(max_value)
    
    all_indices = {0, 1, 2}
    max_min_indices = {min_index, max_index}
    mid_index = (all_indices - max_min_indices).pop()
    mid_value = gsensors[mid_index]
    
    # 2. determine if the region detected by specific sensor is in white or black
    white_or_black = {}
    if (abs(min_value - max_value) > 300):
        white_or_black[max_index] = "white"
        white_or_black[min_index] = "black"
        if (abs(max_value - mid_value) <= abs(min_value - mid_value)):
            white_or_black[mid_index] = "white"
        else:
            white_or_black[mid_index] = "black"
    else: # at least one while one black
        white_or_black[0] = "black"
        white_or_black[1] = "black"
        white_or_black[2] = "black"
    print(gsensors)
    print(white_or_black)
    
    # determin robot should go straight, turn left or turn right
    if (white_or_black[0] == "white" and white_or_black[1] == "black" and white_or_black[2] == "white"):
        return "go_straight" 
    elif (white_or_black[0] == "white" and white_or_black[1] == "white" and white_or_black[2] == "white"):
        print("error state!!! shouldn't happen!!!!!!!!!!!!!!!!!!!!")
        return "3 white"
    elif (white_or_black[0] == "black" and white_or_black[1] == "black" and white_or_black[2] == "black"):
        return "stop"
    if (white_or_black[0] == "white"):
        return "turn_right"
    elif (white_or_black[2] == "white"):
        return "turn_left"
    else:
        print("unexpected state!!!!!!!!!!!!!!!!!!!!!!!")
        return "unexpected state"




# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # calculate xw, yw and omegaz
    delta_x = (phildot + phirdot) * r * delta_time / 2
    delta_omegaz = (phirdot - phildot) * r * delta_time / d
    delta_distance += delta_x
    xw = xw + np.cos(omegaz)*delta_x
    yw = yw + np.sin(omegaz)*delta_x
    omegaz += delta_omegaz    
    print("err:{} xw:{} yw:{} ".format(np.sqrt(xw**2+yw**2), xw, yw))
    
    # Get sensor values and calculate robot's direction
    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue()) 
    direction = determine_direction(gsValues)
    print("direction:", direction)
    
    if (direction == "stop"):
        stop_count += 1
        phildot, phirdot = 0.25* MAX_SPEED, 0.25* MAX_SPEED
        print("stop:{}".format(stop_count))
        if stop_count >= 10:
            phildot, phirdot = 0, 0
            if stop_count > 15:
                #print("d:{} w:{} error:{}".format(delta_distance, delta_omegaz, np.sqrt(xw**2+yw**2)))
                break 
    else:
        stop_count = 0
        if (direction == "turn_left"):
            phildot, phirdot = 0 * MAX_SPEED, 0.5*MAX_SPEED
        elif (direction == "turn_right"):
            phildot, phirdot = 0.5 * MAX_SPEED, 0*MAX_SPEED
        elif (direction == "go_straight"):   
            phildot, phirdot = MAX_SPEED, MAX_SPEED

        
                     
    leftMotor.setVelocity(phildot)
    rightMotor.setVelocity(phirdot)
    pass


# Enter here exit cleanup code.
