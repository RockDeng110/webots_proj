"""line_following controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Supervisor
import numpy as np
from matplotlib import pyplot as plt
from scipy import signal




# create the Robot instance.
robot = Supervisor()

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





# add gps and compass
gps = robot.getDevice('gps')
gps.enable(timestep)

compass = robot.getDevice('compass')
compass.enable(timestep)


# Lidar related
lidar = robot.getDevice('LDS-01')
lidar.enable(timestep)
lidar.enablePointCloud()


# Allow LIDAR to warm up
for _ in range(10):  # Wait for 10 simulation steps
    robot.step(timestep)


# Wait for the LIDAR to initialize
if lidar.getNumberOfPoints() == 0:
    print("LIDAR initialization failed: No points available.")
else:
    print("LIDAR initialized successfully.")

ranges = lidar.getRangeImage()
if ranges is None or len(ranges) == 0:
    print("Error: LIDAR range image is empty or not accessible.")
else:
    print(f"LIDAR range image lens: {len(ranges)}")
    print(f"LIDAR range image: {ranges}")

angles = np.linspace(3.1415, -3.1415, 360)

"""
# plot test

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})       
ax.plot(angles,ranges,'.')
plt.show()
"""










marker = robot.getFromDef("marker").getField("translation")
marker.setSFVec3f([0,0,0.2])


display = robot.getDevice('display')
display.setColor(0xFF0000)
       
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
    # print(gsensors)
    # print(white_or_black)
    
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

USE_GPS_AND_COMPASS = 1
# calculate xw, yw and omegaz
def calculate_robots_pose_in_world():
    global xw, yw, omegaz, delta_distance, delta_omegaz, delta_x
    if USE_GPS_AND_COMPASS == 1:
        xw = gps.getValues()[0]
        yw = gps.getValues()[1]
        omegaz=np.arctan2(compass.getValues()[0],compass.getValues()[1])
   
    else:
        delta_x = (phildot + phirdot) * r * delta_time / 2
        delta_omegaz = (phirdot - phildot) * r * delta_time / d
        delta_distance += delta_x
        xw = xw + np.cos(omegaz)*delta_x
        yw = yw + np.sin(omegaz)*delta_x
        omegaz += delta_omegaz  

def world2map(xw, yw, map_width=300, map_height=300, 
              world_min_x=-0.195, world_max_x=0.805, 
              world_min_y=-0.25, world_max_y=0.75):
    """
    Converts world coordinates (xw, yw) to map coordinates (px, py).
    """
    # Calculate scale factors
    scale_x = map_width / (world_max_x - world_min_x)
    scale_y = map_height / (world_max_y - world_min_y)

    # Map world coordinates to map indices
    px = int((xw - world_min_x) * scale_x)
    py = int((yw - world_min_y) * scale_y)

    # 对 Y 坐标进行翻转
    py = map_height - 1 - py

    # Clamp indices to map bounds
    px = max(0, min(map_width - 1, px))
    py = max(0, min(map_height - 1, py))

    return px, py


# Example usage
xw, yw = -0.195, 0.75  # Example world coordinates

map_indices = world2map(xw, yw)
print(f"Map indices: {map_indices}")


# Create a probabilistic occupancy grid map (300x300) initialized to zero
map = np.zeros((300, 300))

# Define the kernel to grow obstacles
kernel_size = 30  # Corresponds to robot's radius (tune this value)
kernel = np.ones((kernel_size, kernel_size))

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # calculate xw, yw and omegaz
    """
    delta_x = (phildot + phirdot) * r * delta_time / 2
    delta_omegaz = (phirdot - phildot) * r * delta_time / d
    delta_distance += delta_x
    xw = xw + np.cos(omegaz)*delta_x
    yw = yw + np.sin(omegaz)*delta_x
    omegaz += delta_omegaz    
    print("err:{} xw:{} yw:{} ".format(np.sqrt(xw**2+yw**2), xw, yw))
    """
    calculate_robots_pose_in_world()
    
    # Get sensor values and calculate robot's direction
    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue()) 
    direction = determine_direction(gsValues)
    # print("direction:", direction)
    
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
    
    # draw trajectory in display
    px, py = world2map(xw, yw)
    print(f"px: {px} py: {py}")
    display.setColor(0xFF0000)
    display.drawPixel(px,py)
    # lidar related
    ranges = lidar.getRangeImage()
    
    # Filter infinite values
    ranges = np.where(np.isinf(ranges), 100, ranges)  # Replace infinite values with 100

    if 0:
        w_T_r = np.array([[np.cos(theta),-np.sin(theta), xw],
                  [np.sin(theta),np.cos(theta), yw],
                  [0,0,1]])            
        ranges = np.array(lidar.getRangeImage())
        ranges[ranges == np.inf] = 100
        X_r = np.array([ranges*np.cos(angles), 
                        ranges*np.sin(angles),
                        np.ones(len(angles))])
        D = w_T_r @ X_r
    else:
        x_r, y_r = [], [] # coordinates in robot's system
        x_w, y_w = [], [] # coordinates in world's system
        for i, angle in enumerate(angles):
            try:
                # Validate the range value
                if not np.isfinite(ranges[i]) or abs(ranges[i]) > 1:
                    #print(f"Skipping invalid range at index {i}: {ranges[i]}")
                    continue
                
                # Robot's coordinate transformation
                x_i = ranges[i] * np.cos(angle)
                y_i = ranges[i] * np.sin(angle)
                x_r.append(x_i)
                y_r.append(y_i)

                # World's coordinate transformation
                x_w_s = x_i * np.cos(omegaz) - y_i * np.sin(omegaz) + xw
                y_w_s = x_i * np.sin(omegaz) + y_i * np.cos(omegaz) + yw

                # Clamp display coordinates
                #x_w_s = max(0, min(300, x_w_s))
                #y_w_s = max(0, min(300, y_w_s))

                # Append valid coordinates
                x_w.append(x_w_s)
                y_w.append(y_w_s)

                # Debugging output
                #print(f"i: {i}, ranges[i]: {ranges[i]:.2f}, angle: {angle:.2f}, x_i: {x_i:.2f}, y_i: {y_i:.2f}, omegaz: {omegaz:.2f}")

                # Display pixel
                #print(f"Drawing pixel at ({x_w_s}, {y_w_s})")
                #display.setColor(0xFFFFFF)
                #px, py = world2map(x_w_s, y_w_s)
                #display.drawPixel(px,py)

                
                # Convert world coordinates to map indices
                px, py = world2map(x_w_s, y_w_s)

                # Increment probability (up to 1)
                map[px, py] = min(1, map[px, py] + 0.01)

                # Convert probability to grayscale (0-255)
                v = int(map[px, py] * 255)
                color = v * 256**2 + v * 256 + v  # Convert to 24-bit color

                # Display the pixel
                display.setColor(color)
                display.drawPixel(px, py)
            except Exception as e:
                print(f"Error at index {i}: {e}")
                continue
        """
        plt.ion()
        # plt.plot(x_r, y_r, '.') # plot points in robot's coordinates system   
        plt.plot(x_w, y_w, '.')  # plot points in world's coordinates system
        #plt.pause(0.001)
        plt.show()
        """
    # Perform 2D convolution to compute the configuration space every 100 timesteps
    if robot.step(timestep) % 100 == 0:
        cmap = signal.convolve2d(map, kernel, mode='same')
        cspace = cmap > 0.9  # Threshold to mark obstacles
        
        # Visualize the configuration space
        plt.clf()
        plt.imshow(cspace, cmap='gray')
        plt.title("Configuration Space")
        plt.pause(0.001)

# Enter here exit cleanup code.
