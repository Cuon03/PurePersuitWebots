"""EpuckController0 controller."""

from controller import Robot, Motor, Emitter, Receiver, GPS, InertialUnit, DistanceSensor
import math
import struct
import numpy as np

def Turn_left(max_speed):
    left_speed = -max_speed
    right_speed = max_speed
    return left_speed, right_speed

def Turn_right(max_speed):
    left_speed = max_speed
    right_speed = -max_speed
    return left_speed, right_speed
       
def Get_current_position(gps_value, IMU_value):
    # Get the x, y, z coordinates
    current_x = round(gps_value[0], 2)
    current_y = round(gps_value[1], 2)
    current_z = IMU_value[2]
    return current_x, current_y, current_z
    
def VirtualTargetGenaration(x_0, y_0, rh):
    phi_0 = 0 
    phi_h = math.pi / 3
    phi_ij = []
    P_ij = []
    for j in range (1, 7):
        # Genarate the virtual targets' angle
        phi_ij.append( phi_0 + (j - 1) * phi_h )
        # Genarate the virtual targets' position
        P_ij.append([round(x_0 + rh * math.sin(phi_ij[j-1]), 2), round(y_0 + rh * math.cos(phi_ij[j-1]), 2) ])
    
    return phi_ij, P_ij

def PurePersuit(L, current_x, current_y, current_z, x_goal, y_goal, max_speed):
    distance = math.sqrt((current_x - x_goal)**2 + (current_y - y_goal)**2)
        
    # Check if the robot has reached the goal position
    if distance < 0.01:
        # Stop the robot
        left_speed = 0.0
        right_speed = 0.0
        return left_speed, right_speed
        
    # Calculate the targets' distance
    target_dist = min(L, distance)
    target_x = current_x + (target_dist / distance) * (x_goal - current_x)
    target_y = current_y + (target_dist / distance) * (y_goal - current_y)
    # Calculate the steering angle using the Pure Pursuit algorithm
    alpha = math.atan2(target_y - current_y, target_x - current_x) - current_z
    delta = math.atan2(2* L* math.sin(alpha), L**2)
    # Set the wheel speeds based on the steering angle
    left_speed = max_speed * (1 - delta / (2* math.pi))
    right_speed = max_speed * (1 + delta / (2* math.pi))
    
    return left_speed, right_speed

def Go_to_target_position(distance_val, L, current_x, current_y, current_z, x_goal, y_goal, max_speed):
     if distance_val[7] > 76 or distance_val[6] > 76 : #the obstacle at the left
         left_speed, right_speed = Turn_right(max_speed)
     elif distance_val[0] > 76 or distance_val[1] > 76 : #the obstacle at the right
         left_speed, right_speed = Turn_left(max_speed)
     else:
         left_speed, right_speed = PurePersuit(L, current_x, current_y, current_z, x_goal, y_goal, max_speed)
     return left_speed, right_speed
     
def Heuristic(node, goal):
    # Euclidean distance between two nodes
    node_x, node_y = node
    goal_x, goal_y = goal
    return math.sqrt((node_x - goal_x)**2 + (node_y - goal_y)**2)



def RunRobot(Robot):
    timestep = 64
    max_speed = 5.0
    rh = 1 # the radii of hexagon 
    L = 0.5 # Look-ahead distance for Pure persuit
    
    # Enable Devices
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)  
    emitter = robot.getDevice('emitter')
    receiver = robot.getDevice('receiver')
    receiver.enable(timestep)
    gps = robot.getDevice('gps')
    gps.enable(timestep)
    IMU = robot.getDevice('inertial unit')
    IMU.enable(timestep)
    sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
    for name in sensor_names:
        distance_sensor = robot.getDevice(name)
        distance_sensor.enable(timestep)
        
    # Main loop:
    while robot.step(timestep) != -1:
        # Get the distance sensors' value
        distance_val = []
        for name in sensor_names:
            distance_sensor = robot.getDevice(name)
            distance_val.append(distance_sensor.getValue())
        name = robot.getName()
        if name == "e-puck0":
            gps.enable(1)  # Enable the GPS device with a refresh rate of 1ms
            while robot.step(1) != -1:
                if gps.getValues() is not None:
                    break
            first_position = gps.getValues()
            x_0 = first_position[0]
            y_0 = first_position[1]
            # send the current position of the landmark robot to others
            emitter.send(struct.pack('ff', *[x_0, y_0]))
        else:
            # receive command from epuck_0
            while receiver.getQueueLength() > 0:
              data = receiver.getBytes()
              x_centre, y_centre = struct.unpack('ff', data)
              
              receiver.nextPacket()
              
              # Generate virtual targets
              goal_positions = VirtualTargetGenaration(x_centre, y_centre, rh)[1]
              
              # Get the current position of the robots
              gps_value = gps.getValues()
              IMU_value = IMU.getRollPitchYaw()
              
              #Control robots go to the virtual targets
              for i in range (6):
                  x_goal, y_goal = goal_positions[i]
                  # print(i, x_goal, y_goal)
                  
                  if str(i+1) == robot.getName()[-1]: 
                      # Get the x, y, z coordinates
                      current_x, current_y, current_z = Get_current_position(gps_value, IMU_value)
                      # Get the left and Right wheels' speed
                      left_speed, right_speed = Go_to_target_position(distance_val, L, current_x, current_y, current_z, x_goal, y_goal, max_speed)
                      # Set the wheel speeds for the robot
                      left_motor.setVelocity(left_speed)
                      right_motor.setVelocity(right_speed)
                        
if __name__ == "__main__":
    robot = Robot()
    RunRobot(robot)

# Enter here exit cleanup code.
