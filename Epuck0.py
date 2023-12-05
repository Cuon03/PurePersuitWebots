"""EpuckController0 controller."""

from controller import Robot, Motor, Emitter, Receiver, GPS, InertialUnit
import math

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

def RunRobot(Robot):
    timestep = 64
    max_speed = 5.0
    phi_ij = []
    P_ij = []
    rh = 0.3 # the radii of hexagon 
    L = 0.05 # Look-ahead distance
    
    # Enable Devices
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)  
    emitter = robot.getDevice('emitter')
    gps = robot.getDevice('gps')
    gps.enable(timestep)
    IMU = robot.getDevice('inertial unit')
    IMU.enable(timestep)
    
    # Get the first position of the robot
    gps.enable(1)  # Enable the GPS device with a refresh rate of 1ms
    while robot.step(1) != -1:
        if gps.getValues() is not None:
            break
    first_position = gps.getValues()
    x_0 = first_position[0]
    y_0 = first_position[1]

    x_goal, y_goal = VirtualTargetGenaration(x_0, y_0, rh)[1][4]
    print ('x_goal = ',x_goal)
    print('y_goal = ',y_goal)
    # Main loop:
    while robot.step(timestep) != -1:
       
        gps_value = gps.getValues()
        IMU_value = IMU.getRollPitchYaw()
        
        # Get the x, y, z coordinates
        current_x = gps_value[0]
        current_y = gps_value[1]
        current_z = IMU_value[2]
        
        # Calculate the distance to the goal position
        distance = math.sqrt((current_x - x_goal)**2 + (current_y - y_goal)**2)
        
        # #Check if the robot has reached the goal position
        if distance < 0.05:
            # Stop the robot
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            break
        
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
    
        # Set the wheel speeds for the robot
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
        #send a command to epuck_1
        command = "move_straight"
        emitter.send(command.encode())
    
if __name__ == "__main__":
    robot = Robot()
    RunRobot(robot)

# Enter here exit cleanup code.
