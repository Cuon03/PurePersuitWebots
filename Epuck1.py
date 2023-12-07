"""EpuckController1 controller."""

from controller import Robot, Motor, Emitter, Receiver, GPS, InertialUnit
import math
import struct

def PurePersuit(L, current_x, current_y, current_z, x_goal, y_goal, max_speed):
    distance = math.sqrt((current_x - x_goal)**2 + (current_y - y_goal)**2)
        
    # #Check if the robot has reached the goal position
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
    
def RunRobot(robot):
    timestep = 64
    max_speed = 5.0
    L = 0.05 # Look-ahead distance
    
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
    
    # Main loop:
    while robot.step(timestep) != -1:
      # receive command from epuck_0
      while receiver.getQueueLength() > 0:
          
          data = receiver.getBytes()
          x_goal, y_goal = struct.unpack('ff', data)
          x_goal = round(x_goal, 2)
          y_goal = round(y_goal, 2)
        
          gps_value = gps.getValues()
          IMU_value = IMU.getRollPitchYaw()
            
          # Get the x, y, z coordinates
          current_x = round(gps_value[0], 2)
          current_y = round(gps_value[1], 2)
          current_z = IMU_value[2]
            
          # Get the left and Right wheels' speed
          left_speed, right_speed = PurePersuit(L, current_x,  current_y, current_z, x_goal, y_goal, max_speed)
        
          # Set the wheel speeds for the robot
          left_motor.setVelocity(left_speed)
          right_motor.setVelocity(right_speed)
          
          receiver.nextPacket()
if __name__ == "__main__":
    robot = Robot()
    RunRobot(robot)    
# Enter here exit cleanup code.
