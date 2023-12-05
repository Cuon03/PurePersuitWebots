"""EpuckController1 controller."""

from controller import Robot, Motor, Emitter, Receiver

def RunRobot(robot):
    timestep = 64
    
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)
    
    # Main loop:
    while robot.step(timestep) != -1:
      # receive command from epuck_0
      receiver = robot.getDevice("receiver")
      receiver.enable(timestep)
      while receiver.getQueueLength() > 0:
          command = receiver.getString()
          if command == "move_straight":
              left_motor.setPosition(float('inf'))
              right_motor.setPosition(float('inf'))
              left_motor.setVelocity(6.0)
              right_motor.setVelocity(6.0)
              receiver.nextPacket()
            
if __name__ == "__main__":
    robot = Robot()
    RunRobot(robot)    
# Enter here exit cleanup code.
