"""agriControl controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor,Keyboard,Camera

# create the Robot instance.
robot = Robot()


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
motFR = robot.getDevice('motFR')
motFL = robot.getDevice('motFL')
motRR = robot.getDevice('motRR')
motRL = robot.getDevice('motRL')
cam = robot.getDevice('camera')
cam.enable(timestep)
kys = robot.getKeyboard()
kys.enable(timestep)


motFR.setPosition(float('inf'))
motFL.setPosition(float('inf'))
motRR.setPosition(float('inf'))
motRL.setPosition(float('inf'))
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    key1=kys.getKey()
    # Process sensor data here.
    if(key1==ord('A')):
     
        motFR.setVelocity(4)
        motFL.setVelocity(-4)
        motRR.setVelocity(4)
        motRL.setVelocity(-4)
        
    elif(key1==ord('D')):
        
        motFR.setVelocity(-4)
        motFL.setVelocity(4)
        motRR.setVelocity(-4)
        motRL.setVelocity(4)
        
    elif(key1==ord('W')):
    
        motFR.setVelocity(2)
        motFL.setVelocity(2)
        motRR.setVelocity(2)
        motRL.setVelocity(2)
        
    elif(key1==ord('S')):
        
        motFR.setVelocity(-2)
        motFL.setVelocity(-2)
        motRR.setVelocity(-2)
        motRL.setVelocity(-2)

    else:
        
        motFR.setVelocity(0)
        motFL.setVelocity(0)
        motRR.setVelocity(0)
        motRL.setVelocity(0)
        
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    

# Enter here exit cleanup code.
