"""camera_node controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()
#activate the camera
camera = robot.getDevice("camera")


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
camera.enable(timestep)
#enable recognition
camera.recognitionEnable(timestep)
obj=-1

while robot.step(timestep) != -1:
    number_of_objects = camera.getRecognitionNumberOfObjects()
    #print("Recognized objects. -->", number_of_objects)

    objects = camera.getRecognitionObjects()
   
    #if obj has a value remove it from the list
   
    for i in range(number_of_objects):
        pass
        #print position of object on image
        #print("Object position on image: ", objects[i].position_on_image[0],objects[i].position_on_image[1])

        #print object's colors
        #print("Object colors: ", objects[i].colors)

    
    

# Enter here exit cleanup code.
