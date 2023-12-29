"""epuck_go_forward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import numpy as np
from sympy import symbols, cos, sin, atan2, solve, sqrt


from controller import Robot, Motor,Keyboard,Camera,Supervisor
import ikpy
from ikpy.chain import Chain
import sys
import tempfile

MAX_SPEED = 6.28
supervisor = Supervisor()
cam = supervisor.getDevice("camera")
timeStep = int(4 * supervisor.getBasicTimeStep())
cam.enable(timeStep)
cam.recognitionEnable(timeStep)
mot = supervisor.getDevice('mot1')
mot2 = supervisor.getDevice('mot2')
mot3 = supervisor.getDevice('mot3')
wrist=supervisor.getDevice("wristMot")
finger1=supervisor.getDevice("linMot1")
finger2=supervisor.getDevice("linMot2")

fin1 = supervisor.getDevice('finsen1')
fin1.enable(timeStep)
fin2 = supervisor.getDevice('finsen2')
fin2.enable(timeStep)

ps1 = supervisor.getDevice('ps1')
ps1.enable(timeStep)

ps2 = supervisor.getDevice('ps2')
ps2.enable(timeStep)

ps3 = supervisor.getDevice('ps3')
ps3.enable(timeStep)

kys = supervisor.getKeyboard()
kys.enable(timeStep)
obj=-1
# mot.setPosition(float('inf'))
# mot2.setPosition(float('inf'))
# mot3.setPosition(float('inf'))

def mapVals(value, from_low, from_high, to_low, to_high):
    # Map the value from one range to another
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low
def rtd(radians):
    degrees = radians * (180 / math.pi)
    return degrees

def dtr(degrees):
    radians = degrees * (math.pi / 180)
    return radians


def ik(x,y,z):
    l1 = 0.1
    l2 = 0.1
    if(x<0):x=x*-1
    # calculate theta1
    if(x>0):
        theta1 = math.atan(y/x)
    else:
        if(y>0):
            theta1=1.57
        else:
            theta1=-1.57
    
       
    projection = math.sqrt(x**2 + y**2)

    # calculate theta2
    al = math.sqrt((projection**2) + (z**2))
    if z==0:
        z-=0.005
    if z > 0:
        theta2 = math.atan(projection/z) - math.acos((l1**2 + al**2 - l2**2  )/(2*l1*al))
        theta3 = math.pi - math.acos((l1**2 + l2**2 - al**2)/(2*l1*l2))
    else:
        z = abs(z)
        theta2 = math.atan(projection/z) - math.acos((l1**2 + al**2 - l2**2  )/(2*l1*al))
        theta3 = math.pi - math.acos((l1**2 + l2**2 - al**2)/(2*l1*l2))
        #calculate the distance from the origin to the point
        distance = math.sqrt(x**2 + y**2 + z**2)
        
        angle = math.acos((x**2+y**2-z**2)/distance**2)
        theta2 = angle + theta2

    # calculate theta3
    

    return theta1, theta2, theta3, (3.14-(theta2+theta3))

wx = 0.1
wy = 0.0
wz = 0.04
finpos1=0
finpos2=0
busy=0
finger1.setPosition(0.03)
finger2.setPosition(0.03)
phase=0
while supervisor.step(timeStep) != -1:
    #x = 0.001 y = -0.08 , z = 0.02
    #print(f"fing1 = {fin1.getValue()}")
    #print(f"fing2 = {fin2.getValue()}")



    number_of_objects = cam.getRecognitionNumberOfObjects()
    objects = cam.getRecognitionObjects()
   
   
    
    for i in range(number_of_objects):   
        #pass
        #print position of object on image
        #print("Object position on image: ", objects[i].position_on_image[0],objects[i].position_on_image[1])
        
        #print object's colors
        print("Object colors: ", objects[i].colors[0],objects[i].colors[1],objects[i].colors[2])
        if(objects[i].colors[0]!=0.8):
            wx = mapVals(objects[i].position_on_image[0], 1, 62, 0.16, 0.06)
            wy = mapVals(objects[i].position_on_image[1], 1, 62, -0.05, 0.04)
            phase=1

    
    wx = round(wx,3)
    wy = round(wy,3)
    wz = round(wz,3)
    want = ik(wx,wy,wz)
    
    mot.setPosition(want[0])
    mot2.setPosition(want[1])
    mot3.setPosition(want[2])
    wrist.setPosition(want[3])
    # Read the sensors:
    print(f"wx = {wx} wy = {wy} , wz = {wz}")
    #print(f"ps1 = {ps1.getValue()} ps2 = {ps2.getValue()} , ps3 = {ps3.getValue()}")
    #print(f"w1 = {want[0]} w2 = {want[1]} , w3 = {want[2]}")
    #RMSE calculations
    err1=((ps1.getValue()*(180/3.14))-want[0]*(180/3.14))**2
    err2=(ps2.getValue()*(180/3.14)-want[1]*(180/3.14))**2
    err3=(ps3.getValue()*(180/3.14)-want[2]*(180/3.14))**2
    rmse=math.sqrt((err1+err2+err3)/3)
    
    if(rmse<3 and phase==1):
        wz=0.02
        
    
    print("rmse ==",rmse)
    
    key1=kys.getKey()
   
    if(key1==65):
        wx+=0.01
    elif(key1==68):    
        wx-=0.01
    elif(key1==87):
        wy+=0.01
    elif(key1==83):
        wy-=0.01
    elif(key1==81):
        wz+=0.01
    elif(key1==69):
        wz-=0.01
    elif(key1==88):
        finpos2+=0.0005
        finger2.setPosition(finpos2)
        finpos1+=0.0005
        finger1.setPosition(finpos1)
    elif(key1==90):
        finpos2-=0.0005
        finger2.setPosition(finpos2)
        finpos1-=0.0005
        finger1.setPosition(finpos1)
    
        
   
    