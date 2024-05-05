from controller import Robot, Motor, DistanceSensor, Camera, Emitter, GPS, PositionSensor, Gyro, InertialUnit, Lidar, Accelerometer


robot = Robot()
timeStep = 32
MAX = 6.28

wheel1 = robot.getDevice('wheel1 motor')
wheel2 = robot.getDevice('wheel2 motor')

wheel1.setPosition(float('inf'))
wheel2.setPosition(float('inf'))

wheel1.setVelocity(0.0)
wheel2.setVelocity(0.0)


encoder1 = wheel1.getPositionSensor()
encoder2 = wheel2.getPositionSensor()

encoder1.enable(timeStep)
encoder2.enable(timeStep)

s1 = robot.getDevice('ps5') #left
s2 = robot.getDevice('ps7') #front left
s3 = robot.getDevice('ps0') #front right
s4 = robot.getDevice('ps2') #right

s1.enable(timeStep)
s2.enable(timeStep)
s3.enable(timeStep)
s4.enable(timeStep)


def turn90(Cw = True):
    mult = 1
    if not Cw:
        mult = -1
    encoder1_init = encoder1.getValue()
    while abs(encoder1_init - encoder1.getValue()) < 1.80:
        wheel1.setVelocity(mult*3)
        wheel2.setVelocity(-3*mult)
        robot.step(timeStep)

def moveHalfTile():
    RADIUS = 1.5
    PERIMETER = 2 * 3.14159 * RADIUS
    TILE_SIZE = 12
    encoder1_init = encoder1.getValue()
    while abs(encoder1_init - encoder1.getValue()) < 3:
        wheel1.setVelocity(3)
        wheel2.setVelocity(3)
        robot.step(timeStep)

# while robot.step(timeStep) != -1:
#     # follow the left wall
    
#     #1. left is free and front is free -> turn left
#     if s1.getValue() > 0.3 and s2.getValue() > 0.3:
#         print("turning left")
#         moveHalfTile()
#         turn90(Cw = False)
#         moveHalfTile()
#         moveHalfTile()
#     elif s4.getValue() > 0.3 and s3.getValue() > 0.3:
#         print("turning right")
#         moveHalfTile()
#         turn90(Cw = True)
#         moveHalfTile()
#     elif s4.getValue() > 0.3:
#         print("turning right")
#         moveHalfTile()
#         turn90(Cw = True)
#         moveHalfTile()
#     elif s1.getValue() > 0.3:
#         print("turning left")
#         moveHalfTile()
#         turn90(Cw = False)
#         moveHalfTile()
        
#     # check if close to wall and adjust
#     elif s1.getValue() < 0.05:
#         print("adjusting left")
#         wheel1.setVelocity(1)
#         wheel2.setVelocity(2)
#     elif s4.getValue() < 0.05:
#         print("adjusting right")
#         wheel1.setVelocity(2)
#         wheel2.setVelocity(1)
#     else:
#         print("going straight")
#         wheel1.setVelocity(3)
#         wheel2.setVelocity(3)
#     # #2. left is blocked and front is free -> go straight
#     # elif s1.getValue() < 0.2 and s2.getValue() > 0.2:
#     #     updateVelocity(3, 3)

while robot.step(timeStep) != -1:

    moveHalfTile()
    moveHalfTile()
    wheel1.setVelocity(0)
    wheel2.setVelocity(0)
    break
