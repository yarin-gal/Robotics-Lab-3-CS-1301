from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

import math as m

# robot is the instance of the robot that will allow us to call its methods and to define events with the @event decorator.
robot = Create3(Bluetooth("HRISTO-BOT"))  # Will connect to the first robot found.

HAS_COLLIDED = False
HAS_REALIGNED = False
HAS_FOUND_OBSTACLE = False
SENSOR2CHECK = 0
HAS_ARRIVED = False
DESTINATION = (0, 100)
ARRIVAL_THRESHOLD = 5
IR_ANGLES = [-65.3, -38.0, -20.0, -3.0, 14.25, 34.0, 65.3]

# Implementation for fail-safe robots
# EITHER BUTTON
@event(robot.when_touched, [True, True])  # User buttons: [(.), (..)]
async def when_either_button_touched(robot):
    
    global HAS_COLLIDED
    HAS_COLLIDED = True

    await robot.set_wheel_speeds(0, 0)
    await robot.set_lights_rgb(255, 0, 0)
    

# EITHER BUMPER
@event(robot.when_bumped, [True, True])  # [left, right]
async def when_either_bumped(robot):
    
    global HAS_COLLIDED
    HAS_COLLIDED = True

    await robot.set_wheel_speeds(0, 0)
    await robot.set_lights_rgb(255, 0, 0)
    

# ==========================================================

# Helper Functions
def getMinProxApproachAngle(readings):
    IR_ANGLES = [-65.3, -38.0, -20.0, -3.0, 14.25, 34.0, 65.3]
    min_distance = float('inf')
    for elements in readings:        
        proximity = 4095/(elements+1)
        if proximity <= min_distance:
            index = readings.index(elements)
            min_distance = proximity
            angle = IR_ANGLES[index]
    return (round(min_distance, 3), angle)

def getCorrectionAngle(heading):
    correctionAngle = None
    if heading > 90:
        correctionAngle = int(heading-90)
    else:
        correctionAngle = int(-(90-heading))
    return correctionAngle


def getAngleToDestination(currentPosition, destination):
    position = (currentPosition.x, currentPosition.y)
    x1 = position[0]
    y1 = position[1]

    x2 = destination[0]
    y2 = destination[1]

    x = int(x2 - x1)
    y = int(y2 - y1)
    angle = m.degrees(m.atan2(y, x))
    angleToDestination = angle - 90
    angleToDestination = -((angleToDestination + 180) % 360 - 180)

    return int(angleToDestination)


def checkPositionArrived(currentPosition, destination, threshold):
    position = (currentPosition.x, currentPosition.y)
    
    x1 = position[0]
    y1 = position[1]
    x2 = destination[0]
    y2 = destination[1]

    distance = m.sqrt(m.pow(x2 - x1, 2) + m.pow(y2 - y1, 2))

    return distance <= threshold


# === REALIGNMENT BEHAVIOR
async def realignRobot(robot):
    global DESTINATION, HAS_REALIGNED
    
    pos = await robot.get_position()
    
    await robot.set_wheel_speeds(0, 0)
    x, y, heading = pos.x, pos.y, pos.heading
    angle = getAngleToDestination(pos, DESTINATION)
    correction = getCorrectionAngle(heading)
    
    
    await robot.turn_right(correction)
    await robot.turn_right(angle)

    HAS_REALIGNED = True

# === MOVE TO GOAL
async def moveTowardGoal(robot):
    global SENSOR2CHECK, DESTINATION, ARRIVAL_THRESHOLD, HAS_FOUND_OBSTACLE
    
    position = await robot.get_position()
    readings = (await robot.get_ir_proximity()).sensors                                     #get sensor readings repeatedly
    minDistance, minAngle = getMinProxApproachAngle(readings)                               #get the tuple from the helper function

    
    
    if minDistance < 20.0:
        await robot.set_wheel_speeds(0, 0)
        
        if minAngle > 0:
            await robot.turn_left(90-minAngle)
            SENSOR2CHECK = -1
        else:
            await robot.turn_right(90+minAngle)
            SENSOR2CHECK = 0

        HAS_FOUND_OBSTACLE = True
    else:
        await robot.set_wheel_speeds(10, 10)

# === FOLLOW OBSTACLE
async def followObstacle(robot):
    global SENSOR2CHECK, HAS_REALIGNED, HAS_FOUND_OBSTACLE
    proxReading = (await robot.get_ir_proximity()).sensors
    proxReadingSens = 4095/(proxReading[SENSOR2CHECK]+1)

    if proxReadingSens < 20.0:
        if SENSOR2CHECK == -1:
            await robot.turn_left(3)
        else:
            await robot.turn_right(3)
                
    elif proxReadingSens < 100.0:
        await robot.set_wheel_speeds(10, 10)
    else:
        await robot.set_wheel_speeds(0, 0)
        await robot.move(30) 
        HAS_REALIGNED = False
        HAS_FOUND_OBSTACLE = False
            

# ==========================================================

# Main function

@event(robot.when_play)
async def makeDelivery(robot):
    global HAS_ARRIVED, HAS_COLLIDED, HAS_FOUND_OBSTACLE, HAS_REALIGNED
    global DESTINATION, ARRIVAL_THRESHOLD
    await robot.reset_navigation()
    
    while not HAS_COLLIDED:
        pos = await robot.get_position()

        if checkPositionArrived(pos, DESTINATION, ARRIVAL_THRESHOLD):
            
            HAS_ARRIVED = True
            await robot.set_wheel_speeds(0, 0)
            await robot.set_lights_rgb(0, 255, 0)
            break

        if not HAS_REALIGNED:
            await realignRobot(robot)
            print("after realign:")
            print(DESTINATION)
            
        if HAS_FOUND_OBSTACLE:
            await followObstacle(robot)
            print("After follow obstacle:")
            print(DESTINATION)
            
        if not HAS_ARRIVED and not HAS_FOUND_OBSTACLE:
            await moveTowardGoal(robot)

        
            

# start the robot
robot.play()
