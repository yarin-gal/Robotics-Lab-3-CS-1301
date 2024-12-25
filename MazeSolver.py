from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note
from collections import deque

# robot is the instance of the robot that will allow us to call its methods and to define events with the @event decorator.
robot = Create3(Bluetooth("ARVIN-BOT"))  # Will connect to the first robot found.

# === FLAG VARIABLES
HAS_COLLIDED = False
HAS_ARRIVED = False

# === MAZE DICTIONARY
N_X_CELLS = 3 # Size of maze (x dimension)
N_Y_CELLS = 3 # Size of maze (y dimension)
CELL_DIM = 50


# === DEFINING ORIGIN AND DESTINATION
PREV_CELL = None
START = (0,0)
CURR_CELL = START
DESTINATION = (2,2)

MAZE_DICT = {CURR_CELL: {'position': (CURR_CELL[0]*CELL_DIM, CURR_CELL[1]*CELL_DIM), 'neighbors': [], 'visited': False, 'cost': 0}}

MAZE_DICT[CURR_CELL]["visited"] = True


# === PROXIMITY TOLERANCES
WALL_THRESHOLD = 80

# ==========================================================
# FAIL SAFE MECHANISMS

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

def createMazeDict(nXCells, nYCells, cellDim):

    mazeDict = {}
    
    for i in range(nXCells):
        for j in range(nYCells):

            mazeDict[(i,j)] = {'position': (i*cellDim,j*cellDim), 'neighbors': [], 'visited': False, 'cost': 0}

    return mazeDict

def addAllNeighbors(mazeDict, nXCells, nYCells):

    for item in mazeDict.keys():

        x,y = item 

        if x-1 in range(nXCells):
            mazeDict[item]['neighbors'] += [(x-1, y)]
        if y+1 in range(nYCells):
            mazeDict[item]['neighbors'] += [(x, y+1)]
        if x+1 in range(nXCells):
            mazeDict[item]['neighbors'] += [(x+1, y)]
        if y-1 in range(nYCells):
            mazeDict[item]['neighbors'] += [(x, y-1)]
    
    return mazeDict
        
def getRobotOrientation(heading):

    if heading > 45 and heading <= 135:
        return "N"
    elif heading > 135 and heading <= 225:
        return "W"
    elif heading > 225 and heading <= 315:
        return "S"
    else:
        return "E"

def getPotentialNeighbors(currentCell, orientation):

    x,y = currentCell

    if orientation == "N":
        return [(x-1,y),(x,y+1),(x+1,y),(x,y-1)]
    elif orientation == "W":
        return [(x,y-1),(x-1,y),(x,y+1),(x+1,y)]
    elif orientation == "S":
        return [(x+1,y),(x,y-1),(x-1,y),(x,y+1)]
    else:
        return [(x,y+1),(x+1,y),(x,y-1),(x-1,y)]

def isValidCell(cellIndices, nXCells, nYCells):

    x,y = cellIndices

    if x in range(nXCells) and y in range(nYCells):
        return True
    return False

def getWallConfiguration(IR0, IR3, IR6, threshold):
    
    lDist = 4095/(IR0+1)
    cDist = 4095/(IR3+1)
    rDist = 4095/(IR6+1)

    return [lDist <= threshold, cDist <= threshold, rDist <= threshold]

def getNavigableNeighbors(wallsAroundCell, potentialNeighbors, prevCell, nXCells, nYCells):

    navNeighbors = []

    if not wallsAroundCell[0] and (potentialNeighbors[0][0] in range(nXCells) and potentialNeighbors[0][1] in range(nYCells)):
        navNeighbors += [potentialNeighbors[0]]
    if not wallsAroundCell[1] and (potentialNeighbors[1][0] in range(nXCells) and potentialNeighbors[1][1] in range(nYCells)):
        navNeighbors += [potentialNeighbors[1]]
    if not wallsAroundCell[2] and (potentialNeighbors[2][0] in range(nXCells) and potentialNeighbors[2][1] in range(nYCells)):
        navNeighbors += [potentialNeighbors[2]]      
    if (potentialNeighbors[-1] == prevCell) and (potentialNeighbors[-1][0] in range(nXCells) and potentialNeighbors[-1][1] in range(nYCells)):
        navNeighbors += [potentialNeighbors[-1]]
    elif prevCell == None and (potentialNeighbors[-1][0] in range(nXCells) and potentialNeighbors[-1][1] in range(nYCells)):
        navNeighbors += [potentialNeighbors[-1]]

    return navNeighbors

def updateMazeNeighbors(mazeDict, currentCell, navNeighbors):

    for key in mazeDict:

        neighbors = mazeDict[key]['neighbors']

        if currentCell in neighbors and key not in navNeighbors:

            mazeDict[key]['neighbors'].remove(currentCell)

    mazeDict[currentCell]['neighbors'] = navNeighbors           
            
    return mazeDict
    
def getNextCell(mazeDict, currentCell):

    cells_visited = []
    cells_not_visited = []

    neighbors = mazeDict[currentCell]['neighbors']

    for neighbor in neighbors:
        cost = mazeDict[neighbor]['cost']
        status = mazeDict[neighbor]['visited']

        if status: 
            cells_visited.append((cost, status, neighbor))
        else:
            cells_not_visited.append((cost, status, neighbor))

    cells_visited.sort()
    cells_not_visited.sort()

    if cells_not_visited:
        return cells_not_visited[0][2]
    elif cells_visited:
        return cells_visited[0][2]
    else:
        return None
  
def checkCellArrived(currentCell, destination):
    
    if currentCell == destination:
        return True
    return False

def printMazeGrid(mazeDict, nXCells, nYCells, attribute):
    for y in range(nYCells - 1, -1, -1):
        row = '| '
        for x in range(nXCells):
            cell_value = mazeDict[(x, y)][attribute]
            row += '{} | '.format(cell_value)
        print(row[:-1])

def updateMazeCost(mazeDict, start, goal):
    for (i,j) in mazeDict.keys():
        mazeDict[(i,j)]["flooded"] = False

    queue = deque([goal])
    mazeDict[goal]['cost'] = 0
    mazeDict[goal]['flooded'] = True

    while queue:
        current = queue.popleft()
        current_cost = mazeDict[current]['cost']

        for neighbor in mazeDict[current]['neighbors']:
            if not mazeDict[neighbor]['flooded']:
                mazeDict[neighbor]['flooded'] = True
                mazeDict[neighbor]['cost'] = current_cost + 1
                queue.append(neighbor)

    return mazeDict

# === BUILD MAZE DICTIONARY

MAZE_DICT = createMazeDict(N_X_CELLS, N_Y_CELLS, CELL_DIM)
MAZE_DICT = addAllNeighbors(MAZE_DICT, N_X_CELLS, N_Y_CELLS)

# ==========================================================
# EXPLORATION AND NAVIGATION

# === EXPLORE MAZE
async def navigateToNextCell(robot, nextCell, robotOrientation):
    global MAZE_DICT, PREV_CELL, CURR_CELL, CELL_DIM
    
    dirTup = (nextCell[0]-CURR_CELL[0], nextCell[1]-CURR_CELL[1])

    if dirTup[0] == 0:
        if dirTup[1] == 1:
            robotDir = "N"
        else: 
            robotDir = "S"
    elif dirTup[1] == 0:
        if dirTup[0] == 1:
            robotDir = "E"
        else:
            robotDir = "W"

    angle = 0

    if robotDir == "N":
        if robotOrientation == "S":
            angle = 180
        elif robotOrientation == "E":
            angle = -90
        elif robotOrientation == "W":
            angle = 90
    elif robotDir == "S":
        if robotOrientation == "N":
            angle = 180
        elif robotOrientation == "E":
            angle = 90
        elif robotOrientation == "W":
            angle = -90
    elif robotDir == "E":
        if robotOrientation == "W":
            angle = 180
        elif robotOrientation == "N":
            angle = 90
        elif robotOrientation == "S":
            angle = -90
    elif robotDir == "W":
        if robotOrientation == "E":
            angle = 180
        elif robotOrientation == "N":
            angle = -90
        elif robotOrientation == "S":
            angle = 90
        
    await robot.turn_right(angle)
    await robot.move(CELL_DIM)

    pos = await robot.get_position()
    PREV_CELL = CURR_CELL
    CURR_CELL = ((round(pos.x/CELL_DIM,0)+START[0]), (round(pos.y/CELL_DIM,0)+START[1]))
    MAZE_DICT[CURR_CELL]['visited'] = True

@event(robot.when_play)
async def navigateMaze(robot):
    global HAS_COLLIDED, HAS_ARRIVED
    global PREV_CELL, CURR_CELL, START, DESTINATION
    global MAZE_DICT, N_X_CELLS, N_Y_CELLS, CELL_DIM, WALL_THRESHOLD
    
    while not HAS_COLLIDED:

        pos = await robot.get_position()
        readings = (await robot.get_ir_proximity()).sensors

        HAS_ARRIVED = checkCellArrived(CURR_CELL, DESTINATION)

        if HAS_ARRIVED:
            await robot.set_lights_spin_rgb(0, 255, 0)
            await robot.set_wheel_speeds(0, 0)
            break

        robotOrientation = getRobotOrientation(pos.heading)
        potentialNeighbors = getPotentialNeighbors(CURR_CELL, robotOrientation)
        walls = getWallConfiguration(readings[0], readings[3], readings[6], WALL_THRESHOLD)
        navNeighbors = getNavigableNeighbors(walls, potentialNeighbors, PREV_CELL, N_X_CELLS, N_Y_CELLS)

        MAZE_DICT = updateMazeNeighbors(MAZE_DICT, CURR_CELL, navNeighbors)
        MAZE_DICT = updateMazeCost(MAZE_DICT, START, DESTINATION)

        nextCell = getNextCell(MAZE_DICT, CURR_CELL)

        await navigateToNextCell(robot, nextCell, robotOrientation)
    await robot.set_wheel_speeds(0, 0)

robot.play()
