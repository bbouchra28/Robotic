import numpy as np
import math

# Dimensions du robot [m]
robotRadius = 0.08
wheelRadius = 0.032

# Activer les obstacles
enableObstacles = True

# Mouvement automatique de l'objectif
autoMovingGoal = False

# Param�tres du champ de potentiel
kr = 0.02 #10.0 # repulsive potential gain 0.02
ka = 1.5 #5.0 # attractive potential gain 1.5
pmax = 0.4


def updateWheels(t, robotPos, robotYaw, goalPos, obstaclesPos):
    """Appelée par le simulateur pour mettre à jour les vitesses du robot

    Arguments:
        t {float} -- temps écoulé depuis le début [s]
        robotPos {float[2]} -- position du robot dans le repère monde [m], [m]
        robotYaw {float} -- orientation du robot dans le repère monde [rad]
        goalPos {float[2]} -- position cible du robot dans le repère monde￼

    Returns:
        float[3] -- les vitesses des roues [rad/s]
    """
    p = potentialField(robotPos, goalPos, obstaclesPos);
    return goToPosition(robotPos, robotYaw, goalPos)
    #return [0,0,0]

"""   ********** Matrice de cinématique ***********  """
def kinematicMatrix():

    matrice = np.matrix([[-math.cos(np.pi/6), -math.sin(np.pi/6), robotRadius],
                         [math.cos(np.pi/6), -math.sin(np.pi/6), robotRadius],
                         [0, 1, robotRadius]]) / wheelRadius
    print("Kinematic matrix", matrice)
    return matrice

"""   ********** Matrice de rotation ***********  """
def rotationMatrix(theta):

    matrice = np.matrix([[math.cos(theta), -math.sin(theta), 0],
                         [math.sin(theta), math.cos(theta), 0],
                         [0, 0, 1]])
    print("Rotataion matrix", matrice)
    return matrice

"""   ******** Matrice de translation  *********  """
def translationMatrix(u):

    matrice = np.matrix([[1, 0, u[0]],
                        [0, 1, u[1]],
                        [0, 0, 1]])
    print("Translation matrix", matrice)
    return matrice

"""   ******** Matrice de transformation  *********  """
def transformationMatrix(theta, u):

    matrice = rotationMatrix(theta) * translationMatrix(u)
    print("Transformation matrix", matrice)
    return matrice

""" ********** Move to specific position ***********  """
def goToPosition(robotPos, robotYaw, goalPos):
    distance = math.sqrt(math.pow(goalPos[0] - robotPos[0], 2) + math.pow(goalPos[1] - robotPos[1], 2))
    robotSpeed = 1 / distance
    distanceX = goalPos[0] - robotPos[0]
    distanceY = goalPos[1] - robotPos[1]
    speed = [robotSpeed * distanceX / 3, robotSpeed * distanceY / 3, 0]
    s = np.matrix(speed).T
    return np.asarray(kinematicMatrix() * s).flatten()

""" ************ Get distance between two positions ************ """
def getDistance(p1, p2):
    return math.sqrt(math.pow((p1[0] - p2[0]), 2) + math.pow((p1[1] - p2[1]), 2))

""" ************ Get attractive potential ************ """
def attractivePotential(robotPos, goalPos):
    return 0.5 * ka * math.pow(math.sqrt(math.pow(goalPos[0] - robotPos[0], 2) + math.pow(goalPos[1] - robotPos[1], 2)), 2)

""" ************ Get repulsive potential ************ """
def repulsivePotential(robotPos, obstaclePos):
    maxDistance = math.sqrt(math.pow(obstaclePos[0] - robotPos[0], 2) + math.pow(obstaclePos[1] - robotPos[1], 2))
    if (maxDistance < pmax):
    	return 0.5 * kr / np.pow(maxDistance, 2) 
    return 0

""" ************ Get gradient attractive potential ************ """
def attractivePotentialGradient(robotPos, goalPos):
    print ("attractivePotentialGradient", ka * np.array(robotPos - goalPos))
    return ka * (robotPos - goalPos)

""" ************ Get gradient repulsive potential ************ """
def repulsivePotentialGradient(robotPos, obstaclePos):
    maxDistance = math.sqrt(math.pow(obstaclePos[0] - robotPos[0], 2) + math.pow(obstaclePos[1] - robotPos[1], 2))
    if (maxDistance < pmax):
        print ("repulsivePotentialGradient = ", kr * (obstaclePos - robotPos)/ math.pow(maxDistance, 4))
        return kr * (obstaclePos - robotPos) / math.pow(maxDistance, 4)
    return 0

""" *********** Calculate potential field *********** """
def potentialField(robotPos, goalPos, obstaclesPos):
    attraction = attractivePotentialGradient(robotPos, goalPos)
    repulsion = 0
    for o in obstaclesPos:
        repulsion += repulsivePotentialGradient(robotPos, o)
    print ("potentialField = ", -(repulsion + attraction))
    return -(repulsion + attraction)
