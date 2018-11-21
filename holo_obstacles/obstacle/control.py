import numpy as np
import math


# Dimensions du robot [m]
robotRadius = 0.08
wheelRadius = 0.032

# Activer les obstacles
enableObstacles = True

# Mouvement automatique de l'objectif
autoMovingGoal = False
goTo = True

# Param�tres du champ de potentiel
kr = 10.0 # repulsive potential gain
ka = 5.0 # attractive potential gain
pmax = 4.0


def updateWheels(t, robotPos, robotYaw, goalPos, obstaclesPos):
    """Appelée par le simulateur pour mettre à jour les vitesses du robot

    Arguments:
        t {float} -- temps écoulé depuis le début [s]
        speed {float[3]} -- les vitesses entrées dans les curseurs [m/s], [m/s], [rad/s]
        robotPos {float[2]} -- position du robot dans le repère monde [m], [m]
        robotYaw {float} -- orientation du robot dans le repère monde [rad]
        goalPos {float[2]} -- position cible du robot dans le repère monde￼

    Returns:
        float[3] -- les vitesses des roues [rad/s]
    """
    p = path(robotPos, goalPos, obstaclesPos);
    print(p)
    """if (goTo or autoMovingGoal):
        return goToPosition(robotPos, robotYaw, goalPos)"""

    #speedVect = np.matrix(speed).T

    #return np.asarray(kinematicMatrix() * speedVect).flatten()
    return [0,0,0]

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
    distance = getDistance(robotPos, goalPos)
    robotSpeed = (1 / distance) / 3
    distanceX = goalPos[0] - robotPos[0]
    distanceY = goalPos[1] - robotPos[1]
    speed = [robotSpeed * distanceX, robotSpeed * distanceY, 0]
    s = np.matrix(speed).T
    return np.asarray(kinematicMatrix() * s).flatten()

""" ************ Get distance between two positions ************ """
def getDistance(p1, p2):
    return math.sqrt(math.pow((p1[0] - p2[0]), 2) + math.pow((p1[1] - p2[1]), 2))

""" ************ Get attractive potential ************ """
def attractivePotential(robotPos, goalPos):
    return (ka * np.hypot(robotPos[0] - goalPos[0], robotPos[1] - goalPos[1]) ** 2) / 2

""" ************ Get repulsive potential ************ """
def repulsivePotential(robotPos, obstaclesPos):
    maxDistance = np.hypot(robotPos[0] - obstaclesPos[0], robotPos[1] - obstaclesPos[1])
    if (maxDistance < pmax):
    	return kr / (maxDistance ** 2) / 2
    return 0

""" ************ Get gradient attractive potential ************ """
def attractivePotentialGradient(robotPos, goalPos):
    return ka * round(np.hypot(robotPos[0] - goalPos[0], robotPos[1] - goalPos[1]))

""" ************ Get gradient repulsive potential ************ """
def repulsivePotentialGradient(robotPos, obstaclesPos):
    maxDistance = np.hypot(robotPos[0] - obstaclesPos[0], robotPos[1] - obstaclesPos[1])
    if (maxDistance < pmax):
    	return kr * (obstaclesPos - robotPos)/ math.pow(maxDistance, 4)
    return 0

""" *********** Calculate potential field *********** """
def potentialField(robotPos, goalPos, obstaclesPos):
    repulsion = [repulsivePotentialGradient(robotPos, o) for o in obstaclesPos]
    attraction = attractivePotentialGradient(robotPos, goalPos)
    return repulsion + attraction

def path(robotPos, goalPos, obstaclesPos):
    s = 0.01
    n = 0.0
    lastPos = robotPos
    #print(np.hypot(robotPos[0] - goalPos[0], robotPos[1] - goalPos[1]))
    seuil = np.hypot(robotPos[0] - goalPos[0], robotPos[1] - goalPos[1])
    p = [robotPos]
    if(seuil > 0.1):
        gradient = np.gradient(potentialField(lastPos, goalPos, obstaclesPos))
        print ("gradient = ", gradient)
        norm = np.linalg.norm(np.gradient(potentialField(lastPos, goalPos, obstaclesPos)))
        print ("norm = ", norm)
        lastPos = s * gradient / -norm
        p.append(lastPos)
    return p
