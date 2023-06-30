import math
from xml.dom import minidom
import os
import random

class main_body:
    def __new__(cls, size):
        global bodySize
        bodySize = size

        defaultMainBody = root.createElement('default')
        defaultBody.appendChild(defaultMainBody)
        defaultMainBody.setAttribute('class', 'body_main')

        bodyGeom = root.createElement('geom')
        defaultMainBody.appendChild(bodyGeom)
        bodyGeom.setAttribute('type', 'sphere')
        bodyGeom.setAttribute('size', str(size))
        bodyGeom.setAttribute('rgba', '0 .84 .91 1')

        mainBody = root.createElement('body')
        worldbody.appendChild(mainBody)
        mainBody.setAttribute('name', 'body_main')
        mainBody.setAttribute('pos', '0 0 .1')
        mainBody.setAttribute('childclass', 'body')

        bodySite = root.createElement('site')
        mainBody.appendChild(bodySite)
        bodySite.setAttribute('name', 'body')

        bodySensor = root.createElement('framepos')
        sensors.appendChild(bodySensor)
        bodySensor.setAttribute('objtype', 'site')
        bodySensor.setAttribute('objname', 'body')
        bodyRotSensor = root.createElement('framequat')
        sensors.appendChild(bodyRotSensor)
        bodyRotSensor.setAttribute('objtype', 'site')
        bodyRotSensor.setAttribute('objname', 'body')

        joint = root.createElement('freejoint')
        mainBody.appendChild(joint)
        joint.setAttribute('name', 'root')

        mainBodyGeom = root.createElement('geom')
        mainBody.appendChild(mainBodyGeom)
        mainBodyGeom.setAttribute('name', 'body_main')
        mainBodyGeom.setAttribute('class', 'body_main')

        return mainBody

class secondary_bodies:
    def __new__(cls, lastBody, size, n, ndegree, j, jdegree, color, nameNums):
        bodyname='secondaryBody'+str(nameNums)
        jointname = 'BJ'+str(nameNums)
        bodyX = (n)*(math.sin(math.radians(ndegree)))
        bodyY = (n)*(math.cos(math.radians(ndegree)))
        bodyPos = str(bodyX)+' '+str(bodyY)+' 0'
        jointX = (j)*(math.sin(math.radians(jdegree)))
        jointY = (j)*(math.cos(math.radians(jdegree)))
        jointPos = str(jointX)+' '+str(jointY)+' 0'

        secondaryBody = root.createElement('body')
        lastBody.appendChild(secondaryBody)
        secondaryBody.setAttribute('name', bodyname)

        secondaryBodyGeom = root.createElement('geom')
        secondaryBody.appendChild(secondaryBodyGeom)
        secondaryBodyGeom.setAttribute('name', bodyname)
        secondaryBodyGeom.setAttribute('pos', bodyPos)
        secondaryBodyGeom.setAttribute('class', 'body_main')
        secondaryBodyGeom.setAttribute('rgba', color)

        secondaryBodyJoint = root.createElement('joint')
        secondaryBody.appendChild(secondaryBodyJoint)
        secondaryBodyJoint.setAttribute('name', jointname)
        secondaryBodyJoint.setAttribute('range', '-60 60')
        secondaryBodyJoint.setAttribute('pos', jointPos)

        secondaryBodyMotor = root.createElement('motor')
        actuators.appendChild(secondaryBodyMotor)
        secondaryBodyMotor.setAttribute('name', jointname)
        secondaryBodyMotor.setAttribute('gear', '30')
        secondaryBodyMotor.setAttribute('joint', jointname)

        return secondaryBody

class legs:
    def __init__(self, secondaryBody, originalSize, number, degree, n, ndegree, color, nameNums): #n=number in line
        previousLeg = secondaryBody
        length = originalSize
        totalLength = 0
        for i in range(number):
            legName = 'Leg '+str(nameNums)+'-'+str(i)
            jointName ='LJ'+str(nameNums)+'-'+str(i)
            jointRange = '0'+' '+ str(100/number)
            jointEnd = str(100/number)
            color = str(abs(math.sin((i+n*.5)/5)))+' '+str(abs(math.sin((((i+n*.5)/5)+1))))+' '+str(abs(math.sin((((i+n*.5)/5)+2))))+' 1'

            if i == 0:
                legSensor = root.createElement('jointpos')
                sensors.appendChild(legSensor)
                legSensor.setAttribute('joint', jointName)

            jointX = (totalLength+originalSize)*(math.sin(math.radians(degree)))+(n)*(math.sin(math.radians(ndegree)))
            jointY = (totalLength+originalSize)*(math.cos(math.radians(degree)))+(n)*(math.cos(math.radians(ndegree)))
            jointPos = str(jointX)+' '+str(jointY)+' 0'
            jointAxis = str(-1*math.cos(math.radians(degree)))+' '+str(math.sin(math.radians(degree)))+' 0'

            legTurn = '90 '+str(-degree)+' 0'
            
            capsuleSize = str(length*.3)+' '+str(length*.5)

            legX = (totalLength+length*.5+originalSize)*(math.sin(math.radians(degree)))+(n)*(math.sin(math.radians(ndegree)))
            legY = (totalLength+length*.5+originalSize)*(math.cos(math.radians(degree)))+(n)*(math.cos(math.radians(ndegree)))
            legPos = str(legX)+' '+str(legY)+' 0'

            leg = root.createElement('body')
            previousLeg.appendChild(leg)
            leg.setAttribute('name', legName)

            legGeom = root.createElement('geom')
            leg.appendChild(legGeom)
            legGeom.setAttribute('pos', legPos)
            legGeom.setAttribute('size', capsuleSize)
            legGeom.setAttribute('euler', legTurn)
            legGeom.setAttribute('rgba', color)

            legJoint = root.createElement('joint')
            leg.appendChild(legJoint)
            legJoint.setAttribute('name', jointName)
            legJoint.setAttribute('range', jointRange)
            legJoint.setAttribute('axis', jointAxis)
            legJoint.setAttribute('pos', jointPos)
            legJoint.setAttribute('springref', jointEnd)

            if i == 0:
                legMotor = root.createElement('motor')
                actuators.appendChild(legMotor)
                legMotor.setAttribute('name', jointName)
                legMotor.setAttribute('gear', '30')
                legMotor.setAttribute('joint', jointName)
            else:
                legJoint.setAttribute('stiffness', '15')

            totalLength += length
            length = length*.8
            previousLeg = leg

name='Organism'

root = minidom.Document()

mujoco = root.createElement('mujoco')
root.appendChild(mujoco)
mujoco.setAttribute('model', 'test')

visual = root.createElement('visual')
mujoco.appendChild(visual)

map = root.createElement('map')
visual.appendChild(map)
map.setAttribute('force', '0.1')
map.setAttribute('zfar', '30')
rgba = root.createElement('rgba')
visual.appendChild(rgba)
rgba.setAttribute('haze', '0.15 0.25 0.35 1')
globe = root.createElement('global')
visual.appendChild(globe)
globe.setAttribute('offwidth', '2560')
globe.setAttribute('offheight', '1440')
globe.setAttribute('elevation', '-20')
globe.setAttribute('azimuth', '120')

center = root.createElement('statistic')
mujoco.appendChild(center)
center.setAttribute('center', '0 0 0.7')

default = root.createElement('default')
mujoco.appendChild(default)

defaultBody = root.createElement('default')
default.appendChild(defaultBody)
defaultBody.setAttribute('class', 'body')

defaultGeom = root.createElement('geom')
defaultBody.appendChild(defaultGeom)
defaultGeom.setAttribute('type', 'capsule')
defaultGeom.setAttribute('condim', '1')
defaultGeom.setAttribute('friction', '.7')
defaultGeom.setAttribute('solimp', '.9 .99 .003')
defaultGeom.setAttribute('solref', '.015 1')

defaultJoints = root.createElement('joint')
defaultBody.appendChild(defaultJoints)
defaultJoints.setAttribute('type', 'hinge')
defaultJoints.setAttribute('damping', '.2')
defaultJoints.setAttribute('stiffness', '1')
defaultJoints.setAttribute('armature', '.01')
defaultJoints.setAttribute('limited', 'true')
defaultJoints.setAttribute('solimplimit', '0 .99 .01')

worldbody = root.createElement('worldbody')
mujoco.appendChild(worldbody)

light = root.createElement('light')
worldbody.appendChild(light)
light.setAttribute('diffuse', '.5 .5 .5')
light.setAttribute('pos', '0 0 10')
light.setAttribute('dir', '0 0 -1')

plane = root.createElement('geom')
worldbody.appendChild(plane)
plane.setAttribute('type', 'plane')
plane.setAttribute('size', '40 40 0.1')
plane.setAttribute('rgba', '1 1 1 1')

actuators = root.createElement('actuator')
mujoco.appendChild(actuators)

sensors = root.createElement('sensor')
mujoco.appendChild(sensors)

creatureList = [1, 2, 1, 2, 1, 2, 0] #Mainbody (always 1), legs on main body, secondary body number, legs on secondary body, tertiary body number, ect.
bodySize = 0.09
legLength = 4

mainBody = main_body(bodySize)
previousBodies = [mainBody]
previousBodyDistanceVectors = [[0, 0]]
for i in range((int((len(creatureList)-1)/2))):

    color = str(abs(math.sin((i+1)/10)))+' '+str(abs(math.sin((((i+1)/10)+1))))+' '+str(abs(math.sin((((i+1)/10)+2))))+' 1'

    numLegs = creatureList[(i*2+1)]
    numBodies = creatureList[(i*2+2)]

    #Checks to see if it's the first time, so it doesn't need to think about an already existing connection
    if previousBodies == [mainBody]:
        firstStep = 0
        appendeturesPositions = []
        previousDegBodies = [0]
    else:
        firstStep = 1
        appendeturesPositions = [0]
    total = numLegs+numBodies+firstStep

    #Checks whether it's easy to divide out where bodies go vs legs, and evenly distributes legs and bodies is so. Otherwise, it's random.
    #0: Prev Joint, 1: Leg, 2: Body
    if numBodies==0:
        for t in range(total-len(appendeturesPositions)):
            appendeturesPositions.append(1)
    elif total%numBodies==0:
        print('here')
        for t in range(total-len(appendeturesPositions)):
            print(t%(total/numBodies))
            if t%(total/numBodies) == 1:
                appendeturesPositions.append(2)
            else:
                appendeturesPositions.append(1)
    else:
        bodyPositions = []
        bodyPos = random.randint(0, total-2)
        for t in range(numBodies):
            while bodyPos in bodyPositions:
                bodyPos = random.randint(0, total-2)
            bodyPositions.append(bodyPos)
        for t in range(total-len(appendeturesPositions)):
            if t in bodyPositions:
                appendeturesPositions.append(2)
            else:
                appendeturesPositions.append(1)

    print(appendeturesPositions)
    #assigns degree values for each leg and body
    degSteps = 360/total
    degLegs = []
    degBodies = []
    previous2DegBodies = []
    for t in range(len(appendeturesPositions)):
        if appendeturesPositions[t] == 1:
            degLegs.append(degSteps*t+180)
        elif appendeturesPositions[t] == 2:
            degBodies.append(degSteps*t+180)

    currentBodies = []
    currentBodyDistanceVectors = []

    #initiates loop over all of the previous bodies
    for x in range(len(previousBodies)):
        #loop creating legs
        for n in range(numLegs):
            nameNum = str(i)+'-'+str(n+x*numLegs)
            legs(previousBodies[x], bodySize, legLength, degLegs[n]+previousDegBodies[x], previousBodyDistanceVectors[x][0], previousBodyDistanceVectors[x][1], color, nameNum)

        for n in range(numBodies):
            nameNum = str(i)+'-'+str(n+x*numBodies)
            #finds vector to the new body
            d1 = previousBodyDistanceVectors[x][0]
            t1 = math.radians(previousBodyDistanceVectors[x][1])
            d2 = bodySize*2
            t2 = math.radians(degBodies[n]+previousDegBodies[x])
            currentBodyDistanceVectorMagnitude = math.sqrt(d1**2+d2**2+2*d1*d2*math.cos(t2-t1))
            currentBodyDistanceVectorAngle = math.degrees(t1 + math.atan2(d2*math.sin(t2-t1), d1+d2*math.cos(t2-t1)))
            currentBodyDistanceVector = [currentBodyDistanceVectorMagnitude, currentBodyDistanceVectorAngle]
            currentBodyDistanceVectors.append(currentBodyDistanceVector)
            currentBody = secondary_bodies(previousBodies[x], bodySize, currentBodyDistanceVector[0], currentBodyDistanceVector[1], previousBodyDistanceVectors[x][0], previousBodyDistanceVectors[x][1], color, nameNum)
            currentBodies.append(currentBody)
            previous2DegBodies.append(degBodies[n]+previousDegBodies[x])
    previousBodies = currentBodies
    previousBodyDistanceVectors = currentBodyDistanceVectors
    previousDegBodies = previous2DegBodies

xml_str = root.toprettyxml(indent='\t')

save_path_file = 'currentCreature.xml'

with open(save_path_file, 'w') as f:
    f.write(xml_str)