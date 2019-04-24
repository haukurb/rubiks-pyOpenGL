import numpy as np
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from Quaternion import *
from OpenGL.arrays import vbo

vertices = (
    (1, -1, -1),
    (1, 1, -1),
    (-1, 1, -1),
    (-1, -1, -1),
    (1, -1, 1),
    (1, 1, 1),
    (-1, -1, 1),
    (-1, 1, 1),
)

surfaces = (
    (3, 2, 1),
    (3, 1, 0),
    (6, 7, 2),
    (6, 2, 3),
    (1, 2, 7),
    (1, 7, 5),
    (0, 1, 5),
    (0, 5, 4),
    (4, 5, 7),
    (4, 7, 6),
    (6, 3, 0),
    (6, 0, 4),
)

colors = (
    (1., 0.5, 0.),
    (0., 1., 0.),
    (1., 1., 1.),
    (0., 0., 1.),
    (1., 0., 0.),
    (1., 1., 0.),
)

vertexArray = []
colorArray = []
for surface in surfaces:
    vertexArray += (vertices[vertex] for vertex in surface)
for i in range(len(surfaces)):
    for j in range(len(vertices[0])):
        colorArray.append(colors[i / 2])

vboArray = np.array(map(tuple.__add__, vertexArray, colorArray), "f")


class Cubie:
    def __init__(self, position, cubieSize):
        self.initPos = tuple(position)
        self.pos = np.array(position)
        self.scale = cubieSize
        self.q = np.array([0, 0, 0, 1])
        self.qMatrix = toMatrix(self.q)
        self.degrees = 0
        self.axis = np.array([0, 1, 0])
        self.turnSpeed = 2
        self.eps = 0.2

    def update(self):
        if self.degrees >= self.turnSpeed:
            self.pos = encase3(self.pos, quaternion(self.axis, self.turnSpeed))
            self.q = quatMult(self.q, quaternion(self.axis, -self.turnSpeed))
            self.degrees -= self.turnSpeed
            self.qMatrix = toMatrix(self.q)
        elif self.degrees > 0 and self.turnSpeed > self.degrees:
            self.pos = encase3(self.pos, quaternion(self.axis, self.degrees))
            self.q = quatMult(self.q, quaternion(self.axis, -self.degrees))
            self.degrees = 0
            self.qMatrix = toMatrix(self.q)
        elif self.degrees <= -self.turnSpeed:
            self.pos = encase3(self.pos, quaternion(self.axis, -self.turnSpeed))
            self.q = quatMult(self.q, quaternion(self.axis, self.turnSpeed))
            self.degrees += self.turnSpeed
            self.qMatrix = toMatrix(self.q)
        elif self.degrees < 0 and -self.turnSpeed < self.degrees:
            self.pos = encase3(self.pos, quaternion(self.axis, -self.turnSpeed))
            self.q = quatMult(self.q, quaternion(self.axis, self.turnSpeed))
            self.degrees = 0
            self.qMatrix = toMatrix(self.q)
        pass

    def draw(self):
        glPushMatrix()
        glTranslatef(self.pos[0], self.pos[1], self.pos[2])
        glScalef(self.scale, self.scale, self.scale)
        glMultMatrixf(self.qMatrix)

        glDrawArrays(GL_TRIANGLES, 0, 36)

        glPopMatrix()

    def turn(self, axis, degrees, slice):
        value = np.inner(self.pos, axis) / 2.
        if np.abs(value - slice) < self.eps:
            self.degrees = degrees
            self.axis = axis


class Cube:
    pos = np.array([0, 0, 0])
    q = np.array([0, 0, 0, 1])
    qMatrix = np.array(())
    cubieDict = {}
    cubies = []
    normals = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
    halfWidth = 0
    turnSpeed = 2
    size = 3
    degreeCounter = 0
    pickedPoint = ()
    nowAt = ()
    picked = False

    degreesRotated = 0
    degreesTarget = 0
    sliceSelected = False
    slice = 1
    turnAxis = [0, 1, 0]
    direction = 0
    cubieVBO = vbo.VBO(vboArray)

    def __init__(self, position, size, cubieSize):
        self.size = size
        self.pos = position
        for x in np.arange(0, size) - (size - 1) / 2.:
            for y in np.arange(0, size) - (size - 1) / 2.:
                for z in np.arange(0, size) - (size - 1) / 2.:
                    if (
                        np.abs(x) + 0.1 > (size - 1) / 2.
                        or np.abs(y) + 0.1 > (size - 1) / 2.
                        or np.abs(z) + 0.1 > (size - 1) / 2.
                    ):
                        self.cubieDict[(x, y, z)] = Cubie(
                            [2 * x, 2 * y, 2 * z], cubieSize
                        )
                        self.cubies.append(self.cubieDict[(x, y, z)])
        self.halfWidth = size - 1 + cubieSize
        self.qMatrix = toMatrix(self.q)
        self.setTurnSpeed(self.turnSpeed)

    def update(self):

        value = self.degreesTarget - self.degreesRotated
        if value > self.turnSpeed:
            for cubie in self.cubies:
                cubie.turn(self.turnAxis, self.turnSpeed, self.slice)
            self.degreesRotated += self.turnSpeed
        elif value <= self.turnSpeed and 0 < value:
            for cubie in self.cubies:
                cubie.turn(self.turnAxis, value, self.slice)
            self.degreesRotated += value
        elif value <= -self.turnSpeed:
            for cubie in self.cubies:
                cubie.turn(self.turnAxis, -self.turnSpeed, self.slice)
            self.degreesRotated -= self.turnSpeed
        elif value > -self.turnSpeed and value < 0:
            for cubie in self.cubies:
                cubie.turn(self.turnAxis, -value, self.slice)
            self.degreesRotated -= value

        elif (
            not self.picked
            and not self.sliceSelected
            and self.degreesTarget != 0
            and self.degreesRotated != 0
        ):
            if self.degreesRotated % 90 >= 45:
                self.degreesRotated = self.degreesRotated % 90
                self.degreesTarget = 90
            elif self.degreesRotated % 90 < 45:
                self.degreesRotated = self.degreesRotated % 90
                self.degreesTarget = 0

        for cubie in self.cubies:
            cubie.update()

    def draw(self):
        try:
            self.cubieVBO.bind()

            try:
                glPushMatrix()
                glLoadIdentity()
                glTranslatef(self.pos[0], self.pos[1], self.pos[2])
                glMultMatrixf(self.qMatrix)

                glEnableClientState(GL_VERTEX_ARRAY)
                glEnableClientState(GL_COLOR_ARRAY)
                glVertexPointer(3, GL_FLOAT, 24, self.cubieVBO)
                glColorPointer(3, GL_FLOAT, 24, self.cubieVBO + 12)
                for cubie in self.cubies:
                    cubie.draw()
            finally:
                self.cubieVBO.unbind()
                glDisableClientState(GL_VERTEX_ARRAY)
                glDisableClientState(GL_COLOR_ARRAY)
                glPopMatrix()
        finally:
            pass

    def rotate(self, degrees, axis):
        r = quaternion(np.array(axis), degrees)
        self.normals[0] = encase3(self.normals[0], r)
        self.normals[1] = encase3(self.normals[1], r)
        self.normals[2] = encase3(self.normals[2], r)
        self.q = quatMult(self.q, quaternion(axis, -degrees))
        self.qMatrix = toMatrix(self.q)

    def setTurnSpeed(self, turnSpeed):
        self.turnSpeed = turnSpeed
        for cubie in self.cubies:
            cubie.turnSpeed = turnSpeed

    def setCubieSize(self, cubieSize):
        self.halfWidth = self.size - 1 + cubieSize
        for cubie in self.cubies:
            cubie.size = cubieSize

    def turn(self, normal, degrees, slice):
        if self.degreeCounter == 0:
            self.degreeCounter = degrees
            for cubie in self.cubies:
                cubie.turn(normal, degrees, slice)

    def pick(self, pickRay):
        pickedPoints = {}
        pos = self.pos
        for sign in [1, -1]:
            for i in range(3):
                normal = sign * self.normals[i]
                planePoint = self.pos + normal * self.halfWidth
                distanceToPlane = np.abs(np.inner(normal, planePoint))
                rayxnormal = np.inner(pickRay, normal)
                if not np.isfinite(rayxnormal):
                    continue
                t = -distanceToPlane / rayxnormal
                pickedPoint = t * pickRay

                cubeCoords = [0., 0., 0.]
                r = np.inner((pickedPoint - planePoint), self.normals[(i + 1) % 3])
                s = np.inner((pickedPoint - planePoint), self.normals[(i + 2) % 3])
                cubeCoords[i] = sign * self.halfWidth
                cubeCoords[(i + 1) % 3] = r
                cubeCoords[(i + 2) % 3] = s

                if np.abs(s) < self.halfWidth and np.abs(r) < self.halfWidth:
                    pickedPoints[tuple(pickedPoint.tolist())] = cubeCoords

        closestPoint = ()
        minDist = np.linalg.norm(self.pos)
        dakey = ()
        if len(pickedPoints) > 0:
            minDist = np.linalg.norm(self.pos)
            closestPoint = ()
            for key in pickedPoints:
                dist = np.linalg.norm(key)
                if dist < minDist:
                    minDist = dist
                    closestPoint = pickedPoints[key]
            return closestPoint
        else:
            return ()

    def select(self, pickRay):
        if self.degreesTarget != 0 or self.degreesRotated != 0:
            return
        pickedPoint = self.pick(pickRay)
        if len(pickedPoint) > 0:
            self.pickedPoint = pickedPoint
            self.picked = True
        else:
            self.picked = False

    def dragged(self, pickRay):
        i = np.argmax(np.abs(self.pickedPoint))
        sign = np.sign(self.pickedPoint[i])
        faceNormal = self.normals[i] * sign
        secondNormal = self.normals[(i + 1) % 3]
        thirdNormal = self.normals[(i + 2) % 3]

        planePoint = self.pos + faceNormal * self.halfWidth
        distanceToPlane = np.abs(np.inner(faceNormal, planePoint))
        t = -distanceToPlane / np.inner(pickRay, faceNormal)
        pickedPoint = t * pickRay

        r = np.inner((pickedPoint - planePoint), secondNormal)
        s = np.inner((pickedPoint - planePoint), thirdNormal)
        cubeCoords = [0, 0, 0]
        cubeCoords[i] = sign * self.halfWidth
        cubeCoords[(i + 1) % 3] = r
        cubeCoords[(i + 2) % 3] = s
        self.nowAt = np.array(cubeCoords)
        if self.sliceSelected:
            length = (self.nowAt - self.pickedPoint)[
                np.argmax(np.abs(self.nowAt - self.pickedPoint))
            ]
            self.degreesTarget = (int)(
                np.abs(length / self.halfWidth * 45) * self.direction
            )

    def selectSlice(self):
        u = unitBasisVector(self.nowAt - self.pickedPoint)
        n = unitBasisVector(self.pickedPoint)
        axis = np.cross(n, u)
        axisAbs = np.abs(axis)
        direction = np.inner(axis, axisAbs)
        slice = (int)(
            (np.inner(axisAbs, self.pickedPoint) + self.size) / 2
        ) - self.size / 2
        # virkar fyrir oddatolu
        self.slice = slice
        self.sliceSelected = True
        self.turnAxis = axisAbs
        self.direction = direction

    def deselect(self):
        self.picked = False
        self.sliceSelected = False
        pass
