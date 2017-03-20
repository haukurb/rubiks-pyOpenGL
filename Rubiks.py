import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import numpy as np
import time
import os
import sys

from Cube import *
from Quaternion import *




def getPickRay(dimensions, mouseCoords):
	projMat = np.matrix( glGetFloatv(GL_PROJECTION_MATRIX) )	
	modelViewMat = np.matrix( glGetFloatv(GL_MODELVIEW_MATRIX) )
	x = 2*mouseCoords[0]/(1.0*dimensions[0])-1
	y = 1 - 2*mouseCoords[1]/(1.0*dimensions[1])
	
	clipCoords = np.array([ x, y, -1, 1 ])
	
	eyeCoords = np.dot( np.asarray(projMat.I) , clipCoords)
	eyeCoords[2] = -1.
	eyeCoords[3] = 0.
	eyeCoords /= np.linalg.norm(eyeCoords)
	
	worldCoords = np.dot(modelViewMat.I, eyeCoords)[0:3]
	worldCoords /= np.linalg.norm(worldCoords)
	return np.asarray(worldCoords)[0][0:3]
	
def getPickedPoint(cube, sign, normalIndex, _pickRay):
	pos = np.matrix(cube.pos[0:3])
	halfWidth = cube.halfWidth
	faceNormal = np.matrix(cube.normals[normalIndex][0:3])*sign
	secondNormal = np.matrix(cube.normals[ (normalIndex +1)%3 ][0:3])
	thirdNormal = np.matrix(cube.normals[ (normalIndex+2)%3 ][0:3])
	pickRay = np.matrix(_pickRay).T
	
	planePoint = pos + faceNormal * halfWidth
	distanceToPlane = np.abs( faceNormal * planePoint.T )
	t = -distanceToPlane / (pickRay * faceNormal.T)
	pickedPoint = t * pickRay
	
	r = (pickedPoint - planePoint)*secondNormal.T
	s = (pickedPoint - planePoint)*thirdNormal.T
	cubeCoords = [0,0,0]
	cubeCoords[normalIndex] = sign
	cubeCoords[(normalIndex+1)%3] = r
	cubeCoords[(normalIndex+2)%3] = s
	return cubeCoords

def mouseDownEvent(dimensions, mousePos, cube):
	pickRay = getPickRay(dimensions, mousePos)
	pickedPoints = []
	for sign in [1,-1]:
		for i in [0,1,2]:
			point = getPickedPoint(cube, sign, i, pickRay)
			if point is not None:
				pickedPoints.append( [point, i, sign] )
	if len(pickedPoints) == 0:
		return []
	minDistance = np.linalg.norm( pickedPoints[0][0][0] )
	myPoint = []
	for point in pickedPoints:
		dist = np.linalg.norm( point[0][0] )
		if dist <= minDistance:
			minDistance = dist
			myPoint = point
	r = (myPoint[0][1][0]).item()
	s = (myPoint[0][1][1]).item()
	index = myPoint[1]
	sign = myPoint[2]
	cubeCoords = [[0]]*3
	cubeCoords[index] = sign*2.96
	cubeCoords[(index+1)%3] = r
	cubeCoords[(index+2)%3] = s
	return cubeCoords, sign, index

class Queue:
	def __init__(self, size):
		self.data = []
		self.size = size

	def push(self, item):
		if(len(self.data) < self.size):
			self.data.append(item)
		else:
			data = self.data[1:]
			data.append(item)
			self.data = data

	def average(self):
		if(len(self.data) != 0):
			return sum(self.data)*1.0/len(self.data)
		else:
			return 0

def main():
	pygame.init()
	clock = pygame.time.Clock()
	
	os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (100,30)
	
	dimensions = (800,600)
	screen = pygame.display.set_mode(dimensions, pygame.DOUBLEBUF | pygame.OPENGL)
	glEnable(GL_MULTISAMPLE)
	glEnable(GL_DEPTH_TEST)
	glCullFace(GL_BACK)
	glEnable(GL_CULL_FACE)
	
	glMatrixMode(GL_PROJECTION) #
	glLoadIdentity()
	gluPerspective(90, dimensions[0]/dimensions[1], 0.1, 50.0)

	glMatrixMode(GL_MODELVIEW) #
	glLoadIdentity()
	
	cubeDim = 3
	if(len(sys.argv)>1):
		cubeDim = int(sys.argv[1])
	mycube = Cube(np.array([0,0,-16]), cubeDim, 0.96)
	# mycube.setTurnSpeed(1.7)
	
	accumMouseRel = np.array([0,0])
	
	lastCubeCoords = []
	lastScreenCoords = []
	
	counter = 0
	whileQueue = Queue(60)
	fpsQueue = Queue(60)
	upsQueue = Queue(60)
	# Should use while Running: 
	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				quit()
			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_ESCAPE:
					pygame.quit()
					quit()
				if event.key == pygame.K_UP:
					pass
				if event.key == pygame.K_DOWN:
					pass
				if event.key == pygame.K_0:
					glLoadIdentity()
					gluLookAt(0,0,13,  0,0,0,  0,1,0)
					
			if event.type == pygame.MOUSEBUTTONDOWN and event.button == 4:
			## scroll up
				mycube.pos[2] += 1.
			if event.type == pygame.MOUSEBUTTONDOWN and event.button == 5:
			## scroll down
				mycube.pos[2] -= 1.
			
			if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
			#### left mouse clicked down
				pygame.mouse.get_rel()
				accumMouseRel[0:2] = 0
				mycube.select(getPickRay(dimensions, pygame.mouse.get_pos()))
				pass
				
			if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
			#### left mouse released
				mycube.deselect()
				pass

		keys = pygame.key.get_pressed()
		if keys[pygame.K_UP]:
			mycube.rotate(1,[1,0,0])
			pass
		if keys[pygame.K_DOWN]:
			mycube.rotate(1,[-1,0,0])
		if keys[pygame.K_LEFT]:
			mycube.rotate(1,[0,1,0])
			pass
		if keys[pygame.K_RIGHT]:
			mycube.rotate(1,[0,-1,0])
			pass
		if keys[pygame.K_d]:
			mycube.rotate(1,[0,-1,0])
			pass
		if keys[pygame.K_a]:
			mycube.rotate(1,[0,1,0])
			pass
		if keys[pygame.K_w]:
			mycube.rotate(1,[1,0,0])
			pass
		if keys[pygame.K_s]:
			mycube.rotate(1,[-1,0,0])
		if keys[pygame.K_q]:
			mycube.rotate(1,[0,0,1])
			pass
		if keys[pygame.K_e]:
			mycube.rotate(1,[0,0,-1])
			pass
		if pygame.mouse.get_pressed()[0]:
			if not mycube.picked:
				accumMouseRel += pygame.mouse.get_rel()
				if np.abs(accumMouseRel[0]) > 2:
					mycube.rotate(accumMouseRel[0]/5.,[0,1,0])
					accumMouseRel[0] = 0
				if np.abs(accumMouseRel[1]) > 2:
					mycube.rotate(accumMouseRel[1]/5.,[1,0,0])
					accumMouseRel[1] = 0
			if mycube.picked and not mycube.sliceSelected:
				accumMouseRel += pygame.mouse.get_rel()
				if np.linalg.norm(accumMouseRel) > 15:
					mycube.dragged(getPickRay(dimensions, pygame.mouse.get_pos()))
					mycube.selectSlice()
					accumMouseRel[0:2] = 0
			elif mycube.picked and mycube.sliceSelected:
				mycube.dragged(getPickRay(dimensions, pygame.mouse.get_pos()))
		
		updateBefore = time.time()
		mycube.update()
		updateAfter = time.time()
		upsQueue.push(updateAfter - updateBefore)

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		mycube.draw()
		drawAfter = time.time()
		fpsQueue.push(drawAfter - updateAfter)
		
		pygame.display.flip()
		clock.tick()
		counter += 1
		# print clock.get_fps()

		# whileQueue.push(time.time()-whileBefore)
		# pygame.time.wait(10)
		# print "update : ", upsQueue.average()*1000.0, " ms"
		if counter%10 == 0:
			print "frame  : ", fpsQueue.average()*1000.0, " ms"
		# print whileQueue.average()*1000.0, " ms for while loop"

if len(sys.argv) > 1:
	if int(sys.argv[1]) % 2 == 0:
		raise ValueError("Sorry, even numbers are broken")
main()
		
		