import math
import numpy as np
'''

Euler chained rotation matrices:

Rz(phi)=[cos(phi) -sin(phi) 0
		 sin(phi) cos(phi)  0
		 0 			0 		1]

Ry(theta)=[cos(theta) 0 -sin(theta)
			0		  1 	0
		   sin(theta) 0 cos(theta)]

Rotation matrix below is Rz*Ry

'''

def rotationmatrix(phi,theta):
	return np.array(
		[[math.cos(phi)*math.cos(theta), -math.sin(phi), -math.cos(phi)*math.sin(theta)],
		[math.sin(phi)*math.cos(theta), math.cos(phi), -math.sin(theta)*math.sin(phi)],
		[math.sin(theta), 0, math.cos(theta)]])

'''
Multiply rotation matrix times unit vectors in each direction.

'''

xhat=np.array([[1],[0],[0]])
yhat=np.array([[0],[1],[0]])
zhat=np.array([[0],[0],[1]])

print(rotationmatrix(0,math.pi/2))

#x unit vector rotations

print(np.dot(rotationmatrix(math.pi/2, 0),xhat))

print(np.dot(rotationmatrix(math.pi/4, math.pi/4),xhat))

print(np.dot(rotationmatrix(0, math.pi/2),xhat))

print(np.dot(rotationmatrix(math.pi/2, math.pi/2),xhat))

print(np.dot(rotationmatrix(0, 0),xhat))

#y unit vector rotations

print(np.dot(rotationmatrix(math.pi/2, 0),yhat))

print(np.dot(rotationmatrix(math.pi/4, math.pi/4),yhat))

print(np.dot(rotationmatrix(0, math.pi/2),yhat))

print(np.dot(rotationmatrix(math.pi/2, math.pi/2),yhat))

print(np.dot(rotationmatrix(0, 0),yhat))

#z unit vector rotations

print(np.dot(rotationmatrix(math.pi/2, 0),zhat))

print(np.dot(rotationmatrix(math.pi/4, math.pi/4),zhat))

print(np.dot(rotationmatrix(0, math.pi/2),zhat))

print(np.dot(rotationmatrix(math.pi/2, math.pi/2),zhat))

print(np.dot(rotationmatrix(0, 0),zhat))



'''
print(np.dot(xhat.transpose(), rotationmatrix(math.pi/2,0)))
print(np.dot(yhat.transpose(), rotationmatrix(math.pi/2,0)))
print(np.dot(zhat.transpose(), rotationmatrix(math.pi/2,0)))
'''


