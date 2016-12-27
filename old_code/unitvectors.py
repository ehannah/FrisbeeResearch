#Unit vectors in the frisbee body frame

import angle_of_attack as aa
import numpy as np
import rotationmatrix as rm

v_lab=aa.v_lab(vx,vy,vz)
v_plane=aa.v_plane()
zbhat=rm.rotationmatrix(phi,theta).transpose()[2]
	
def vhat(): #Unit vector in the direction of the disc's velocity
	return v_lab/np.linalg.norm(v_lab)

def xbhat(): #Unit vector in the direction of the plane of the disc
#Defined as the x body unit vector
	return v_plane/np.linalg.norm(v_plane)

def ybhat(): #Unit vector in the y direction of the frisbee frame
#Calculated as cross product of z-body unit vector from rotation matrix
#and x-body unit vector as defined above
	return np.cross(zbhat,xbhat())

def zbhat(): #Unit vetor in z-body direction
#Taken from inertial coordinates of rotation matrix
	return zbhat()
