'''
The angle of attack is defined as the angle between the velocity of the disc
and the plane of the frisbee.

To calculate the angle of attack:

1. Create lab-frame velocity vector ([vx,vy,vz])
2. Determine which column or row of rotation matrix represents inertial coordinates
of z-body unit vector
3. Calculate dot product of z-body unit vector and velocity vector; this dot product
is the scalar component of the velocity that is not in the plane of the disc
4. Calculate the velocity in the plane of the disc by subtracting the above dot product times
the z-body unit vector from the lab-frame velocity.

'''
import numpy as np
import rotationmatrix as rm

#Inertial coordinates of unit vectors correspond to columns of rotation matrix
#Unit vectors are [1,0,0], [0,1,0], and [0,0,1], respectively

def xbhat(phi, theta):
	return np.rm.rotationmatrix(phi,theta).transpose()[0]

def ybhat(phi, theta):
	return np.rm.rotationmatrix(phi,theta).transpose()[1]

def zbhat(phi, theta)
	return np.rm.rotationmatrix(phi,theta).transpose()[2]
	
def v_lab(vx,vy,vz):
	return np.array([vx,vy,vz])
	
def v_plane():
	z_component=np.dot(zhat,v_lab(vx,vy,vz))
	return v_lab-z_component*zhat
	
def angle_of_attack():
	return math.atan(z_component/np.linalg.norm(v_plane)) #Not sure what the sign of arctan function should be
