#Unit vectors in the frisbee body frame

import angle_of_attack as aa
import numpy as np

v_lab=aa.v_lab(vx,vy,vz)
v_plane=aa.v_plane()

def vphat(): #Unit vector in the direction of the plane of the disc
#Defined as the x direction
	return v_plane/np.linalg.norm(v_plane)
	
def xbhat(): #Unit vector in the direction of the disc's velocity
#Defined 
	return v_lab/np.linalg.norm(v_lab)


	
def ybhat(): #Unit vector in the y direction of the frisbee frame
