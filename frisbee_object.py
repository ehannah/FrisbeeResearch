import numpy as np
import math

#Create a Frisbee class, and assign Frisbee self values that correspond 
#to physical characteristics of the frisbee.
class Frisbee(object):
  def __init__(self,x,y,z,vx,vy,vz,wx,wy,wz,phi,theta,gamma):
    self.x=x
    self.y=y
    self.z=z
    self.vx=vx
    self.vy=vy
    self.vz=vz
    self.wx=wx
    self.wy=wy
    self.wz=wz
    self.phi=phi
    self.theta=theta
    self.gamma=gamma
    self.velocity=np.array([vx,vy,vz])
    
  #Represent Frisbee object by printing instantaneous position and velocity.
  def __str__(self):
    return 'Position: (' + str(self.x)+','+str(self.y)+','+str(self.z)+')'+'\n'+'Velocity: ('+str(self.vx)+','+str(self.vy)+','+str(self.vz)+')'

  #Calculate rotation matrix. Rotation matrix is the product Rz*Ry of "Euler Chained Rotation
  #Matrices", found at https://en.wikipedia.org/wiki/Davenport_chained_rotations. 
  def rotationmatrix(self):
    return np.array(
  		[[math.cos(self.phi)*math.cos(self.theta), -math.sin(self.phi), -math.cos(self.phi)*math.sin(self.theta)],
  		[math.sin(self.phi)*math.cos(self.theta), math.cos(self.phi), -math.sin(self.theta)*math.sin(self.phi)],
  		[math.sin(self.theta), 0, math.cos(self.theta)]])
  	
  #Calculate angle of attack, defined as angle between plane of disc and velocity vector
  #of the frisbee's motion. First step is to calculate scalar component of the velocity 
  #that is not in the plane of the disc (dot product of z-body unit vector and velocity
  #vector). We then subtract the scalar component calculated above times the z-body unit
  #vector itself from the velocity vector. This gives us the velocity in the plane of the
  #disc. The angle of attack is the angle formed between the z-component and the velocity
  #in the plane of the disc, which we calculate using an arctan function.
  def attackangle(self):
    zbhat=np.transpose(self.rotationmatrix()[2])
    zcomponent=np.dot(self.velocity,zbhat)
    v_plane=self.velocity-zbhat*zcomponent
    return math.atan(zcomponent/np.linalg.norm(v_plane))

