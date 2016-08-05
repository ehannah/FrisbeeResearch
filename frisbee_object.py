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

  #Below are unit important unit vectors which we will ultimately use to orient the forces
  #and torques in the correct directions.

  #Calculate unit vector in the direction of the disc's velocity (in lab frame).
  def vhat(self):
    return self.velocity/np.linalg.norm(self.velocity)

  #Calculate unit vector in z-body direction. Corresponds to 3rd column of rotation matrix. 
  def zbhat(self):
    return np.transpose(self.rotationmatrix()[2])

  #Calculate unit vector in x-body direction. Corresponds to unit vector in the direction
  #of the plane of the disc (distinct from the velocity of the disc)
  def xbhat(self):
    zcomponent=np.dot(self.velocity,self.zbhat())
    return self.velocity-self.zbhat()*zcomponent

  def ybhat(self):
    return np.cross(self.zbhat(),self.xbhat())

  #Calculate angle of attack, defined as angle between plane of disc and velocity vector
  #of the frisbee's motion. First step is to calculate scalar component of the velocity 
  #that is not in the plane of the disc (dot product of z-body unit vector and velocity
  #vector). We then subtract the scalar component calculated above times the z-body unit
  #vector itself from the velocity vector. This gives us the velocity in the plane of the
  #disc. The angle of attack is the angle formed between the z-component and the velocity
  #in the plane of the disc, which we calculate using an arctan function.
  def attackangle(self):
    zcomponent=np.dot(self.velocity,self.zbhat())
    v_plane=self.velocity-zbhat*zcomponent
    return math.atan(zcomponent/np.linalg.norm(v_plane))
