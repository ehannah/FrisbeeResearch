import numpy as np
import math

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
    
  def __str__(self):
    return 'Position: (' + str(self.x)+','+str(self.y)+','+str(self.z)+')'+'\n'
    +'Velocity: ('+str(self.vx)+','+str(self.vy)+','+str(self.vz)+')'

  #Calculate rotation matrix.
  def rotationmatrix(self):
    return np.array(
  		[[math.cos(self.phi)*math.cos(self.theta), -math.sin(self.phi), -math.cos(self.phi)*math.sin(self.theta)],
  		[math.sin(self.phi)*math.cos(self.theta), math.cos(self.phi), -math.sin(self.theta)*math.sin(self.phi)],
  		[math.sin(self.theta), 0, math.cos(self.theta)]])
  	
  #Calculate angle of attack.
  def attackangle(self):
    zbhat=np.transpose(rotationmatrix(self))[2]
    zcomponent=np.dot(self.velocity,zbhat)
    v_plane=self.velocity-zbhat*zcomponent
    return math.atan(zcomponent/np.linalg.norm(v_plane))
