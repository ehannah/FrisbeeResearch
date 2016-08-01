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
    
  def __str__(self):
    return 'Position: (' + str(self.x)+','+str(self.y)+','+str(self.z)+')'+'\n'
      +'Velocity: ('+str(self.vx)+','+str(self.vy)+','+str(self.vz)+')'
