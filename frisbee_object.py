import numpy as np
import math
import new_fris_coefficients as coef

#Create a Frisbee class, and assign Frisbee self values that correspond 
#to physical characteristics of the frisbee.
class Frisbee(object):
  def __init__(self,x,y,z,vx,vy,vz,wx,wy,wz,psi,theta,phi,):
    self.x=x
    self.y=y
    self.z=z
    self.vx=vx
    self.vy=vy
    self.vz=vz
    self.wx=wx
    self.wy=wy
    self.wz=wz
    self.psi=psi
    self.theta=theta
    self.phi=phi
    self.velocity=np.array([vx,vy,vz])
    self.area=0.057 #m^2, surface area of Discraft Ultrastar (Hummel 2003)
    self.diameter=0.269 #m, diameter of Discraft Ultrastar (Hummel 2003)

    
  #Represent Frisbee object by printing instantaneous position and velocity.
  def __str__(self):
    return "Position: (%f,%f,%f)\n"%(self.x,self.y,self.z)+"Velocity: (%f,%f,%f)\n"%(self.vx,self.vy,self.vz)

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
    v_plane=self.velocity-self.zbhat()*zcomponent
    return math.atan(zcomponent/(np.linalg.norm(v_plane)))

  #Calculate dot product of velocity vector, by which we multiply forces.
  def velocity_dot(self):
    return np.dot(self.velocity,self.velocity)

  #Calculate lift force acting on Frisbee. (Note that at this point, forces are not rotated
  #from lab frame to frisbee frame.
  def F_lift(self):
    return coef.coef_L(self.attackangle(), param_L_0, param_L_alpha)*0.5*rho*self.area*self.velocity_dot()

  def F_drag(self):
    return coef.coef_D_total(self.attackangle(), param_D_alpha, param_D_0)*0.5*rho*self.area*self.velocity_dot()


