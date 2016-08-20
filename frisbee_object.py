import numpy as np
import math
import new_fris_coefficients as coef
import model_object

rho=1.225 #kg/m^3, density of air
area=0.057 #m^2, area of disc used in Hummel 2003
m=1.25 #kg, mass of disc used in Hummel 2003
g=9.81 #m/s^2, gravitational acceleration
Izz=0.00235 #kg-m^2
Ixx=Iyy=Ixy=0.00122 #kg-m^2
#---------------------------------------------------------------------------------------------------#

#Create a Frisbee class, and assign Frisbee self values that correspond 
#to physical characteristics of the frisbee.
class Frisbee(object):
  def __init__(self,x,y,z,vx,vy,vz,phidot,thetadot,gammadot,phi,theta,gamma):
    self.x=x
    self.y=y
    self.z=z
    self.vx=vx
    self.vy=vy
    self.vz=vz
    self.phidot=phidot
    self.thetadot=thetadot
    self.gammadot=gammadot
    self.phi=phi
    self.theta=theta
    self.gamma=gamma
    self.velocity=np.array([vx,vy,vz])
    self.area=0.057 #m^2, surface area of Discraft Ultrastar (Hummel 2003)
    self.diameter=0.269 #m, diameter of Discraft Ultrastar (Hummel 2003)

  #Represent Frisbee object by printing instantaneous position and velocity.
  def __str__(self):
    return "Position: (%f,%f,%f)\n"%(self.x,self.y,self.z)+"Velocity: (%f,%f,%f)\n"%(self.vx,self.vy,self.vz)

  def initialize_model(self, PL0, PLa, PD0, PDa, PTya, PTywy, PTy0, PTxwx, PTxwz, PTzwz):
    self.model=model_object.Model(PL0, PLa, PD0, PDa, PTya, PTywy, PTy0, PTxwx, PTxwz, PTzwz)

#---------------------------------------------------------------------------------------------------#

  #Calculate rotation matrix. Rotation matrix is the product Rz*Ry of "Euler Chained Rotation
  #Matrices", found at https://en.wikipedia.org/wiki/Davenport_chained_rotations. 
  def rotationmatrix(self):
    return np.array(
  		[[math.cos(self.phi)*math.cos(self.theta), -math.sin(self.phi), -math.cos(self.phi)*math.sin(self.theta)],
  		[math.sin(self.phi)*math.cos(self.theta), math.cos(self.phi), -math.sin(self.theta)*math.sin(self.phi)],
  		[math.sin(self.theta), 0, math.cos(self.theta)]])

#---------------------------------------------------------------------------------------------------#
  #DECIDE WHETHER LAB-->FRIS ROTATION OR VICE VERSA
  
  #Below are unit important unit vectors which we will ultimately use to orient the forces
  #and torques in the correct directions.

  #Calculate unit vector in the direction of the disc's velocity (in lab frame).
  def vhat(self):
    return self.velocity/np.linalg.norm(self.velocity)

  #Calculate unit vector in z-body direction of lab frame. 
  #C3 vector in Hummel 2003 is equivalent to z-body unit vector expressed in inertial coordinates
  #In rotation matrix used for this program, Hummel's C3 corresponds to the first row
  #of the matrix written from right to left. (C3'=[-cos(phi)sin(theta), -sin(phi), cos(phi)cos(theta)])
  #In python, we obtain the first row of the rotation matrix from right to left by
  #using np.fliplr(), which flips an array in the left/right direction.

  def zbhat(self):
    return np.fliplr(self.rotationmatrix())[0] #([001])

  #Calculate unit vector in x-body direction. Corresponds to unit vector in the direction
  #of the plane of the disc, which is distinct from the direction of the disc's velocity
  def xbhat(self):
    zcomponent=np.dot(self.velocity,self.zbhat()) #This is the lab-frame velocity vector
    #dotted with the frisbee-frame z-body unit vector
    vplane=self.velocity-self.zbhat()*zcomponent #Velocity in the plane of the disc's motion
    return vplane/np.linalg.norm(vplane) #Unit vector in direction of plane of disc

  #Calculate unit vector in y-body direction
  def ybhat(self):
    return np.cross(self.zbhat(),self.xbhat())

#---------------------------------------------------------------------------------------------------#
  #HOW DO I KNOW WHETHER I WANT ATAN OR -ATAN??

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

#---------------------------------------------------------------------------------------------------#

  #Calculate dot product of velocity vector, by which we multiply forces.
  def velocity_dot(self):

    return np.dot(self.velocity,self.velocity)

#---------------------------------------------------------------------------------------------------#

  #Calculate forces acting on Frisbee
  def get_force(self):

    alpha=self.attackangle()
    velocity=self.velocity_dot()

    F_lift=self.model.coef_lift(alpha)*0.5*rho*area*velocity*np.cross(self.vhat(),self.ybhat())

    F_drag=self.model.coef_drag(alpha)*0.5*rho*area*velocity*-self.vhat()

    F_gravity=m*g*np.array([0,0,-1])
    
    total_force=F_lift+F_drag+F_gravity

    return total_force

#---------------------------------------------------------------------------------------------------#
  #ASK TOM ABOUT THIS SECTION!!!!1
  
  #Calculate angular velocities in Frisbee frame using equation from Hummel 2003 (pg. 34)

  def ang_velocity_frisframe(self):
    return np.array([self.phidot*math.cos(self.theta), self.thetadot, self.phidot*math.sin(self.theta)])

  #Calculate angular velocities in lab frame by taking dot product of angular_velocity_frisframe
  #array and rotation matrix.

  def ang_velocity_labframe(self):
    return np.dot(self.rotationmatrix(),self.ang_velocity_frisframe())

  #Calculate the components of the lab frame angular velocities along body frame unit vectors
  def unit_ang_velocity(self):
    wxb=np.dot(self.ang_velocity_labframe(), self.xbhat())
    wyb=np.dot(self.ang_velocity_labframe(), self.ybhat())
    wzb=np.dot(self.ang_velocity_labframe(), self.zbhat())
    return wxb, wyb, wzb

#---------------------------------------------------------------------------------------------------#
  #ASK TOM ABOUT THIS SECTION!!!!!

  #Calculate torques acting on Frisbee

  def get_torque(self):

    #Calculate alpha and velocity dot product
    alpha=self.attackangle()
    velocity=self.velocity_dot()

    #Get x,y,z components of angular velocityfrom unit_ang_velocity function
    wxb=self.unit_ang_velocity()[0]
    wyb=self.unit_ang_velocity()[1]
    wzb=self.unit_ang_velocity()[2]

    #X-body torque
    roll_moment=self.model.coef_roll(wxb,wxb)*0.5*rho*area*velocity*self.xbhat()
    #Y-body torque
    pitch_moment=self.model.coef_pitch(alpha,wyb)*0.5*rho*area*velocity*self.ybhat()
    #Z-body torque
    spin_moment=self.model.coef_spin(wzb)*0.5*rho*area*velocity*np.array([0,0,1])

    #Total torque - (SEE IMPORTANT NOTE IN TOM'S CODE!)
    total_torque=np.dot(self.rotationmatrix(),roll_moment)+np.dot(self.rotationmatrix(),pitch_moment)+spin_moment

    return total_torque
#---------------------------------------------------------------------------------------------------#
  #Calculate derivatives of phidot, thetadot, and gammadot, which correspond to angular acceleration
  #values for phi, theta, and gamma. Units are radians/sec^2. Equations can be found in Hummel 2003 (pg. 48)

  def ang_acceleration(self):

    total_torque=self.get_torque()

    phi_dd=(total_torque[0] + Ixy*self.thetadot*self.phidot*math.sin(self.theta) - 
      Izz*self.thetadot*(self.phidot*math.sin(self.theta)+self.gammadot) + 
      Ixy*self.thetadot*self.phidot*math.sin(self.theta)*math.cos(self.theta)/Ixy)

    theta_dd=(total_torque[1]+ Izz*self.phidot*math.cos(self.theta)*(self.phidot*math.sin(self.theta)+
      self.gammadot)-Iyy*(self.phidot**2)*math.cos(self.theta)*math.sin(self.theta)/Ixy)

    gamma_dd=(total_torque[2] -Izz*phi_dd*math.sin(self.theta)+self.thetadot*self.phidot*math.cos(self.theta)/Izz)

    return np.array([phi_dd, theta_dd, gamma_dd])
#---------------------------------------------------------------------------------------------------#

  #Create array of derivatives to feed into numerical integrator
  #variable_array=[x-velocity, y-velocity, z velocity
    #x-accelration, y-acceleration, z-acceleration,
    #phi ang. velocity, theta ang. velocity, gamma ang. velocity
    #phi ang. acceleration, theta ang. acceleration, gamma ang. acceleration]

  def derivatives_array(self):
    return [self.vx, self.vy, self.vz,
      self.get_force()[0]/m, self.get_force()[1]/m, self.get_force()[2]/m,
      self.phidot, self.thetadot, self.gammadot,
      self.ang_acceleration()[0], self.ang_acceleration()[1], self.ang_acceleration()[2]]

