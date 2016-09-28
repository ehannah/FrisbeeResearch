import numpy as np
import math
import new_fris_coefficients as coef
import model_object

rho=1.225 #kg/m^3, density of air
area=0.057 #m^2, area of disc used in Hummel 2003
m=.175 #kg, mass of disc used in Hummel 2003
g=9.81 #m/s^2, gravitational acceleration
Izz=0.002352 #kg-m^2
Ixx=Iyy=Ixy=0.001219 #kg-m^2
d=2*(area/math.pi)**0.5 ##m; diameter of disc
print "DIAMETER:",d
#---------------------------------------------------------------------------------------------------#

#Create a Frisbee class, and assign Frisbee self values that correspond 
#to initial conditions (positions) of the frisbee.
class Frisbee(object):
  def __init__(self,x,y,z,vx,vy,vz,phi,theta,gamma, phidot,thetadot,gammadot,debug=False):
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
    self.debug=debug #Default value is false if debug isn't specified.
    
  #Represent Frisbee object by printing instantaneous position and velocity.
  def __str__(self):
    return "Position: (%f,%f,%f)\n"%(self.x,self.y,self.z)+"Velocity: (%f,%f,%f)\n"%(self.vx,self.vy,self.vz)

  def initialize_model(self,      PL0, PLa, PD0, PDa, PTya, PTywy, PTy0, PTxwx, PTxwz, PTzwz):
    self.model=model_object.Model(PL0, PLa, PD0, PDa, PTya, PTywy, PTy0, PTxwx, PTxwz, PTzwz)

#---------------------------------------------------------------------------------------------------#

  #Calculate rotation matrix. Rotation matrix is the product Ry(theta)*Rx(phi) of "Euler Chained Rotation
  #Matrices", found at https://en.wikipedia.org/wiki/Davenport_chained_rotations. 
  def rotationmatrix(self):
    phi=self.phi 
    theta=self.theta
    ct = math.cos(theta)
    st = math.sin(theta)
    cp = math.cos(phi)
    sp = math.sin(phi)

    return np.array([[ct, sp*st, -st*cp],
                     [0, cp, sp],
                     [st, -sp*ct, cp*ct]])

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
    return self.rotationmatrix()[2] #([001])

  #Calculate unit vector in x-body direction. Corresponds to unit vector in the direction
  #of the plane of the disc, which is distinct from the direction of the disc's velocity
  def xbhat(self):
    zcomponent=np.dot(self.velocity,self.zbhat()) #This is the lab-frame velocity vector
    #dotted with the lab z-body unit vector
    vplane=self.velocity-(self.zbhat()*zcomponent) #Velocity in the plane of the disc's motion
    return vplane/np.linalg.norm(vplane) #Unit vector in direction of plane of disc

  #Calculate unit vector in y-body direction, also expressed in lab frame.
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
    v_plane=self.velocity-(self.zbhat()*zcomponent)
    if self.debug:
      print "\nIn attackangle:"
      print zcomponent
      print v_plane
    return -math.atan(zcomponent/(np.linalg.norm(v_plane)))

#---------------------------------------------------------------------------------------------------#

  #Calculate dot product of velocity vector, by which we multiply forces.
  def velocity_dot(self):
    return np.dot(self.velocity,self.velocity)

#---------------------------------------------------------------------------------------------------#

  #Calculate forces acting on Frisbee
  def get_force(self):

    alpha=self.attackangle()
    v2=self.velocity_dot()
    F_lift=self.model.coef_lift(alpha)*0.5*rho*area*v2*np.cross(self.vhat(),self.ybhat())
    F_drag=self.model.coef_drag(alpha)*0.5*rho*area*v2*(-self.vhat())
    F_gravity=m*g*np.array([0.,0.,-1.])
    if self.debug:
      print "\nIn get_forces"
      print "CL:",self.model.coef_lift(alpha)
      print "CD:",self.model.coef_drag(alpha)
      print "Amplitude:",0.5*rho*area*v2

      print "alpha, v2:",alpha,v2
      print "F_lift:",F_lift
      print "F_drag:",F_drag
      print "F_grav:",F_gravity
      
    total_force=F_lift+F_drag+F_gravity

    return total_force

#---------------------------------------------------------------------------------------------------#
  #ASK TOM ABOUT THIS SECTION!!!!1
  
  #Calculate angular velocities in Frisbee frame using equation from Hummel 2003 (pg. 34)

  def ang_velocity_frisframe(self):
    return np.array([self.phidot*math.cos(self.theta), self.thetadot, (self.phidot*math.sin(self.theta)+self.gammadot)])

  #Calculate angular velocities in lab frame by taking dot product of angular_velocity_frisframe
  #array and rotation matrix.

  def ang_velocity_labframe(self):
    #return np.dot(self.rotationmatrix(),self.ang_velocity_frisframe())
    return np.dot(self.ang_velocity_frisframe(),self.rotationmatrix()) #I think I fixed the issue here - tom


  #Calculate the components of the lab frame angular velocities along body frame unit vectors
  def unit_ang_velocity(self):
    av_labframe = self.ang_velocity_labframe()
    wxb=np.dot(av_labframe, self.xbhat())
    wyb=np.dot(av_labframe, self.ybhat())
    wzb=np.dot(av_labframe, self.zbhat())
    if self.debug:
      print "bhat vectors:",self.xbhat(), self.ybhat(), self.zbhat() #This is fine
      print "Angular velocity in the frisbeeframe:",self.ang_velocity_frisframe()
      print "Angular velocity in the labframe:    ",av_labframe #I think there is a minus sign issue here
    return wxb, wyb, wzb

#---------------------------------------------------------------------------------------------------#
  #ASK TOM ABOUT THIS SECTION!!!!!

  #Calculate torques acting on Frisbee

  def get_torque(self):
    #Calculate alpha and velocity dot product
    alpha=self.attackangle()
    v2=self.velocity_dot()

    #Get x,y,z components of angular velocity from unit_ang_velocity function
    av_unit = self.unit_ang_velocity()
    wxb=av_unit[0]
    wyb=av_unit[1]
    wzb=av_unit[2]

    #X-body torque
    roll_moment  = self.model.coef_roll(wxb,wzb)*0.5*rho*d*area*v2*self.xbhat()
    #Y-body torque
    pitch_moment = self.model.coef_pitch(alpha,wyb)*0.5*rho*d*area*v2*self.ybhat()
    #Z-body torque
    spin_moment  = self.model.coef_spin(wzb)*0.5*rho*d*area*v2*np.array([0,0,1])

    #Total torque - (SEE IMPORTANT NOTE IN TOM'S CODE!)
    rotation_matrix = self.rotationmatrix()
    total_torque=np.dot(rotation_matrix,roll_moment)+np.dot(rotation_matrix,pitch_moment)+spin_moment
    #total_torque=np.dot(roll_moment,rotation_matrix)+np.dot(pitch_moment,rotation_matrix)+spin_moment

    if self.debug:
      print "\nIn get_torque"
      print "Aero Amp:",0.5*rho*d*area*v2
      print "Roll amp:",self.model.coef_roll(wxb,wzb)*0.5*rho*d*area*v2
      print "Pitch amp:", self.model.coef_pitch(alpha,wyb)*0.5*rho*d*area*v2
      print "Spin amp:",self.model.coef_spin(wzb)*0.5*rho*d*area*v2
      print "PitchMoment:",pitch_moment
      print "Pitch moment amplitude:",self.model.coef_pitch(alpha,wyb)
      print "w_b:",wxb, wyb, wzb
      print "Raw moments:",roll_moment,pitch_moment,spin_moment
      print "Moments:",np.dot(roll_moment,rotation_matrix),np.dot(pitch_moment,rotation_matrix),spin_moment
      print "total_torque:",total_torque
    return total_torque
#---------------------------------------------------------------------------------------------------#
  #Calculate derivatives of phidot, thetadot, and gammadot, which correspond to angular acceleration
  #values for phi, theta, and gamma. Units are radians/sec^2. Equations can be found in Hummel 2003 (pg. 48)

  def ang_acceleration(self):
    total_torque=self.get_torque()
    st = math.sin(self.theta)
    ct = math.cos(self.theta)
    phidot=self.phidot
    thetadot=self.thetadot
    gammadot=self.gammadot

    phi_dd=total_torque[0] + 2*Ixy*thetadot*phidot*st - Izz*thetadot*(phidot*st+gammadot)
    phi_dd /= (ct*Ixy)

    theta_dd=total_torque[1] + Izz*phidot*ct*(phidot*st+gammadot) - Ixy*phidot**2*ct*st
    theta_dd /= Ixy

    gamma_dd=total_torque[2] - Izz*(phidot*thetadot*ct + phi_dd*st)
    gamma_dd /= Izz

    if self.debug:
      print "\nIn ang_acceleration:"
      print "angle_dot:",phidot, thetadot, gammadot #This is fine
      print "I:",Ixy,Izz #This is fine
      print "phi_dd parts:",total_torque[0],2*Ixy*thetadot*phidot*st,Izz*thetadot*(phidot*st+gammadot)
      print "theta_dd parts:",total_torque[1],Izz*phidot*ct*(phidot*st+gammadot),Ixy*phidot**2*ct*st
      print "gamma_dd parts:",total_torque[2],Izz*(phidot*thetadot*ct + phi_dd*st)
    return np.array([phi_dd, theta_dd, gamma_dd])
#---------------------------------------------------------------------------------------------------#

  #Create array of derivatives to feed into numerical integrator
  #variable_array=[x-velocity, y-velocity, z velocity
    #x-accelration, y-acceleration, z-acceleration,
    #phi ang. velocity, theta ang. velocity, gamma ang. velocity
    #phi ang. acceleration, theta ang. acceleration, gamma ang. acceleration]

  def derivatives_array(self):
    forces = self.get_force()
    ang_acc = self.ang_acceleration()
    if self.debug:
      print "\nIn derivatives_array:"
      #print "forces: ",forces
      print "ang_accs:",ang_acc
    return [self.vx, self.vy, self.vz,
      forces[0]/m, forces[1]/m, forces[2]/m,
      self.phidot, self.thetadot, self.gammadot,
      ang_acc[0], ang_acc[1], ang_acc[2]]

