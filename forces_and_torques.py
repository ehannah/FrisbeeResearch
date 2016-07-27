import new_fris_coefficients as coef

rho=1.225 #kg/m^3, density of air
v=[] #velocity vector, passed in to force/torque equations
A=0.057 #m^2, area of disc used in Hummel 2003

#Force and torque functions found on pg. 35 of Hummel 2003

#Velocity vector (dot product of vx,vy,vz)
def velocity(vx,vy,vz):
	v.append(vx) #velocity in x direction, m/s
	v.append(vy) #velocity in y direction, m/s
	v.append(vz) #velocity in z direction, m/s
	return (v[0]**2)+(v[1]**2)+(v[2]**2)

#Lift force
def F_lift(vx,vy,vz,alpha):
	return coef.coef_L(alpha)*0.5*rho*A*(velocity(vx,vy,vz))

#Drag force
def F_drag(vx,vy,vz,alpha):
	return coef.coef_D_total(alpha)*0.5*rho*A*(velocity(vx,vy,vz))

#X-body torque (tau_x, roll moment)
def x_torque(wx,wz,vx,vy,vz):
	#wx,wz correspond to angular velocity in respective directions
	return coef.coef_tau_x_total(wx,wz)*0.5*rho*A*(velocity(vx,vy,vz))

#Y-body torque (tau_y, pitch moment)
def y_torque(wy,vx,vy,vz, alpha):
	return coef.coef_tau_y_total(alpha,wy)*0.5*rho*A*(velocity(vx,vy,vz))

#Z-body torque (tau_z, yaw moment):
def z_torque(wz,vx,vy,vz):
	return coef.coef_tau_z_total(wz)*0.5*rho*A*(velocity(vx,vy,vz))