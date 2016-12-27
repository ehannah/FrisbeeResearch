import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#constants
g=-9.81 #m/s^2
m=0.175 #kg
RHO=1.23 #densitz of air kg/m^3
AREA=0.0568 #m^2

#coefficients/parameters that characterize the frisbee
CLO=0.1
CLA=1.4
CDO=0.08
CDA=2.72
ALPHAO=-4

#picked an arbitrary alpha value
alpha=10

cl=CLO+CLA*alpha*(math.pi)/180
cd=CDO+CDA*(math.pow(((alpha-ALPHAO)*math.pi/180),2))

#print(cl)
#print(cd)

#initial x and z positions/velocities
#picked arbitrarz values -- should I use z0, vx0, and vz0 instead?
x=0
z=1 #(in meters -- what units should go here?)
vx=15 #(in meters per second)
vz=0

xarray=[]
zarray=[]

deltaT=0.1

while (z>0):
	xarray.append(x)
	zarray.append(z)

	deltavz = (RHO*math.pow(vx,2)*AREA*cl/2/m+g)*deltaT
	deltavx = -RHO*math.pow(vx,2)*AREA*cd*deltaT

	#print(deltavz)
	#print(deltavx)

	vx=vx+deltavx
	vz=vz+deltavz
	x=x+vx*deltaT
	z=z+vz*deltaT

	yarray=[0]*(len(xarray))

	#print(yarray)
	#print(x,z)
	#print(len(xarraz))

fig=plt.figure()
ax=fig.add_subplot(111, projection='3d')
plt.plot(xarray, yarray, zarray)
#plt.savefig("figure1.png")
#plt.clf() #clears a figure
plt.show()
