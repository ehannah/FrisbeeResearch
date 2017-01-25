#Import relevant modules
import numpy as np
from scipy.integrate import odeint #Integrates a system of ordinary differential equations given initial conditions.
import math, sys
import frisbee_object as frisbee_object
import model_object
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#Initialize frisbee object with appropriate coefficient values and initial conditions.
#Current parameter input values obtained from Hummel 2003 (pg. 82)
#Change to debug=False to supress printing
init_positions = [0.,0.,1.,20.,0.,0.,0.,-.087,0.,0.,0.,-50.]
test_fris=frisbee_object.Frisbee(0.,0.,1.,20.,0.,0.,0.,-5.0*np.pi/180.,0.,0.,0.,-50.,debug=False)


#We pass the model directly to the frisbee
model = model_object.Model(PL0=0.3331, PLa=1.9124,PD0=0.1769,PDa=0.685,
                           PTya=0.4338,PTywy=-0.0144,PTy0=-0.0821,
                           PTxwx=-0.0125,PTxwz=-0.00171,
                           PTzwz=-0.0000341)
test_fris.model=model

pi=math.pi
alpha_0 = 4.0*math.pi/180. #radians (4 degrees), constant value reported in Hummel 2003

#Plot coefficients to show dependence on alpha

#Plotting lift coefficient
alpha_array=[]
lift_coef_array=[]

alpha=0

while alpha <= 2*pi:
	lift_coef=test_fris.model.coef_lift(alpha)
	#print(lift_coef)
	alpha_array.append(alpha)
	lift_coef_array.append(lift_coef)
	alpha+=pi/128

plt.plot(alpha_array, lift_coef_array)
plt.show()

#Plotting drag coefficient
alpha_array=[]
drag_coef_array=[]

alpha=0

while alpha <= 2*pi:
	drag_coef=test_fris.model.coef_drag(alpha)
	print(drag_coef)
	alpha_array.append(alpha)
	drag_coef_array.append(lift_coef)
	alpha+=pi/128

plt.plot(alpha_array, drag_coef_array)
plt.show()