"""
This is a template for what the analysis code should look like.
"""

#Thoughts from Kevin: Set prior concentrated on true value and see if true value comes back
#Initialize near true answer and see if we get true answer back
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import InterpolatedUnivariateSpline as IUS
import fris_wrapper as wrapper
"""
Step 1, read in the flight data.
The data file should have the following format:
t x y z x_err y_err z_err
"""
input_name = ("new_simulated_solution.txt")
data = np.genfromtxt(input_name).T #Need to flip it to get it to be 7xN
print data.shape
#Define parameters that we are not interested in calculating (i.e. everything but lift and drag)
other_params = np.array([0.3331, 1.9124, 0.685, 0.4338, -0.0144, -0.0821, -0.0125, -0.00171, -0.0000341])

"""
Step 2
Define our prior
"""
def lnprior(parameters):
    #Extract parameter we are interested in studying
    PD0 = parameters[2]
    print(PD0)
    PL0, Pla, PDa, PTya, PTywy, PTy0, PTxwx, PTxwz, PTzwz = other_params
    """
    Account for unphysical models
    and consider them impossible.
    """

    """
    This is an example of a flat prior
    """
    use_flat_priors = True
    if use_flat_priors:
        if abs(PL0)>1:
            return -np.inf
        else:
            return 0.0


    """
    This is an example of a gaussian prior.
    Let's pretend that a previous measurement of
    CL0 gave 0.1 +/- 0.02
    The rest of the parameters have flat priors (0.0)
    on them.
    """
    #We are not using for now (perhaps later, we may try to guess validity of error bars)
    #return - 0.5 * (CL0 - 0.1)**2/0.02**2

"""
Step 3
Define our likelihood
"""
def lnlike(parameters,data):
    PD0 = parameters[2]
    PL0, Pla, PDa, PTya, PTywy, PTy0, PTxwx, PTxwz, PTzwz = other_params
    new_parameters = np.array([PL0, Pla, PD0, PDa, PTya, PTywy, PTy0, PTxwx, PTxwz, PTzwz])
    t,x,y,z,x_err,y_err,z_err = data


    #print parameters

    """
    Get a simulated throw, our "model"
    This requires the model parameters,
    the initial and final times (which should be longer than the real throw),
    and the initial positions.
    """
    #Hard code in initial positions beyond x, y, z
    initial_conditions=[x[0], y[0], z[0], 20, 0, 0, 0*np.pi/180, -5*np.pi/180, 0*np.pi/180, 0, 0, -50]
    #print(initial_conditions)

    #Put in correct order (i.e. that which is in get_throw)
    #Here, instead of just writing parameters, use array that contains lift and drag parameters, and then hard code in torque parameters
    model = wrapper.get_throw(initial_conditions, new_parameters,t[0],t[-1]+1)
    t_model = model[0]
    #print(t_model)
    model_positions = model[1]
    x_model,y_model,z_model = model_positions[:,0], model_positions[:,1], model_positions[:,2] #Disassemble the output of the model
    #print x_model, y_model, z_model
    """
    Make splines for our positions
    """
    x_spline = IUS(t_model,x_model)
    y_spline = IUS(t_model,y_model)
    z_spline = IUS(t_model,z_model)

    """
    Use a chi^2 on the positions
    """
    xchi2 = -0.5*sum((x-x_spline(t))**2/(x_err**2))
    ychi2 = -0.5*sum((y-y_spline(t))**2/(y_err**2))
    zchi2 = -0.5*sum((z-z_spline(t))**2/(z_err**2))

    #print xchi2, tychi2, zchi2
    return xchi2 + ychi2 + zchi2

"""
Step 4
Define the posterior probability that we are sampling.
"""
def lnprob(parameters,data,blah):
    prior = lnprior(parameters)
    if not np.isfinite(prior):
        return -np.inf
    return prior + lnlike(parameters,data)

test_parameters = [0.3331,1.9124,0.1769,0.685,0.4338,-0.0144,-0.0821,-0.0125,-0.00171,-0.0000341]

#should get zeros for both lnprior and lnprob
#Step two: try with small perterbations on parameters; should produce non-zero outputs.
#print lnprior(test_parameters)
#lnprob(test_parameters, data)

"""
(Note that written by Tom)

Step 5
Interface with emcee in order to do MCMC and
sample the posterior.
"""
import emcee
"""
This is a 10 dimensional posterior,
so we need at least double that number of walkers.
"""
nwalkers, ndim = 2, 1

"""
Create a sampler object from emcee. It needs to know
how many walkers it will have and the dimensionality of the
problem. It also needs to know what function is the posterior (lnprob)
and if there are any extra arguments to it (data).
"""
sampler = emcee.EnsembleSampler(nwalkers,ndim,lnprob,args=(data,0.0))

"""
Step 6
Guess initial positions of the walkers.
For now, use the parameter values that
we used in our throw, but smeared out
a bit in random directions.

Let's use a gaussian with 50% width
on the means, where the means are
the true parameters.
"""
true_params = [0.1769]
pos = np.zeros((nwalkers,ndim))
for i in range(nwalkers):
    #The position is the true parameters plus a random component.
    #The random component is a gaussian (randn) with a width of
    #50% (0.5) of the true parameters.
    pos[i] = true_params + 0.5*np.fabs(true_params)*np.random.randn(ndim)

"""
Step 7
Decide how many steps you will use and tell the sampler to do mcmc.
"""
#Increase to larger numbers, until corner plots remain the same upon substantial increases.
nsteps = 50 #Arbitrary
sampler.run_mcmc(pos,nsteps)

"""
Step 8
Pull out the chain and analyze it.
"""
chain = sampler.chain.reshape((-1,ndim))
np.save("chain",chain)
means = np.mean(chain,1) #Find the means of the parameters
stddev = np.std(chain,1) #Find the standard deviations

import corner
#Create a corner plot
fig = corner.corner(chain)#, labels=[],truths = true_params)
fig.savefig("PD0_simulation_recovery_nsteps=2000.png")
#plt.show()
