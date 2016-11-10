"""
This is a template for what the analysis code should look like.
"""
import numpy as np
from scipy import InterpolatedUnivariateSpline as IUS
import fris_wrapper as wrapper
"""
Step 1, read in the flight data.
The data file should have the following format:
t x y z x_err y_err z_err
"""
input_name = "new_simulated_throw.txt"
data = np.genfromtxt(input_name).T #Need to flip it to get it to be 7xN
print data.shape

"""
Step 2
Define our prior
"""
def lnprior(parameters):
    PL0, Pla, PD0, PDa, PTya, PTywy, PTy0, PTxwx, PTxwz, PTzwz = parameters 
    """
    Account for unphysical models
    and consider them impossible.
    """

    """
    This is an example of a flat prior
    """
    use_flat_priors = True
    if use_flat_priors:
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
    PL0, Pla, PD0, PDa, PTya, PTywy, PTy0, PTxwx, PTxwz, PTzwz = parameters
    t,x,y,z,x_err,y_err,z_err = data
    
    """
    Get a simulated throw, our "model"
    This requires the model parameters,
    the initial and final times (which should be longer than the real throw),
    and the initial positions.
    """
    model = wrapper.get_throw(parameters,t[0],t[-1]+1,x[0],y[0],z[0])
    t_model = model[0]
    model_positions = model[1]
    x_model,y_model,z_model = model_positions[:,0], model_positions[:,1], model_positions[:,2] #Disassemble the output of the model

    """
    Make splines for our positions
    """
    x_spline = IUS(t_model,x_model)
    y_spline = IUS(t_model,y_model)
    z_spline = IUS(t_model,z_model)

    """
    Use a chi^2 on the positions
    """
    xchi2 = -0.5*(x-x_spline(t))**2/(x_err**2)
    ychi2 = -0.5*(y-y_spline(t))**2/(y_err**2)
    zchi2 = -0.5*(z-z_spline(t))**2/(z_err**2)

    return xchi2 + ychi2 + zchi2

"""
Step 4
Define the posterior probability that we are sampling.
"""
def lnprob(parameters,data):
    prior = lnprior(parameters):
    if not np.isfinite(prior):
        return -np.inf
    return prior + lnlike(parameters,data)

"""
Step 5
Interface with emcee in order to do MCMC and
sample the posterior.
"""
import emcee
"""
This is a 10 dimensional posterior,
so we need at least double that number of walkers.
"""
nwalkers, ndim = 24, 10

"""
Create a sampler object from emcee. It needs to know
how many walkers it will have and the dimensionality of the
problem. It also needs to know what function is the posterior (lnprob)
and if there are any extra arguments to it (data).
"""
sampler = emcee.EnsembleSampler(nwalkers,ndim,lnprob,args=(data))

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
true_params = [0.3331,1.9124,0.1769,0.685,0.4338,-0.0144,-0.0821,-0.0125,-0.00171,-0.0000341]
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
nsteps = 500 #Arbitrary
sampler.run_mcmc(pos,nsteps)

"""
Step 8
Pull out the chain and analyze it.
"""
chain = sampler.chain
means = np.mean(chain,1) #Find the means of the parameters
stddev = np.stddev(chain,1) #Find the standard deviations

import matplotlib.pyplot as plt
import corner
#Create a corner plot
fig = corner.corner(chain, labels=[],truths = true_params)
plt.show()
