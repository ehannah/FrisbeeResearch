#http://dan.iel.fm/emcee/current/user/line/
#Consider linear model where uncertainties underestimated by constant fractional amount



import emcee
import numpy as np
import scipy.optimize as op
import matplotlib.pyplot as pl

#To ensure same results as example:
np.random.seed(123)
#Define "true" parameters
m_true=-0.9594
b_true=4.294
f_true=0.534

#Generate synthetic data from model
N=50
x=np.sort(10*np.random.rand(N))
yerr=0.1+0.5*np.random.rand(N)
y=m_true*x+b_true
y += np.abs(f_true*y) * np.random.rand(N)
y += yerr * np.random.rand(N)

#Plotting synthetic data and true model:
x1=np.array([0,10])
pl.errorbar(x,y,yerr=yerr, fmt=".k")
pl.plot(x1, m_true*x1+b_true, "k", lw=3, alpha=0.6)
pl.ylim(-9,9)
pl.xlabel("$x$")
pl.ylabel("$y$")
pl.tight_layout()
pl.savefig("line-data.png")

#We can plot the synthetic data and estimate the parameters m and b via linear least squares,
#but it does not give us the best possible solution because it assumes the error bars
#in the model are correct. 

#Instead, we will use a likelihood function which is a Gaussian where the variance is 
#underestimated by some amount: f. Note that using ln(f) instead of just f forces f 
#to always be positive
'''
#Define the probability function as a likelihood * prior
def lnprior(theta):
	m, b, lnf = theta
	if -.5<m<0.5 and 0.0<b<10.0 and -10.0<lnf<1.0:
		return 0.0
	return -np.inf

#Define likelihood function
def lnlike(theta, x, y, yerr):
	m, b, lnf =theta
	model=m*x+b
	inv_sigma2=1.0/(yerr**2 + model**2*np.exp(2*lnf))
	return -0.5*(np.sum((y-model)**2*inv_sigma2 - np.log(inv_sigma2)))

#Combining lnlike and lnprior, we have a full log probability function
def lnprob(theta, x, y, yerr):
	lp=lnprior(theta)
	if not np.isfinite(lp):
		return -np.inf
	return lp +lnlike(theta, x, y, yerr)

#Find maximum likelihood value
chi2=lambda*args: -2* lnlike(*args)
result=op.minimize(chi2, [m_true, b_true, np.log(f_true)], args=(x,y,yerr))
m_ml, b_ml, lnf_ml=result["x"]
print("""Maximum likelihood result:
	m= {0} (truth: {1})
	b= {2} (truth: {3})
	f= {4} (truth: {5})
""".format(m_ml, m_true, b_ml, b_true, np.exp(lnf_ml), f_true))'''