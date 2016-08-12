#Markov chain monte carlo practice

import random

Nin=0.0
Ntotal=0.0

for i in range(100000):
	x=random.random()	
	y=random.random()
	if (x**2)+(y**2)<1:
		Nin+=1.0
	Ntotal+=1.0

print(Nin)
print(Ntotal)

#Calculate area of quarter circle
area=Nin/Ntotal
print(area)

#Calculate area of quarter circle, which we see converges to pi
print(area*4)
