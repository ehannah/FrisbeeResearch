solution=open('solution.txt')
timestamps=open('solutiontime.txt')
out_file=open('simulated_solution.txt','w')

time=[]
xyz=[]
for line in timestamps:
	time.append(line.strip())
for line in solution:
	xyz.append(line.split()[0:3])

for i in range(len(xyz)):
	xyz[i].insert(0,time[i])
	xyz[i].append(str(0.25))
	xyz[i].append(str(0.25))
	xyz[i].append(str(0.25))
time_position_uncertainty=xyz

count=0
for element in time_position_uncertainty:
	if count%3==0:
		line=element[0]+' '+element[1]+' '+element[2]+' '+element[3]+' '+element[4]+' '+element[5]+' '+element[6]+' '
		out_file.write(line)
	count+=1
