PKG_NAME = 'cobot'
#import roslib; roslib.load_manifest(PKG_NAME)
import rospy
from random import *
from std_msgs.msg import *
import os
from math import *

horizon = 100 #units of time for which we are running the simulation
avSpeed = 10 #Cobot average speed used for task duration information
nCobots = 3
maxSubLengthFactor = 5 #maximum ratio of tasks which will be assigned to the same deadline
fnV = "../../maps/" #vertices path
fnO = "../../" #objects path

# Load vertices
vfn = os.path.join(os.path.dirname(fnV), "MapVertices.dat")
vdict = {}
rsz = struct.calcsize("3d")

with open(vfn) as fh:
    while True:
        dat = fh.read(rsz)
        if rsz!=len(dat): break
        (i, x, y) = struct.unpack("3d", dat)
        vdict[i] = (x, y)
        #print "VERTEX", i, "at", x, y
print "Loaded MapVertices.dat"

# Load object inventory
ofn = os.path.join(os.path.dirname(fnO), "objects.txt")
olist = []
with open(ofn) as fh:
    while True:
        line = fh.readline()
        if line == "end\n": break
        olist.append(line.rstrip('\n'))
        #print line
print "Loaded objects.txt"

vKeys = vdict.keys()
nVertices = len(vKeys)
nObjects = len(olist)


seed()

class task:
	object_id = None
	location = None
	deadline = None
	est_time =  None

def generate_tasks(): #generates a list of tasks for one Cobot which contains object_id 
	global horizon
	global nCobots
	global maxSubLengthFactor
	global avSpeed
	global nVertices
	global nObjects

	totalTime = 0
	task_list = []

	while True:
		object_id = randint(0,nObjects-1)
		location = vKeys[randint(0,nVertices-1)]
		#get the estimated task execution time (for now Euclidean distance from reference point (0,0))
		est_time = 2*sqrt(vdict[location][1]*vdict[location][1] + vdict[location][0]*vdict[location][0])/avSpeed
		totalTime += est_time
		if totalTime > horizon: break
		t = task()
		t.object_id = object_id
		t.location = location
		t.est_time = est_time
		task_list.append(t)


	#set deadlines
	cursor = 0 #a cursor on task index
	project_length = 0 #stores minimum time to complete so far
	nTasks = len(task_list)
	maxSubLength = nTasks/maxSubLengthFactor #maximum number of tasks assigned to same deadline
	print "maxSubLength:" + str(maxSubLength)
	print "nTasks:" + str(nTasks)
	while cursor < nTasks:
		subset = randint(1,min(maxSubLength,nTasks-cursor))
		#print subset
		for i in xrange(cursor,cursor+subset):
			project_length += task_list[i].est_time
		dl = uniform(project_length,horizon)
		for i in xrange(cursor,cursor+subset):
			task_list[i].deadline = dl
		cursor += subset
		#print cursor

	return task_list	

if __name__ == '__main__':
	task_lists = []
	for i in xrange(0,nCobots):
		task_list = generate_tasks()
		task_lists.append(task_list)
		print "Cobot" + str(i)
		for j in xrange(0,len(task_list)):
			print "Object:"+str(olist[task_list[j].object_id])+" Location:"+str(task_list[j].location)+" Est time:"+str(task_list[j].est_time)+" Deadline:"+str(task_list[j].deadline)