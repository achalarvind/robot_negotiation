#!/usr/bin/python

# PKG_NAME = 'cobot'
#import roslib; roslib.load_manifest(PKG_NAME)

import rospy
from random import *
from std_msgs.msg import *
import os
from math import *

from robot_negotiation.srv import *
from robot_negotiation.msg import *

horizon = 100 #units of time for which we are running the simulation
avSpeed = 10 #Cobot average speed used for task duration information
nCobots = 3
maxSubLengthFactor = 5 #maximum ratio of tasks which will be assigned to the same deadline

os.chdir('..')
path=os.path.split(os.path.realpath(__file__))
path=path[0]
fnV=os.path.join(path,'../data_files') #vertices path
print fnV
fn0 = os.path.join(path,'../data_files') #objects path
waitingNode = 16; #GHC6


get_distance = rospy.ServiceProxy('get_distance', GetDistance)
# Load vertices
vfn = os.path.join(fnV ,"MapVertices.dat")
print vfn
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
ofn = os.path.join(fnV, "objects.txt")
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
		# print("distance is",get_distance(int(42),int(waitingNode)))

		object_id = randint(0,nObjects-1)
		location = vKeys[randint(0,nVertices-1)]
		#get the estimated task execution time (for now Euclidean distance from reference point
		distance=get_distance(int(location),int(waitingNode))
		#distance=100
		est_time = 2*distance.distance/avSpeed
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

def generate_tasks_handler(req):
	tasks=generate_tasks()
	print "service called"
	task_list=TaskList()
	for i in xrange(len(tasks)):
		t = Task()
		t.object_id = tasks[i].object_id
		t.destination = tasks[i].location
		t.est_time = tasks[i].est_time
		t.deadline = tasks[i].deadline
		task_list.task_list.append(t)

	return GetTasksResponse(task_list)	



# if __name__ == '__main__':
# 	task_lists = []
# 	for i in xrange(0,nCobots):
# 		task_list = generate_tasks()
# 		task_lists.append(task_list)
# 		print "Cobot" + str(i)
# 		for j in xrange(0,len(task_list)):
# 			print "Object:"+str(olist[task_list[j].object_id])+" Location:"+str(task_list[j].location)+" Est time:"+str(task_list[j].est_time)+" Deadline:"+str(task_list[j].deadline)

def add_two_ints_server():
    rospy.init_node('task_server')
    s = rospy.Service('task_generator', GetTasks, generate_tasks_handler)
    print "Ready to generate tasks"
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()