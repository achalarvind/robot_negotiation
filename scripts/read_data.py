import os
import struct
os.chdir('..')
path=os.path.split(os.path.realpath(__file__))
path=path[0]
data_dir=os.path.join(path,'data_files')
rsz = struct.calcsize("5d")



edges=os.path.join(data_dir,"MapEdges.dat")
vertices=os.path.join(data_dir,"MapVertices.dat")

vdict = {}
rsz = struct.calcsize("3d")
with open(vertices) as fh:
	while True:
		dat = fh.read(rsz)
		if rsz!=len(dat): break
		(i, x, y) = struct.unpack("3d", dat)
		vdict[i] = (x, y)
		print "VERTEX", i, "at", x, y
print "Loaded MapVertices.dat"

# Load edges
elist = []
rsz = struct.calcsize("5d")
with open(edges) as fh:
	eid = 0
	while True:
		dat = fh.read(rsz)
		if rsz!=len(dat): break
		e = struct.unpack("5d", dat)
		elist.append(list(e))
		print "EDGE", eid, "on", e
		eid += 1
print "Loaded MapEdges.dat"
# self.gvertices, self.gedges = vdict, elist



# for fh in edges:
# 	dat = fh.read(rsz)
# 	if rsz!=len(dat):
# 		break
# 	e = struct.unpack("5d", dat)
# 	print e
# 	print list(e)
# 	elist.append(list(e))
# 	#print "EDGE", eid, "on", e
# 	eid += 1