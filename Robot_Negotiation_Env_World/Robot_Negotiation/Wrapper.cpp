#include "WorldMap.h"
#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <iostream>
#include "Temp.h"
#include <fstream>
#include <sstream>

using namespace std;

int main()
{
	//Serialization for WorldMap
	//WorldMap *pWorld = new WorldMap();

	////Save file
	//std::ofstream ofs("WorldMap.txt");
	//boost::archive::text_oarchive oa(ofs);
	//oa & *pWorld;
	//ofs.close();

	////Load from file
	//WorldMap obNewWorld;
	//std::ifstream ifs("WorldMap.txt");
	//boost::archive::text_iarchive ia(ifs);
	//ia >> obNewWorld;
	//ifs.close();

	std::ifstream stVtxPath;
	stVtxPath.open("G:\\Visual_Studio_Projects\\Robot_Negotiation\\Robot_Negotiation\\MapVertices.txt");

	std::ifstream stEdges;
	stEdges.open("G:\\Visual_Studio_Projects\\Robot_Negotiation\\Robot_Negotiation\\MapEdges.txt");

	WorldMap *pWorld = new WorldMap(stVtxPath, stEdges);
	pWorld->ComputeAllPairsShortestPath();

	delete(pWorld);

	return 1;
}

//int main()
//{
//	Temp obj;
//	obj.Initialize();
//	
//
//	//save file
//	std::ofstream ofs("worldmap.txt");
//	boost::archive::text_oarchive oa(ofs);
//	oa & obj;
//	ofs.close();
//
//	Temp newobj;
//	std::ifstream ifs("worldmap.txt");
//	boost::archive::text_iarchive ia(ifs);
//	ia >> newobj;
//
//
//	/*for (int icount = 0; icount < newobj.m_DoubleVector.size(); icount++)
//	{
//		cout << newobj.m_DoubleVector[icount];
//	}*/
//
//	for (int icount = 0; icount < newobj.m_DoubleHash.size(); icount++)
//	{
//		cout << newobj.m_DoubleHash.find(icount)->second;
//	}
//
//	return 1;
//}