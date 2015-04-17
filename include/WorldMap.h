#ifndef WORLDMAP_H	   
#define WORLDMAP_H	  

#define BOOST_HAS_HASH
#include<stdio.h>
#include<unordered_map>
#include<vector>
#include<cmath>
#include<cstring>
#include<algorithm>
// #include<ext/std::unordered_map>
// #include <boost/archive/text_oarchive.hpp>
// #include <boost/archive/text_iarchive.hpp>
// #include <boost/serialization/std::unordered_map.hpp>
#include <fstream>
#include <sstream>

// using namespace __gnu_cxx;

class Edge
{
	private:
		int m_iSrcID;
		int m_iDstID;
		double dDist;

	public:
		Edge(int iNodeID1 , int iNodeID2 , double dDist);
		int GetSourceID();
		int GetDstID();
		double GetDistance();
};

class WorldMap
{
	private:
		// friend class boost::serialization::access;

		std::unordered_map< int, std::unordered_map< int, double > > m_hashWorldMap;
		std::unordered_map< int, std::unordered_map< int, double > > m_AllPairDistances;
		typedef std::unordered_map< int, double > InnerMap;
		typedef std::unordered_map< int, std::unordered_map< int, double > > OuterMap;
		bool bShortestPaths;
		
		// void serialize(boost::archive::text_iarchive & ar, const unsigned int file_version);
		// void serialize(boost::archive::text_oarchive & ar, const unsigned int file_version);
		
	public:	
		WorldMap(std::ifstream &pstVtxPath, std::ifstream &pstEdges);
		bool  AddEdge(Edge stEdge);
		bool  AddEdge(Edge* pstEdge);
		double GetDistance(int iSrcID , int iDstID);
		void ComputeAllPairsShortestPath();		
};

#endif