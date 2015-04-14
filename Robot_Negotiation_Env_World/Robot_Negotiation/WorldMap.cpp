#include "WorldMap.h"
#include "Constants.h"
using namespace std;

int Edge::GetSourceID()
{
	return m_iSrcID;
}

int Edge::GetDstID()
{
	return m_iDstID;
}

double Edge::GetDistance()
{
	return dDist;
}

Edge::Edge(int iNodeID1, int iNodeID2, double Distance)
{
	m_iSrcID = iNodeID1;
	m_iDstID = iNodeID2;
	dDist = Distance;
}

void WorldMap::serialize(boost::archive::text_iarchive & ar, const unsigned int file_version)
{
	ar & m_hashWorldMap;
	ar & m_AllPairDistances;
}

void WorldMap::serialize(boost::archive::text_oarchive & ar, const unsigned int file_version)
{
	ar & m_hashWorldMap;
	ar & m_AllPairDistances;
}

WorldMap::WorldMap(std::ifstream &pstVtxPath, std::ifstream &pstEdges)
{
	bShortestPaths = false;

	std::hash_map< double, std::pair<double, double> > obHashVtx;

	std::string stVertexline;
	while (std::getline(pstVtxPath, stVertexline))
	{
		std::istringstream iss(stVertexline);
		std::string stVertex, stDummy;
		double dVertexNum;
		double dX, dY;

		//cout << iss.str();

		if (!(iss >> stVertex >> dVertexNum >> stDummy >> dX >> dY))
		{
			break;
		} // error

		obHashVtx.insert(std::make_pair(dVertexNum, std::make_pair(dX, dY)));
	}

	char chars[] = "(),";
	std::string stEdgeline;
	while (std::getline(pstEdges, stEdgeline))
	{
		for (unsigned int i = 0; i < strlen(chars); ++i)
		{
			// you need include <algorithm> to use general algorithms like std::remove()
			stEdgeline.erase(std::remove(stEdgeline.begin(), stEdgeline.end(), chars[i]), stEdgeline.end());
		}

		std::istringstream iss(stEdgeline);
		std::string stVertex;
		std::string stEdge, stDummy;
		int iEdgeNum;
		double dVtx1, dVtx2;
		int iVtx1, iVtx2;
		double dX, dY;

	//	cout << iss.str()<<"\n";

		if (!(iss >> stEdge >> iEdgeNum >> stDummy >> dVtx1 >> dVtx2))
		{
			break;
		} // error

		iVtx1 = (int)dVtx1;
		iVtx2 = (int)dVtx2;

		std::pair<double , double> stCoord1 = (obHashVtx.find(iVtx1))->second;
		std::pair<double, double> stCoord2 = (obHashVtx.find(iVtx2))->second;

		dX = stCoord1.first - stCoord2.first;
		dY = stCoord1.second - stCoord2.second;

		double dDist = sqrt( (dX * dX) + (dY * dY) );

		Edge stNewEdge(iVtx1, iVtx2, dDist);
		AddEdge(stNewEdge);
	}
}

bool WorldMap::AddEdge(Edge stEdge)
{	
	return AddEdge(&stEdge);
}

bool WorldMap::AddEdge(Edge* pstEdge)
{
	OuterMap::iterator itSrcIndex = m_hashWorldMap.find(pstEdge->GetSourceID());

	if (itSrcIndex == m_hashWorldMap.end())
	{
		InnerMap hashIndexMap;
		hashIndexMap.insert(std::make_pair(pstEdge->GetDstID(), pstEdge->GetDistance()));
		std::pair <OuterMap::iterator, bool> stSuccess = m_hashWorldMap.insert(std::make_pair(pstEdge->GetSourceID(), hashIndexMap));
		
		if (!stSuccess.second)
		{
			return false;
		}
	}
	else
	{
		std::pair <InnerMap::iterator, bool> stSuccess = itSrcIndex->second.insert(std::make_pair(pstEdge->GetDstID(), pstEdge->GetDistance()));
		
		if (!stSuccess.second)
		{
			return false;
		}
	}

	OuterMap::iterator itDstIndex = m_hashWorldMap.find(pstEdge->GetDstID());
	if (itDstIndex == m_hashWorldMap.end())
	{
		InnerMap hashIndexMap;
		hashIndexMap.insert(std::make_pair(pstEdge->GetSourceID(), pstEdge->GetDistance()));
		std::pair <OuterMap::iterator, bool> stSuccess = m_hashWorldMap.insert(std::make_pair(pstEdge->GetDstID(), hashIndexMap));

		if (!stSuccess.second)
		{
			return false;
		}
	}
	else
	{
		std::pair <InnerMap::iterator, bool> stSuccess = itDstIndex->second.insert(std::make_pair(pstEdge->GetSourceID(), pstEdge->GetDistance()));

		if (!stSuccess.second)
		{
			return false;
		}
	}

	return true;
}

double WorldMap::GetDistance(int iSrcID, int iDstID)
{
	if (bShortestPaths)
	{
		OuterMap::iterator itSrc = m_AllPairDistances.find(iSrcID);
		InnerMap::iterator itDst = itSrc->second.find(iDstID);

		return itDst->second;
	}

	OuterMap::iterator itSrc = m_hashWorldMap.find(iSrcID);
	
	if (m_hashWorldMap.end() == itSrc)
	{
		return MAX_DIST_VALUE;
	}

	InnerMap::iterator itDst = itSrc->second.find(iDstID);

	if (itSrc->second.end() == itDst)
	{
		return MAX_DIST_VALUE;
	}

	return itDst->second;
}

void WorldMap::ComputeAllPairsShortestPath()
{
	int iNumOfNodes = m_hashWorldMap.size();

	m_AllPairDistances.reserve(iNumOfNodes);

	InnerMap hMap;
	std::vector<int> vecKeys;

	for (OuterMap::iterator itOutBegin = m_hashWorldMap.begin(); itOutBegin != m_hashWorldMap.end(); itOutBegin++)
	{
		hMap.insert(std::make_pair(itOutBegin->first , MAX_DIST_VALUE));
		vecKeys.push_back(itOutBegin->first);
	}

	for (OuterMap::iterator itOutBegin = m_hashWorldMap.begin(); itOutBegin != m_hashWorldMap.end(); itOutBegin++)
	{
		std::pair<OuterMap::iterator, bool> pairCurr = m_AllPairDistances.insert(std::make_pair(itOutBegin->first, hMap));
		
		InnerMap::iterator itTemp = pairCurr.first->second.find(itOutBegin->first);
		itTemp->second = 0;

		int iNumOfEdges = itOutBegin->second.size();
		itTemp = itOutBegin->second.begin();

		for (int iNumEdge = 0; iNumEdge < iNumOfEdges; iNumEdge++)
		{
			pairCurr.first->second.find(itTemp->first)->second = itTemp->second;
			itTemp++;
		}
	}
	
	// Will rewrite this more effeciently later, does not matter much due to serialization

	typedef std::vector<int> KeyMap;

	for (KeyMap::iterator itLvlk = vecKeys.begin(); itLvlk != vecKeys.end(); itLvlk++)
	{
		for (KeyMap::iterator itLvli = vecKeys.begin(); itLvli != vecKeys.end(); itLvli++)
		{
			for (KeyMap::iterator itLvlj = vecKeys.begin(); itLvlj != vecKeys.end(); itLvlj++)
			{
				InnerMap::iterator it_ij = m_AllPairDistances.find(*itLvli)->second.find(*itLvlj);
				InnerMap::iterator it_ik = m_AllPairDistances.find(*itLvli)->second.find(*itLvlk);
				InnerMap::iterator it_kj = m_AllPairDistances.find(*itLvlk)->second.find(*itLvlj);

				if (it_ij->second > it_ik->second + it_kj->second)
				{
					it_ij->second = it_ik->second + it_kj->second;
				}
			}
		}
	}

	bShortestPaths = true;
}

