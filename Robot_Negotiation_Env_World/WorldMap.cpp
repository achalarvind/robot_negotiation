#include "WorldMap.h"
#include "Constants.h"

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

		if (!(iss >> stVertex >> dVertexNum >> stDummy >> dX , dY))
		{
			break;
		} // error

		obHashVtx.insert(std::make_pair(dVertexNum, std::make_pair(dX, dY)));
	}

	//std::string stEdgeline;
	//while (std::getline(pstEdges, stEdgeline))
	//{
	//	std::istringstream iss(stEdgeline);
	//	std::string stEdge, stDummy;
	//	double dEdgeNum;
	//	double dX, dY;

	//	if (!(iss >> stVertex >> dVertexNum >> stDummy >> dX, dY))
	//	{
	//		break;
	//	} // error

	//	obHashVtx.insert(std::make_pair(dVertexNum, std::make_pair(dX, dY)));
	//}

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

	for (OuterMap::iterator itOutBegin = m_hashWorldMap.begin(); itOutBegin != m_hashWorldMap.end(); itOutBegin++)
	{
		hMap.insert(std::make_pair(itOutBegin->first , MAX_DIST_VALUE));
	}

	for (OuterMap::iterator itOutBegin = m_hashWorldMap.begin(); itOutBegin != m_hashWorldMap.end(); itOutBegin++)
	{
		std::pair<OuterMap::iterator, bool> pairCurr = m_AllPairDistances.insert(std::make_pair(itOutBegin->first, hMap));
		InnerMap::iterator itTemp = pairCurr.first->second.find(itOutBegin->first);
		itTemp->second = 0;
	}

	for (OuterMap::iterator itLvl1 = m_AllPairDistances.begin(); itLvl1 != m_AllPairDistances.end(); itLvl1++)
	{
		for (OuterMap::iterator itLvl2 = m_AllPairDistances.begin(); itLvl2 != m_AllPairDistances.end(); itLvl2++)
		{
			InnerMap::iterator itLvl1Curr = itLvl1->second.find(itLvl2->first);
			
			for (OuterMap::iterator itLvl3 = m_AllPairDistances.begin(); itLvl3 != m_AllPairDistances.end(); itLvl3++)
			{
				if (itLvl1Curr->second > itLvl1->second.find(itLvl3->first)->second + itLvl3->second.find(itLvl2->first)->second)
				{
					itLvl1Curr->second = itLvl1->second.find(itLvl3->first)->second + itLvl3->second.find(itLvl2->first)->second;
				}
			}
		}
	}
}

