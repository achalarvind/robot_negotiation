#include"Temp.h"

/*void Temp::Initialize()
{
	for (int iCount = 0; iCount < 10; iCount++)
	{
		m_IntVector.push_back(10);
		m_DoubleVector.push_back(10);
	}
}*/

void Temp::Initialize()
{
	for (int iCount = 0; iCount < 10; iCount++)
	{
		m_IntHash.insert(std::make_pair(iCount , 2*iCount));
		m_DoubleHash.insert(std::make_pair(iCount, 2.0 * iCount));
	}
}

/*void Temp::serialize(boost::archive::text_iarchive & ar, const unsigned int file_version) 
{
	ar & m_IntVector;
	ar & m_DoubleVector;
}

void Temp::serialize(boost::archive::text_oarchive & ar, const unsigned int file_version) 
{
	ar & m_IntVector;
	ar & m_DoubleVector;
}*/


void Temp::serialize(boost::archive::text_iarchive & ar, const unsigned int file_version)
{
	ar & m_IntHash;
	ar & m_DoubleHash;
}

void Temp::serialize(boost::archive::text_oarchive & ar, const unsigned int file_version)
{
	ar & m_IntHash;
	ar & m_DoubleHash;
}


