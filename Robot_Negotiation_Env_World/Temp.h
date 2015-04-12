#define BOOST_HAS_HASH

#include<vector>
#include <hash_map>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp> 
#include <boost/serialization/hash_map.hpp>

class Temp
{
public:
		friend class boost::serialization::access;
	
		//std::vector<int> m_IntVector;
		//std::vector<double> m_DoubleVector;

		std::hash_map<int, int> m_IntHash;
		std::hash_map<int, double> m_DoubleHash;

		void serialize(boost::archive::text_iarchive & ar, const unsigned int file_version);
		void serialize(boost::archive::text_oarchive & ar, const unsigned int file_version);

	public:	
		void Initialize();
		
};