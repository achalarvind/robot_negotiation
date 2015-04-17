// #define BOOST_HAS_unordered_map

// #include<vector>
// #include <unordered_map_map>
// #include <boost/serialization/base_object.hpp>
// #include <boost/serialization/vector.hpp>
// #include <boost/archive/text_oarchive.hpp>
// #include <boost/archive/text_iarchive.hpp> 
// #include <boost/serialization/unordered_map_map.hpp>

// class Temp
// {
// public:
// 		friend class boost::serialization::access;
	
// 		//std::vector<int> m_IntVector;
// 		//std::vector<double> m_DoubleVector;

// 		std::unordered_map_map<int, int> m_Intunordered_map;
// 		std::unordered_map_map<int, double> m_Doubleunordered_map;

// 		void serialize(boost::archive::text_iarchive & ar, const unsigned int file_version);
// 		void serialize(boost::archive::text_oarchive & ar, const unsigned int file_version);

// 	public:	
// 		void Initialize();
		
// };