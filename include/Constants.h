#ifndef CONSTANTS_H	   
#define CONSTANTS_H	  

#include<limits>

const double MAX_DIST_VALUE = std::numeric_limits<double>::infinity();
const double BAXTER_TRAV_SPEED = .5;         //IN M/SEC
const double BAXTER_PICK_UP_SPEED = 0.15;   // IN M/SEC
const double TIME_TO_PLACE_OBJECT = 5.0;   // IN SECONDS  

const double TIME_TO_PLACE_OBJECT_TABLE_0_SMALL = 5.0;   // IN SECONDS  
const double TIME_TO_PLACE_OBJECT_TABLE_0_MEDIUM = 5.0;   // IN SECONDS  
const double TIME_TO_PLACE_OBJECT_TABLE_0_LARGE = 5.0;   // IN SECONDS  

const double TIME_TO_PLACE_OBJECT_TABLE_1_SMALL = 5.0;   // IN SECONDS  
const double TIME_TO_PLACE_OBJECT_TABLE_1_MEDIUM = 5.0;   // IN SECONDS  
const double TIME_TO_PLACE_OBJECT_TABLE_1_LARGE = 5.0;   // IN SECONDS  

const double TIME_TO_PLACE_OBJECT_TABLE_2_SMALL = 5.0;   // IN SECONDS  
const double TIME_TO_PLACE_OBJECT_TABLE_2_MEDIUM = 5.0;   // IN SECONDS  
const double TIME_TO_PLACE_OBJECT_TABLE_2_LARGE = 5.0;   // IN SECONDS  

const std::string NAMELESS_BLOCK = " ";
const double MAX_DELAY_TIME_PROP_DISTANCE = 0.25;

#endif