#ifndef COMPILATION_FLAGS_H
#define COMPILATION_FLAGS_H
//--------------------------------------
//compilation flags - number types
//--------------------------------------
//#define DOUBLE_USED
#define PURE_RATIONAL_USED

//--------------------------------------
//compilation flags - kernel
//--------------------------------------

//--------------------------------------
//compilation flags - minkowski sum
//--------------------------------------
//minkowski sum without decomposition method has some bugs
//use this flag!
#define MINKWOSKI_WITH_DECOMPOSITION 

//decomposition method
#define ANGLE_BISECTOR
//#define HERTEL_MELHORN


//--------------------------------------
// Input	   
//--------------------------------------
#define ANGLES_IN_RADIANS	//used when reading from OOPSMP files

//--------------------------------------
// Graphics    
//--------------------------------------
#define USE_GLOBAL_GRAPHICS
#define DEBUG_PRINT_TIME_LOG

#endif //COMPILATION_FLAGS_H