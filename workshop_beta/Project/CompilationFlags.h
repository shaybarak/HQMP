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
#define EXACT_READ_MODE	1 //turn off when reading from gui files

//--------------------------------------
// Graphics    
//--------------------------------------
#define USE_GLOBAL_GRAPHICS

//--------------------------------------
// Logging and Playback    
//--------------------------------------
//#define DEBUG_PRINT_TIME_LOG
//#define DEBUG_PLANNER

//#define PLAYBACK_MODE

#endif //COMPILATION_FLAGS_H