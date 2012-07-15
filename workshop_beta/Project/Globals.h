#ifndef GLOBALS_H
#define GLOBALS_H

#include "Project\Includes.h"
#include "Project\CgalTypedefs.h"
/////////////////////
// const variables //
/////////////////////
const int SCREEN_SIZE = 100;
const double PI = 3.141592653589793;
const double SMALL_STEP = 0.008;  //used to define reference point
const double CROSS_SIZE = 0.025;


#ifndef PURE_RATIONAL_USED
const double EPS_NUM = 1;
const double EPS_DEN_CRUDE = 20;	//crude approximation (step is arround 2 degrees), 
const double EPS_DEN_CRUDE_CRUDIER = 1;
const double SMALL_SINE_CRUDE(double(218)/double(11882));		//angle = 1.05127, crude, use with EPS_DEN = 20
const double SMALL_COSINE_CRUDE(double(11880)/double(11882));   //angle = 1.05127, crude, use with EPS_DEN = 20
const double SMALL_SINE_FINE(double(218)/double(11882));		//angle = 1.05127, crude, use with EPS_DEN = 20
const double SMALL_COSINE_FINE(double(218)/double(11882));      //angle = 1.05127, crude, use with EPS_DEN = 20
const double EPS_DEN_FINE = 1000;	//good approximation (step is arround 0.005 degrees)
const double EPS_DEN_FINE_CRUDIER = 50;
#else //PURE_RATIONAL_USED
const Number_type EPS_NUM = 1;
const Number_type EPS_DEN_CRUDE = 20;	//crude approximation (step is arround 2 degrees), 
const Number_type EPS_DEN_CRUDE_CRUDIER = 1;
const Number_type SMALL_SINE_CRUDE(218,11882);		            //angle = 1.05127, crude, use with EPS_DEN = 20
const Number_type SMALL_COSINE_CRUDE(11880,11882);              //angle = 1.05127, crude, use with EPS_DEN = 20
const Number_type SMALL_SINE_FINE(218,11882);		            //angle = 1.05127, crude, use with EPS_DEN = 20
const Number_type SMALL_COSINE_FINE(11880,11882);               //angle = 1.05127, crude, use with EPS_DEN = 20
const Number_type EPS_DEN_FINE = 75;	//good approximation (step is arround 0.005 degrees)
const Number_type EPS_DEN_FINE_CRUDIER = 50;
#endif //PURE_RATIONAL_USED

////////////////////
// debugging      //
////////////////////
const double INFINITY = 10000000;


#endif