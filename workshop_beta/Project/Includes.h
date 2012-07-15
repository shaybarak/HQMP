#ifndef INCLUDES_H
#define INCLUDES_H

//
//#include "stdafx.h"
//#include <windows.h>

//////////////////
// General CGAL //
//////////////////
#include <CGAL/basic.h>
#include <CGAL/enum.h>
#include <CGAL/Timer.h>
#include <CGAL/intersections.h>


//////////////////
// Numbers      //
//////////////////
#include <CGAL/Gmpq.h>
#include <CGAL/Quotient.h> 
#include <CGAL/MP_Float.h>
#include <CGAL/number_utils.h>
#include <CGAL/Root_of_traits.h>
#include <CGAL/Sqrt_extension.h>
#include <CGAL/Lazy_exact_nt.h>

#include <CGAL/CORE_algebraic_number_traits.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Arithmetic_kernel.h>
#include <CGAL/Algebraic_kernel_d_1.h>


//////////////////
// Kernel       //
//////////////////
#include <CGAL/Cartesian.h>
#include <CGAL/Simple_cartesian.h> 
#include <CGAL/Filtered_kernel.h>
#include <CGAL/Lazy_kernel.h>

//////////////
// STL      //
//////////////
#include <utility>
#include <string>
#include <vector>
#include <set>

//////////////
//boost		//
//////////////
#include <boost/foreach.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/filtered_graph.hpp>

/////////
// I/O //
/////////
#include <iostream>
#include <fstream>
#include <sstream>

#include <math.h>

#endif