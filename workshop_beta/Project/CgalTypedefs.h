#ifndef CGAL_TYPEDEFS_H
#define CGAL_TYPEDEFS_H

#include "includes.h"
#include "Project\CompilationFlags.h"

/////////////////
// Kernel      //
/////////////////
#ifdef PURE_RATIONAL_USED
typedef CGAL::Simple_cartesian<CGAL::CORE_arithmetic_kernel::Rational>				          Rational_kernel;
//typedef CGAL::Lazy_kernel<CGAL::Simple_cartesian<CGAL::CORE_arithmetic_kernel::Rational> >  Rational_kernel;
typedef Rational_kernel::FT                                                                   Number_type;
#endif //#ifdef PURE_RATIONAL_USED

#endif  //CGAL_TYPEDEFS_H