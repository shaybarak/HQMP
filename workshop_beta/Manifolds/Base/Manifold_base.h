#ifndef MANIFOLD_BASE
#define MANIFOLD_BASE

#include "Manifolds\MMSTypedefs.h"
#include "Utils\Polygon_utils\ExtendedPolygon.h"
#include "Utils\ReferencePoint.h"

//-----------------------------------------------------
// a manifold is part of the full configuration space
// it is defined by a constraint and can be decomposed 
// into a set of free space cells.
// each cell internally should be represented by an id
// upon user request the cell may be constructed explicitly
// if so it is cached
//-----------------------------------------------------

namespace mms{

template <typename Kernel,typename Fsc,typename Constraint>
class Manifold_base
{
public:
  typedef Fsc                     Fsc;
  typedef Constraint              Constraint;
protected:
  typedef Manifold_base<Kernel,Fsc,Constraint>   Base;
public:
  typedef CGAL::Polygon_2 <Kernel>      Polygon;
  typedef std::vector<Polygon>          Polygon_vec;
  typedef Extended_polygon<Kernel>      Extended_polygon;
  typedef Reference_point<Kernel>       Reference_point;
protected:
  Constraint  _constraint; //constraint defining the manifols
public:
  //constructor
  Manifold_base( const Constraint& constraint = Constraint())
    : _constraint(constraint) 
  {}
  virtual void decompose(Extended_polygon& robot,const Polygon_vec& workspace) =0;
  
  //accessors
  const Constraint& constraint() const 
  {
    return _constraint;
  }
  
  //fsc utils
  virtual Fsc&  get_fsc(const int fsc_id) =0;
  virtual int get_fsc_id(const Reference_point& ref_point)
  {
    //if not needed, shouldn't be called
    CGAL_assertion (false); assert (false);
    return NO_ID;
  }
  virtual int free_space_location_hint (const Reference_point& ref_point) 
  {
    //if not needed, shouldn't be called
    CGAL_assertion (false); assert (false);
    return NO_ID;
  }
  virtual int num_of_fscs() const
  {
    //if not needed, shouldn't be called
    CGAL_assertion (false); assert (false);
    return NO_ID;
  }
};  //Manifold_base
} //namespace mms{
#endif // MANIFOLD_BASE