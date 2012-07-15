#ifndef FIXED_POINT_MANIFOLD_H
#define FIXED_POINT_MANIFOLD_H

#include "Manifolds\Base\Manifold_base.h"
#include "Manifolds\Fixed_point\Fixed_point_constraint.h"
#include "Manifolds\Fixed_point\Fixed_point_fsc.h"  
#include "Configuration_spaces\PointPrimitive\ConfigurationSpace.h"

namespace mms{

template <typename Kernel, typename AK, typename AK_conversions>
class Fixed_point_manifold : 
  public Manifold_base <Kernel,                                       //Kernel
                        Fixed_point_fsc<Kernel, AK, AK_conversions>,  //FSC
                        Fixed_point_constraint<Kernel> >              //Constraint
{
public:
  typedef Manifold_base <Kernel,                            
                        Fixed_point_fsc<Kernel, AK, AK_conversions>,
                        Fixed_point_constraint<Kernel> >              Base;
  typedef Fixed_point_manifold<Kernel, AK, AK_conversions>            Self;
  typedef Fixed_point_fsc<Kernel, AK, AK_conversions>                 Fsc;
  typedef Fixed_point_constraint<Kernel>                              Constraint;
  typedef Kernel                                                      K;

  typedef FixedPoint::Configuration_space<Kernel, AK, AK_conversions> C_space;
private:
  typedef std::map<int , Fsc*>           Cache; //created fscs
private:

  C_space _configuration_space;
  Cache   _cache;
public:
//constructor
  Fixed_point_manifold (const Constraint& constraint, AK& ak)
    :Base(constraint), _configuration_space(ak)
  {}
  ~Fixed_point_manifold ()
  {
    for (Cache::iterator iter = _cache.begin() ; iter != _cache.end(); ++iter)
      delete (iter->second);
  }
  virtual void decompose( Extended_polygon& robot, const Polygon_vec& workspace)
  {
    CGAL_precondition (_constraint.is_legal());
    if (_constraint.is_roi_defined())
    {
      _configuration_space.decompose(robot, workspace,
                                     _constraint.restriction(),
                                     _constraint.region_of_interest());
    }
    else
    {
      _configuration_space.decompose(robot, workspace,
                                     _constraint.restriction());
    }
  }
  void add_obstacle(Extended_polygon& robot, Polygon& obstacle)
  {
    CGAL_precondition (_constraint.is_legal());
    clear_cache();
    _configuration_space.add_obstacle(robot, obstacle);
    
  }
  void copy(const Self &other, bool copy_cache) 
  {
    if (this == &other) // protect against invalid self-assignment
      return ;
 
    // 1: clear cache
    clear_cache();
 
    // 2: allocate new memory and copy the elements    
    if (copy_cache)
    {
      for (Cache::const_iterator iter = other.cache().begin() ; iter != other.cache().end(); ++iter)
      {
        Fsc::Cell cell(iter->second->cell());
        Fsc* fsc_ptr = new Fsc(cell);
        _cache.insert(std::make_pair(iter->first, fsc_ptr));
      }
    }
      
    // 3: assign the new memory to the object
    _configuration_space.copy(other.configuration_space());
    if (other.constraint().is_roi_defined())
      _constraint  = Constraint(other.constraint().restriction(),
                                other.constraint().region_of_interest()); 
    else //no RoI is defined
      _constraint  = Constraint(other.constraint().restriction()); 

    return ;
  }
  virtual Fsc&  get_fsc(const int fsc_id)
  {
    Fsc* fsc_ptr;

    //try to look in the cache
    Cache::iterator iter = _cache.find(fsc_id);
    if (iter != _cache.end())
    {
      //cache hit
      fsc_ptr = iter->second;
    }
    else
    {
      //contstruct fsc
      C_space::Interval_set interval(true);
      _configuration_space.get_interval(fsc_id, interval);
      fsc_ptr = new Fsc(interval);
      
      //update cache
      _cache[fsc_id] = fsc_ptr;
    }      
    
    return *fsc_ptr;
  }
  virtual int free_space_location_hint (const Reference_point& ref_point) 
  {
    return _configuration_space.free_space_location_hint (ref_point);
  }
  const Cache& cache() const
  {
    return _cache;
  }
  const C_space& configuration_space() const
  {
    return _configuration_space;
  }
private:
  void clear_cache()
  {
    for (Cache::iterator iter = _cache.begin() ; iter != _cache.end(); ++iter)
      delete (iter->second);
    _cache.clear();
  }
};  //Fixed_point_manifold

} //mms
#endif //FIXED_POINT_MANIFOLD_H