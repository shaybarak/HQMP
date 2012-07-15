#ifndef FIXED_ANGLE_MANIFOLD_H
#define FIXED_ANGLE_MANIFOLD_H

#include "Manifolds\Base\Manifold_base.h"
#include "Manifolds\Fixed_angle\Fixed_angle_constraint.h"
#include "Manifolds\Fixed_angle\Fixed_angle_fsc.h"  
#include "Utils\Polygon_utils\Smart_polygon_with_holes.h"
#include "Configuration_spaces\AnglePrimitive\ConfigurationSpace.h"

namespace mms{

template <typename Kernel>
class Fixed_angle_manifold : 
  public Manifold_base <Kernel,                             //Kernel
                        Fixed_angle_fsc<Kernel>,            //FSC
                        Fixed_angle_constraint<Kernel> >    //Constraint
{
public:
  typedef Manifold_base <Kernel,                            
                        Fixed_angle_fsc<Kernel>,
                        Fixed_angle_constraint<Kernel> >    Base;
  typedef Fixed_angle_manifold<Kernel>                      Self;
  typedef Fixed_angle_fsc<Kernel>                           Fsc;
  typedef Fixed_angle_constraint<Kernel>                    Constraint;
  typedef Kernel                                            K;

  typedef FixedRotation::Configuration_space<Kernel>        C_space;
private:
  typedef std::map<int , Fsc*>           Cache; //created fscs
private:

  C_space _configuration_space;
  Cache   _cache;
public:
//constructor
  Fixed_angle_manifold (const Constraint& constraint)
    :Base(constraint){}
  ~Fixed_angle_manifold ()
  {
    for (Cache::iterator iter = _cache.begin() ; iter != _cache.end(); ++iter)
      delete (iter->second);
  }
  virtual void decompose(Extended_polygon& robot, const Polygon_vec& workspace)
  {
    robot.move_origin();
    robot.rotate_absolute(_constraint.restriction());

    //compute configuration space of robot under rotation
    _configuration_space.decompose(robot.get_absolute_polygon(), workspace);
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
      fsc_ptr = new Fsc(_configuration_space.get_fsc(fsc_id));
      
      //update cache
      _cache[fsc_id] = fsc_ptr;
    }      
    
    return *fsc_ptr;
  }
  virtual int  get_fsc_id(const Reference_point& ref_point)
  {
    return _configuration_space.get_fsc_id(ref_point.get_location());
  }
  virtual int num_of_fscs() const
  {
    return _configuration_space.num_of_fscs();
  }

public:
  const Cache& cache() const
  {
    return _cache;
  }
  const C_space& configuration_space() const
  {
    return _configuration_space;
  }
  void copy(const Self &other, bool copy_cache) 
  {
    if (this == &other) // protect against invalid self-assignment
      return ;
 
    // 1: clear cache
    for (Cache::iterator iter = _cache.begin() ; iter != _cache.end(); ++iter)
      delete (iter->second);
    _cache.clear();
 
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
    _configuration_space = other.configuration_space();
    _constraint  = Constraint(other.constraint().restriction()); //no RoI is defined
    return ;
  }
  void add_c_space_obstacle(typename C_space::Polygon_with_holes& c_space_obstacle)
  {
    _configuration_space.add_c_space_obstacle(c_space_obstacle);

    //clear cache...
    _cache.clear();
  }
  void add_obstacle(Extended_polygon& robot, Polygon& obstacle)
  {
    robot.move_origin();
    robot.rotate_absolute(_constraint.restriction());
    _configuration_space.add_obstacle(robot.get_absolute_polygon(), obstacle);

    //clear cache...
    _cache.clear();
  }
public:
  bool is_free(const typename Kernel::Point_2& p)
  {
    return _configuration_space.is_free(p);
  }
  int get_containing_cell (const typename Kernel::Point_2& p) 
  {
    return _configuration_space.get_fsc_id(p);
  }  
  template <typename OutputIterator>
  void get_points_in_intersecting_cells(const typename Kernel::Segment_2& seg, 
                                        OutputIterator& oi) 
  {
    //intersects seg with all manifold cells
    //foreach intersecting cell returns a pair (point on intersection, cell id)

    //Todo (inefficient implementation)
    for (int fsc_id = 0; fsc_id < num_of_fscs(); ++fsc_id)
    {
      Point p;
      if (get_fsc(fsc_id).cell().point_in_intersection(seg,p))
      {
        oi = std::make_pair(p,fsc_id);
        ++oi;
      }           
    }  
    return;
  }
  const typename C_space::Polygon_set& get_free_space() const
  {
    return _configuration_space.get_free_space();
  }
};  //Fixed_angle_manifold

} //mms
#endif //FIXED_ANGLE_MANIFOLD_H