#ifndef FIXED_POINT_MANIFOLD_CONTAINER_H
#define FIXED_POINT_MANIFOLD_CONTAINER_H

#include "Manifolds\Base\Manifold_container_base.h"
#include "Manifolds\Fixed_point\Fixed_point_manifold.h"

#include "Utils\Rotation_utils\RotationRange.h"
#include "Utils\Random_utils\RandomUtils.h"
#include "Utils\ReferencePoint.h"
#include "Utils\Geometry_utils\Less_than_point.h"

//-----------------------------------------------------
// a manifold contairer stores manifolds (pointers)
//-----------------------------------------------------

namespace mms{

template <typename Kernel, typename AK, typename AK_conversions>
class Fixed_point_manifold_container: 
  public Manifold_container_base<Fixed_point_manifold<Kernel, AK, AK_conversions> >
{
public:
  typedef Manifold_container_base<Fixed_point_manifold<Kernel, AK, AK_conversions> >  Base;
  typedef typename Fixed_point_manifold_container<Kernel, AK, AK_conversions>         Self;
  typedef typename Fixed_point_manifold<Kernel, AK, AK_conversions>                   Manifold;
  typedef typename Manifold::Fsc                                  Fsc;
  typedef typename Manifold::Constraint                           Constraint;
  typedef typename Constraint::Restriction                        Rotation;

  typedef typename Kernel::Point_2                                Point;
  typedef Reference_point<Kernel>                                 Reference_point;
  typedef Rotation_range_absolute<typename Kernel::FT>            Rotation_range;
  typedef Random_utils<Kernel>                                    Random;  
  
  typedef Less_than_point_2<Kernel>                               Less_than_point;
  
  typedef std::map<Point, int, Less_than_point>   Converter;
  typedef typename Converter::iterator            Converter_iter;
  typedef std::vector<Manifold*>                  Container;
  typedef typename Container::iterator            Container_iter;

private:
  Container fixed_point_manifolds;
  Converter point_to_id;
  int       iterator_id;
public:
  //constructor
  Fixed_point_manifold_container() {};

  virtual int add_manifold(Manifold* manifold_ptr)
  {
    int id = get_new_id();
    CGAL_precondition (id == fixed_point_manifolds.size());
    fixed_point_manifolds.push_back(manifold_ptr);

    Point p = manifold_ptr->constraint().restriction();
    point_to_id.insert(std::make_pair(p, id));
    return id;
  }
  virtual Manifold* get_manifold(int manifold_id)
  {
    CGAL_precondition (manifold_id < (int)fixed_point_manifolds.size());
    return fixed_point_manifolds[manifold_id];
  }

  virtual Manifold* get_manifold(const Constraint& constraint)
  {
    CGAL_precondition (constraint.type() == FIXED_POINT);
    return get_manifold(constraint.restriction());
  }
  virtual void clear()
  {
    for ( Container_iter iter = fixed_point_manifolds.begin();
          iter != fixed_point_manifolds.end();
          ++iter)
    {
      delete (*iter);
    }
  }

  virtual int manifold_id_iterator_begin()
  {
    iterator_id=0;
    return iterator_id;
  }
  virtual int manifold_id_iterator_next()
  {
    iterator_id++;
    return iterator_id;
  }
  virtual int manifold_id_iterator_end()
  {
    return fixed_point_manifolds.size();
  }
public:
  bool contains(const Reference_point& ref_point)
  {
    return (point_to_id.find(ref_point.get_location()) != point_to_id.end());
  }
  std::pair<int,int> get_random_fsc_id(Random& random)
  {
    //Todo chnage implementation
    
    //	pick a random layer
	int rand_manifold_id  = random.get_random_num<int> (0, fixed_point_manifolds.size()-1);
	//	pick a random feature
	int rand_fsc_id       = random.get_random_num<int> (0, fixed_point_manifolds[rand_manifold_id]
                                                              ->num_of_fscs());
	
    return (std::make_pair(rand_manifold_id, rand_fsc_id));
  }
  std::pair<int,int> get_containig_fsc(const Reference_point& ref_point)
  {
    CGAL_precondition(this->contains(ref_point));
    Converter_iter iter = point_to_id.find(ref_point.get_location());
    CGAL_postcondition (iter != point_to_id.end());

    int manifold_id = iter->second;
    int fsc_id = get_manifold(manifold_id)->get_fsc_id(ref_point);

    return std::make_pair(manifold_id,fsc_id);
  }
private:
  Manifold* get_manifold(const Point& p)
  {
    Converter_iter iter = point_to_id.find(p);    
    CGAL_postcondition (iter != point_to_id.end());
    return get_manifold(iter->second);
  }
};  //Fixed_point_manifold_container
} //namespace mms{
#endif // FIXED_POINT_MANIFOLD_CONTAINER_H