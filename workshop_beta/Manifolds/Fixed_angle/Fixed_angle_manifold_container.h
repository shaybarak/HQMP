#ifndef FIXED_ANGLE_MANIFOLD_CONTAINER_H
#define FIXED_ANGLE_MANIFOLD_CONTAINER_H

#include "Manifolds\Base\Manifold_container_base.h"
#include "Manifolds\Fixed_angle\Fixed_angle_manifold.h"

#include "Utils\Rotation_utils\RotationRange.h"
#include "Utils\Random_utils\RandomUtils.h"
#include "Utils\ReferencePoint.h"

//-----------------------------------------------------
// a manifold contairer stores manifolds (pointers)
//-----------------------------------------------------

namespace mms{

template <typename Kernel>
class Fixed_angle_manifold_container: 
  public Manifold_container_base<Fixed_angle_manifold<Kernel> >
{
public:
  typedef Manifold_container_base<Fixed_angle_manifold<Kernel> >  Base;
  typedef typename Fixed_angle_manifold_container<Kernel>         Self;
  typedef typename Fixed_angle_manifold<Kernel>                   Manifold;
  typedef typename Manifold::Fsc                                  Fsc;
  typedef typename Manifold::Constraint                           Constraint;
  typedef typename Constraint::Restriction                        Rotation;

  typedef Reference_point<Kernel>                                 Reference_point;
  typedef Rotation_range_absolute<typename Kernel::FT>            Rotation_range;
  typedef Random_utils<Kernel>                                    Random;  
  
  typedef Less_than_rotation<typename Kernel::FT>   Less_than;
  typedef std::map< Rotation, int, Less_than>       Converter;
  typedef typename Converter::iterator              Converter_iter;
  typedef std::vector<Manifold*>                    Container;
  typedef typename Container::iterator              Container_iter;

private:
  Container fixed_angle_manifolds;
  Converter angle_to_id;
  int       iterator_id;
public:
  //constructor
  Fixed_angle_manifold_container (){};

  virtual int add_manifold(Manifold* manifold_ptr)
  {
    int id = get_new_id();
    CGAL_precondition (id == fixed_angle_manifolds.size());
    fixed_angle_manifolds.push_back(manifold_ptr);

    Rotation rotation = manifold_ptr->constraint().restriction();
    angle_to_id.insert(std::make_pair(rotation,id));
    return id;
  }
  virtual Manifold* get_manifold(int manifold_id)
  {
    CGAL_precondition (manifold_id < (int)fixed_angle_manifolds.size());
    return fixed_angle_manifolds[manifold_id];
  }

  virtual Manifold* get_manifold(const Constraint& constraint)
  {
    CGAL_precondition (constraint.type() == FIXED_ANGLE);
    return get_manifold(constraint.restriction());
  }
  virtual void clear()
  {
    for ( Container_iter iter = fixed_angle_manifolds.begin();
          iter != fixed_angle_manifolds.end();
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
    return fixed_angle_manifolds.size();
  }
public:
  int get_containing_manifold_id(const Rotation& rotation)
  {
    Converter_iter iter = angle_to_id.find(rotation);
    if (iter == angle_to_id.end())
      return NO_ID;
    return iter->second;
  }
  std::pair<int,int> get_adjacent_layer_ids(int manifold_id)
  {
    const Rotation& rotation = this->get_manifold(manifold_id)->constraint().restriction();
    Converter_iter iter = angle_to_id.find(rotation);
    CGAL_postcondition (iter != angle_to_id.end());

    //get id of manifold before current manifold
    int id_before;
    if (iter == angle_to_id.begin())
    {
      Converter_iter last = angle_to_id.end();
      last--;   
      id_before = last->second;
    }
    else
    {
      --iter;
      id_before = iter->second;
      ++iter;
    }

    //get id of manifold after current manifold
    int id_after;
    ++iter;
    if (iter == angle_to_id.end())
      id_after = angle_to_id.begin()->second;
    else
      id_after = iter->second;

    return std::make_pair(id_before,id_after);    
  }
  bool contains(const Reference_point& ref_point)
  {
    return (angle_to_id.find(ref_point.get_rotation()) != angle_to_id.end());
  }
  int get_closest_layer_id(const Rotation& rotation)
  {
    Converter_iter iter = angle_to_id.lower_bound(rotation);
    if (iter == angle_to_id.end())
      return angle_to_id.begin()->second;
    else
      return iter->second;
  }
  template <typename OutputIterator>
  void get_interseceting_manifolds(const Rotation_range& range, OutputIterator& oi)
  {
    if (range.is_full_range())
    {
      for (unsigned int id(0); id<fixed_angle_manifolds.size(); ++id)
      {
        *oi = id;
        ++oi;
      }
      return;
    }

    Converter_iter lower = angle_to_id.lower_bound(range.get_start_rotation());
    Converter_iter upper = angle_to_id.upper_bound(range.get_end_rotation());

    if (range.get_end_rotation().is_larger_than(range.get_start_rotation()))
    {
      //from lower to upper
      for (Converter_iter  iter = lower; iter != upper; ++iter)
      {
        *oi = iter->second;
        ++oi;
      }
    }
    else
    {
      for (Converter_iter  iter = lower; iter != angle_to_id.end(); ++iter)
      {
        *oi = iter->second;
        ++oi;
      }
      
      //from start to lower
      for (Converter_iter  iter = angle_to_id.begin(); iter != upper; ++iter)
      {
        *oi = iter->second;
        ++oi;
      }
    }
    return;
  }
  std::pair<int,int> get_random_fsc_id(Random& random)
  {
    //Todo chnage implementation
    static bool fsc_exist = false;
    static bool check_if_fsc_exist = true;
    if (check_if_fsc_exist)
    {
      check_if_fsc_exist = false;
      BOOST_FOREACH(Manifold* manifold_ptr, fixed_angle_manifolds)
      {
        if (manifold_ptr->num_of_fscs()>=1)
        {
          fsc_exist = true;
          break;
        }
      }
    }

    if (fsc_exist == false)
      return std::make_pair(NO_ID, NO_ID);

    //	pick a random layer
	int rand_manifold_id  = random.get_random_num<int> (0,fixed_angle_manifolds.size()-1);
    //check if a feature exists - we need at least one
    while (fixed_angle_manifolds[rand_manifold_id]->num_of_fscs()<1) 
      rand_manifold_id  = random.get_random_num<int> (0,fixed_angle_manifolds.size()-1);
	//	pick a random feature
	int rand_fsc_id       = random.get_random_num<int> (0,fixed_angle_manifolds[rand_manifold_id]
                                                              ->num_of_fscs());
	
    return (std::make_pair(rand_manifold_id,rand_fsc_id));
  }
  std::pair<int,int> get_containig_fsc(const Reference_point& ref_point)
  {
    CGAL_precondition(this->contains(ref_point));
    Converter_iter iter = angle_to_id.find(ref_point.get_rotation());
     CGAL_postcondition (iter != angle_to_id.end());

    int manifold_id = iter->second;
    int fsc_id = get_manifold(manifold_id)->get_fsc_id(ref_point);

    return std::make_pair(manifold_id,fsc_id);
  }

public: //dbg
  int size() {return fixed_angle_manifolds.size();}
public:
  Manifold* get_manifold(const Rotation& rotation)
  {
    Converter_iter iter = angle_to_id.find(rotation);    
    CGAL_postcondition (iter != angle_to_id.end());
    return get_manifold(iter->second);
  }
};  //Fixed_angle_manifold_container
} //namespace mms{
#endif // FIXED_ANGLE_MANIFOLD_CONTAINER_H