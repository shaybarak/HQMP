#ifndef MOTION_STEP_TRANSLATIONAL_H
#define MOTION_STEP_TRANSLATIONAL_H

#include "Path_planning\Motion_step_base.h"
#include "FSC.h"

template <typename K>
class Motion_step_translational : public Motion_step_base<K>
{
public:
  typedef Motion_step_base<K>           Base;
  typedef Motion_step_translational<K>  Self;
  typedef Base::Reference_point         Reference_point;
  typedef typename K::Point_2           Point;  
  typedef Rotation<typename K::FT>      Rotation;
  typedef Smart_polygon_with_holes<K>	Cell;

private:
  Reference_point	s, t;
  Cell*				c;

public:
  Motion_step_translational () : Base(UNINITIALIZED)
  {}
  
  Motion_step_translational ( const Reference_point& source,
                              const Reference_point& target,
							  Cell* cell)
                              :Base(TRANSLATION), s(source), t(target), c(cell)
  {
    CGAL_precondition(is_legal_motion());
  }
  
  Motion_step_translational(const Point& source,
                            const Point& target,
                            const Rotation& rotation,
							Cell* cell)
                            :Base(TRANSLATION), s(source, rotation), t(target, rotation), c(cell)
  {
    CGAL_precondition(is_legal_motion());
  }
  
  Motion_step_translational(const Self& other)
      :Base(TRANSLATION)
  {
      if (&other == this)
          return;
      s = other.s;
      t = other.t;
	  c = other.c;
      return;
  }  

  Reference_point source() const 
  {
    CGAL_precondition (motion_type != UNINITIALIZED);
    return s;
  }
  
  Reference_point target() const 
  {
    CGAL_precondition (motion_type != UNINITIALIZED);
    return t;
  }

  Cell* cell() const {
	  return c;
  }

  double approximate_motion_time(double translational_speed) //unit per second
  {
    CGAL_precondition (motion_type != UNINITIALIZED);

    Sqrt_approximation<typename K::FT> sqrt_approximation;
    K::FT d_sq = CGAL::squared_distance(source().get_location(),
                                        target().get_location());
    double d = CGAL::to_double(sqrt_approximation(d_sq));
      
    //t = x / v
    double  t= d / translational_speed;
    return t;      
  }
  
  void reverse_motion_step()
  {
      std::swap(s,t);
      return;
  }

  void read(std::istream& is)
  {
    motion_type = TRANSLATION;

    s.read(is);
    t.read(is);
    
    return;
  }
  
  void write(std::ostream& os)
  {
    CGAL_precondition (motion_type != UNINITIALIZED);

    s.write(os);
    t.write(os);
    return;
  }

private:
  bool is_legal_motion()
  {
    CGAL_precondition(motion_type == TRANSLATION);
    bool b = (s.get_rotation() == t.get_rotation());
    return (b);
  }  
}; //Motion_step_translational
#endif //MOTION_STEP_TRANSLATIONAL_H