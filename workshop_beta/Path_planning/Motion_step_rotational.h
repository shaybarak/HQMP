#ifndef MOTION_STEP_ROTATIONAL_H
#define MOTION_STEP_ROTATIONAL_H

#include "Path_planning\Motion_step_base.h"
#include "Utils\ReferencePoint.h"

template <typename K>
class Motion_step_rotational : public Motion_step_base<K>
{
public:
  typedef Motion_step_base<K>           Base;
  typedef Motion_step_rotational<K>     Self;
  typedef Base::Reference_point         Reference_point;
  typedef typename K::Point_2           Point;  
  typedef Rotation<typename K::FT>      Rotation;

private:
  Reference_point     s,t;
  CGAL::Orientation   o;
public:
  Motion_step_rotational () : Base(UNINITIALIZED)
  {}
  Motion_step_rotational (const Reference_point& source,
                          const Reference_point& target,
                          CGAL::Orientation orientation)
                          :Base(ROTATION), s(source), t(target), o(orientation)
  {
    CGAL_precondition(is_legal_motion());
  }
  Motion_step_rotational (const Point& p,
                          const Rotation& r_s,
                          const Rotation& r_t,
                          CGAL::Orientation orientation)
                          :Base(ROTATION), s(p, r_s), t(p, r_t),  o(orientation)
  {
    CGAL_precondition(is_legal_motion());
  }
  Motion_step_rotational(const Self& other)
      :Base(ROTATION)
  {
      if (&other == this)
          return;
      s = other.source();
      t = other.target();
      o = other.orientation();
      return;
  }  
public:
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

  CGAL::Orientation orientation() const
  {
    CGAL_precondition (motion_type != UNINITIALIZED);
    return o;
  }
public:
  void reverse_motion_step()
  {
      std::swap(s,t);
      o = (o == CGAL::CLOCKWISE) ? CGAL::COUNTERCLOCKWISE : CGAL::CLOCKWISE; //swap orientation
      return;
  }
  double get_angular_difference_in_deg()
  {
    CGAL_precondition (motion_type != UNINITIALIZED);

    double s = source().get_rotation().to_angle();
    double t = target().get_rotation().to_angle();

    double d; //angular difference
    CGAL::Comparison_result cr = CGAL::compare(s,t);
    switch( cr )
    {
    case (CGAL::EQUAL) : //s == t
      {
        d = 0;
        break;
      }
    case (CGAL::SMALLER) : //s < t
      {
        if (o == CGAL::CLOCKWISE)
          d = s - t + 360;
        else // o == CGAL::COUNTERCLOCKWISE
          d = t - s;
        break;
      }
    case (CGAL::LARGER) : // s > t
      {
        if (o == CGAL::CLOCKWISE)
          d = s - t;
        else // o == CGAL::COUNTERCLOCKWISE
          d = t - s + 360;
        break;
      }
    }
    return d;    
  }
  double approximate_motion_time(double rotational_speed) //full rotation per second
  {
    double d = get_angular_difference_in_deg();
    CGAL_postcondition( d >=0 && d <360);
    double time = d / rotational_speed;
    return time;      
  }
public:
  void read(std::istream& is)
  {
    motion_type = ROTATION;

    s.read(is);
    t.read(is);

    std::string str;;
    is >> str;
    CGAL_postcondition (str == "CW" || str == "CCW");

    if (str == "CW")
      o = CGAL::CLOCKWISE;
    else // (str == CCW)
      o = CGAL::COUNTERCLOCKWISE;

    return;
  }
  void write(std::ostream& os)
  {
    CGAL_precondition (motion_type != UNINITIALIZED);

    s.write(os);
    t.write(os);
    if (o == CGAL::CLOCKWISE)
      os << "CW" <<std::endl;
    else
      os << "CCW" <<std::endl;
    return;
  }
private:
  bool is_legal_motion()
  {
    CGAL_precondition(motion_type == ROTATION);    
    bool b = (s.get_location() == t.get_location());

    return (b);
  }
}; //Motion_step_rotational 
#endif //MOTION_STEP_ROTATIONAL_H