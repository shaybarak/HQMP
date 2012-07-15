#ifndef REFERENCE_POINT_T_H
#define REFERENCE_POINT_T_H

#include "Utils\Rotation_utils\Rotation.h"

template <typename _K>
class Reference_point
{
public:
  typedef _K                                K;
  typedef typename K::Point_2               Point;
  typedef typename Rotation<typename K::FT> Rotation;
public:
  //constructors
  Reference_point(){}
  Reference_point(Point p, Rotation r)
    : _p(p), _r(r) 
  {
    CGAL_precondition(r.has_complete_rotation() == false);
  }
  Reference_point(const Reference_point<K>& other) 
    : _p(other.get_location()), _r(other.get_rotation()) {}
  
  //assignment operator
  Reference_point<K>& operator= (const Reference_point<K> &other)
  {
    if (this != &other) 
    {
      _p = other.get_location();
      _r = other.get_rotation();
    }
    return *this;
  }

  bool operator== (const Reference_point<K> &other) const
  {
    return ((_p == other.get_location()) &&
            (_r == other.get_rotation()) );
  }
  bool operator!= (const Reference_point<K> &other) const
  {
    return ((_p.x() != other.get_location().x()) ||
            (_p.y() != other.get_location().y()) ||
            (_r != other.get_rotation()) );
  }
  //get
  Point get_location() const
  {
    return _p;
  }
  const Rotation& get_rotation() const
  {
    return _r;
  }

  //set
  void set_location(const Point& p)
  {
    _p = p;
    return;
  }
  void set_rotation(const Rotation& r)
  {
    CGAL_precondition(r.has_complete_rotation() == false);
    _r = r;
    return;
  }

  //print
  void print() const
  {
    std::cout << "[";
    print_point_nice<K>(_p);
    std::cout << " , ";
    _r.print_nice();
    std::cout << "]";
  }
  void write(std::ostream& os)
  {
    os  <<  _p.x() << std::endl
        <<  _p.y() << std::endl
        <<  _r.sin() << std::endl
        <<  _r.cos() << std::endl;
    return;
  }
  void read(std::istream& is)
  {
    typename K::FT  s_x, s_y, s_sin, s_cos;
    is >> s_x >> s_y >> s_sin >> s_cos;

    _p = Point(s_x, s_y);
    _r = Rotation(s_sin, s_cos);
    return;
  }

private:
  Point    _p;    //location
  Rotation _r;    //rotation
}; //Reference_point


template <typename K>
struct Less_than_reference_point
{
  bool operator()(const Reference_point<K>& p1,const Reference_point<K>& p2) const 
  {
    CGAL::Comparison_result cr = CGAL::compare_xy(p1.get_location(),
                                                  p2.get_location());
    if (cr != CGAL::EQUAL)
      return (cr == CGAL::SMALLER) ? true : false;

    //(cr == CGAL::EQUAL)
    const Rotation<typename K::FT>& r1 = p1.get_rotation();
    const Rotation<typename K::FT>& r2 = p2.get_rotation();
    return (( r1.is_larger_than(r2) || (r1 == r2)) ?
            false:
            true);
  }
}; //Less_than_reference_point
#endif REFERENCE_POINT_T_H