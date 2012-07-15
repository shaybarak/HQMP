#ifndef MOTION_STEP_STATIC_H
#define MOTION_STEP_STATIC_H

#include "Path_planning\Motion_step_base.h"
#include "Utils\ReferencePoint.h"

template <typename K>
class Motion_step_static : public Motion_step_base<K>
{
public:
  typedef Motion_step_base<K>       Base;
  typedef Motion_step_static<K>     Self;
  typedef Base::Reference_point     Reference_point;

private:
  Reference_point     r;
public:
  Motion_step_static () : Base(UNINITIALIZED)
  {}
  Motion_step_static (const Reference_point& reference_point)
                          :Base(STATIC), r(reference_point)
  {}  
  Motion_step_static(const Self& other)
      :Base(STATIC)
  {
      if (&other == this)
          return;
      r = other.source();
      return;
  }
public:
  Reference_point source() const 
  {
    CGAL_precondition (motion_type != UNINITIALIZED);
    return r;
  }
  
  Reference_point target() const 
  {
    CGAL_precondition (motion_type != UNINITIALIZED);
    return r;
  }
public:
  void reverse_motion_step()
  {
      return;
  }
  double approximate_motion_time(double speed) 
  {
    CGAL_precondition (motion_type != UNINITIALIZED);

    return 0; 
  }
public:
  void read(std::istream& is)
  {
    motion_type = STATIC;

    r.read(is);
    return;
  }
  void write(std::ostream& os)
  {
    CGAL_precondition (motion_type != UNINITIALIZED);

    r.write(os);
    return;
  }
private:
  bool is_legal_motion()
  {
    return (true);
  }
}; //Motion_step_static
#endif //MOTION_STEP_STATIC_H