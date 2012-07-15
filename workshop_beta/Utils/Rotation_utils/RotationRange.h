#ifndef ROTATION_RANGE_ABSOLUTE_H
#define ROTATION_RANGE_ABSOLUTE_H

#include "Utils\Rotation_utils\Rotation.h"

template <typename NT>
class Rotation_range_absolute //CCW rotation
{
private:
  typedef Rotation<NT>  Rotation;
public:
  //////////////
  //constructors
  Rotation_range_absolute ()
    : _r1 (0,1),
      _r2 (0,1),
      _full_range(true)
  {}
  Rotation_range_absolute (const Rotation& r1,const Rotation& r2)
    : _r1 (r1),
      _r2 (r2),
      _full_range(false)
  {}
  Rotation_range_absolute(const Rotation_range_absolute& other)
    : _full_range(other.is_full_range())
  {
    _r1 = (_full_range ? Rotation(0,1) : other.get_start_rotation());
    _r2 = (_full_range ? Rotation(0,1) : other.get_end_rotation());
  }

  //////////////
  //predicates
  bool is_full_range() const 
  {
    return _full_range;
  }
  const Rotation& get_start_rotation() const
  {
    CGAL_precondition(_full_range == false);
    return _r1;
  }
  const Rotation& get_end_rotation() const
  {
    CGAL_precondition(_full_range == false);
    return _r2;
  }
  bool is_in_range(const Rotation& r) const
  {
    if (_full_range)
      return true;
    if (_r2.is_larger_than(_r1))
    {
      if (r.is_larger_than(_r1) &&
          _r2.is_larger_than(r))
          return true;
      else
        return false;
    }
    else //(_r1.is_larger_than(_r2))
    {
      if (r.is_larger_than(_r1) ||
          _r2.is_larger_than(r))
          return true;
      else
        return false;
    }
  }
protected:
  Rotation  _r1;			
  Rotation  _r2;			
  bool      _full_range;
};//Rotation_range_base
#endif  //ROTATION_RANGE_ABSOLUTE_H