#ifndef FIXED_POINT_CONSTRAINT_H
#define FIXED_POINT_CONSTRAINT_H

#include "Manifolds\Base\Constraint_base.h"
#include "Utils\Rotation_utils\RotationRange.h"

namespace mms{

template <typename Kernel>
class Fixed_point_constraint: 
  public Constraint_base<typename Kernel::Point_2,
                         typename Rotation_range_absolute<typename Kernel::FT> >
{
public:
  typedef  Constraint_base<typename Kernel::Point_2,
                         typename Rotation_range_absolute<typename Kernel::FT> > Base;  
public:
  //Ctrs
  Fixed_point_constraint()
    : Base(FIXED_POINT){}
  Fixed_point_constraint(const Restriction& restriction)
    : Base(restriction, FIXED_POINT)  {}
  Fixed_point_constraint(const Restriction& restriction, const RoI& roi)
    : Base(restriction, roi, FIXED_POINT)  {}
};  //class Fixed_point_constraint
} //namespace mms{

#endif //FIXED_POINT_CONSTRAINT_H