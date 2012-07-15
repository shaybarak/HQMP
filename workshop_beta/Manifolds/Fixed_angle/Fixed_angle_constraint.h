#ifndef FIXED_ANGLE_CONSTRAINT_H
#define FIXED_ANGLE_CONSTRAINT_H

#include "Manifolds\Base\Constraint_base.h"
#include "Utils\Rotation_utils\Rotation.h"

namespace mms{

template <typename Kernel>
class Fixed_angle_constraint : public Constraint_base<typename Rotation<typename Kernel::FT>,
                                                      CGAL::Bbox_2>
{
public:
  typedef Constraint_base<typename Rotation<typename Kernel::FT>,
                          CGAL::Bbox_2>                             Base;
  typedef typename Rotation<typename Kernel::FT>                    Restriction;
  typedef CGAL::Bbox_2                                              RoI;
public:
  //Ctrs
  Fixed_angle_constraint()
    : Base(FIXED_ANGLE){}
  Fixed_angle_constraint(const Restriction& restriction)
    : Base(restriction,FIXED_ANGLE)  {}
  Fixed_angle_constraint(const Restriction& restriction, const RoI& roi)
    : Base(restriction, roi, FIXED_ANGLE)  
  {
    assert(false); //currently not supported
  }
};  //class Fixed_angle_constraint
} //namespace mms{

#endif //FIXED_ANGLE_CONSTRAINT_H