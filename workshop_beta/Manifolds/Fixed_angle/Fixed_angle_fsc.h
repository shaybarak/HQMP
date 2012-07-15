#ifndef FIXED_ANGLE_FSC
#define FIXED_ANGLE_FSC

#include "Manifolds\Base\Fsc_base.h"
#include "Utils\Polygon_utils\Smart_polygon_with_holes.h"

namespace mms{

template <typename Kernel>
class Fixed_angle_fsc : public Fsc_base<Smart_polygon_with_holes<Kernel> >
{
public:
  typedef Fsc_base<Smart_polygon_with_holes<Kernel> >         Base;
public:
  Fixed_angle_fsc( const Cell& cell)
    : Base (cell)
  {}
  Fixed_angle_fsc(const Fixed_angle_fsc& other) 
    :Base(other)
  {}    
};  //Fixed_angle_fsc

} // namespace mms{

#endif //FIXED_ANGLE_FSC