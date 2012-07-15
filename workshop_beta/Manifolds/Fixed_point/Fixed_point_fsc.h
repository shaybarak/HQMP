#ifndef FIXED_POINT_FSC
#define FIXED_POINT_FSC

#include "Manifolds\Base\Fsc_base.h"
#include "Utils\Interval_utils\IntervalSet.h"
#include "Configuration_spaces\PointPrimitive\ConfigurationSpace.h"

namespace mms{

template <typename K, typename AK, typename AK_conversions>
class Fixed_point_fsc : public Fsc_base<typename FixedPoint::Configuration_space<K, AK, AK_conversions>::Interval_set >
{
public:
  typedef Fsc_base<typename  FixedPoint::Configuration_space<K, AK, AK_conversions>::Interval_set >      Base;
public:
  Fixed_point_fsc( const Cell& cell)
    : Base (cell)
  {}
  Fixed_point_fsc(const Fixed_point_fsc& other) 
    :Base(other)
  {}    
};  //Fixed_point_fsc

} //namespace mms{
#endif //FIXED_POINT_FSC