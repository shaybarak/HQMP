#ifndef FREE_POINT_UTILS_H
#define FREE_POINT_UTILS_H

#include "Utils\Rotation_utils\Rotation.h"
#include "Utils\Rotation_utils\RotationUtils.h"
#include "Utils\ReferencePoint.h"
#include "Utils\Polygon_utils\ExtendedPolygon.h"

namespace FixedPoint{

template <typename K>
Rotation<typename K::FT> get_rotation_from_parametrization( const typename K::FT& t)
{
  K::FT t_sq (t*t);
  K::FT cos ( (1 - t_sq) / (1 + t_sq) );
  K::FT sin ( (2 * t) / (1 + t_sq) );

  return Rotation<K::FT>(sin,cos);
}


template <typename K>
Reference_point<K> get_reference_point( const typename K::FT& t, 
                                        const typename K::Point_2& p)
{
  Rotation<K::FT>theta(get_rotation_from_parametrization(t));
  Reference_point<K> ref_point( p , theta ); 
  
  return ref_point ;
}

template <typename K>
typename K::FT get_parametrization_theta( const typename Rotation<typename K::FT>& theta)
{
  K::FT sin (theta.sin());
  K::FT cos (theta.cos());
  
  K::FT t (!CGAL::is_zero(1+cos)?
            (sin / (1 + cos)) :		//tan (theta / 2) = sin (theta) / (1 + cos theta) 
            (CGAL::sign (sin) ? INFINITY : -INFINITY));
  return t;
}

} //namespace FixedPoint

#endif //FREE_POINT_UTILS_H