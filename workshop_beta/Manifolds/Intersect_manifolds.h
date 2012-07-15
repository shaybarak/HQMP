#ifndef INTERSECT_MANIFOLDS_H
#define INTERSECT_MANIFOLDS_H

#include "Manifolds\Fixed_angle\Fixed_angle_manifold.h"
#include "Manifolds\Fixed_point\Fixed_point_manifold.h"

#include "Utils\ReferencePoint.h"
#include "Utils\Rotation_utils\Rotation.h"
#include "Utils\Rotation_utils\RotationRange.h"

namespace mms{


template <typename Kernel, typename Fixed_point_manifold>
void intersect (typename Fixed_angle_manifold<Kernel>& layer, 
                Fixed_point_manifold& connector,
                Int_pair& intersected_ids)
{
  BOOST_STATIC_ASSERT((boost::is_same<typename Fixed_point_manifold::K, Kernel>::value));

  //initialize return value to NO_ID;
  intersected_ids.first = NO_ID;
  intersected_ids.second = NO_ID;

  // a layer intersects a segment if 
  //the angle defining the layer is within the range of the segment's angles
  const typename Rotation<typename Kernel::FT>&                angle = layer.constraint().restriction();
  const typename Rotation_range_absolute<typename Kernel::FT>& range = connector.constraint().region_of_interest();
  if (range.is_in_range(angle) == false)
    return;

  //intersect two manifolds
  typename Kernel::Point_2  p = connector.constraint().restriction();
  int layer_cell_id = layer.get_containing_cell(p);
  if (layer_cell_id == NO_ID)
    return;
  
  typename Reference_point<Kernel> ref_point(p, angle);
  int connector_cell_id = connector.free_space_location_hint(ref_point);

  intersected_ids.first = layer_cell_id;
  intersected_ids.second= connector_cell_id;
  
  return;
}

template <typename Kernel, typename Fixed_point_manifold>
void intersect (Fixed_point_manifold& connector,
                typename Fixed_angle_manifold<Kernel>& layer,
                Int_pair& intersected_ids)
{
  std::vector<Int_pair> intersection;
  intersect(layer, connector, intersection);
  intersected_ids.first  = intersection.second;
  intersected_ids.second = intersection.first;  
  return;
} 
}//namespace mms{

#endif //INTERSECT_MANIFOLDS_H