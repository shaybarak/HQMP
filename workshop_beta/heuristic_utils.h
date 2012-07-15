#ifndef HEURISTIC_UTILS_H
#define HEURISTIC_UTILS_H

#include "Utils\Polygon_utils\Smart_polygon_with_holes.h"

namespace mms{

template <typename K>
double get_size_percentage(Smart_polygon_with_holes<K>& pgn)
{
  //0 is small; 1 is large
  double polygon_area(CGAL::to_double(pgn.area()));
    
  //small polygon
  double max_min_polygon_area = configuration.get_max_area_of_small_feature () / 100 * 
                                configuration.get_workspace_area();
  
  if (polygon_area <= max_min_polygon_area)
      return 0;

  //large polygon
  double min_max_polygon_area = configuration.get_min_area_of_large_feature() / 100 * 
                                configuration.get_workspace_area();
  if (polygon_area  >= min_max_polygon_area)
    return 1;

  //middle polygon
  double cell_size_ratio = ((polygon_area - max_min_polygon_area) / 
                            (min_max_polygon_area - max_min_polygon_area));
  CGAL_precondition (cell_size_ratio >=0 && cell_size_ratio <=1);
  return cell_size_ratio;

}

} //namespace mms{
#endif //HEURISTIC_UTILS_H