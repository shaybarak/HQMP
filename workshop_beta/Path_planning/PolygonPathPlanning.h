#ifndef POLYGON_PATH_PLANNING_H 
#define POLYGON_PATH_PLANNING_H

#include "Path_planning\ShortestPathInPolygon.h"

template <typename K>
void plan_path_in_polygon( const typename K::Point_2& source,
                           const typename K::Point_2& target,
                           const CGAL::Polygon_with_holes_2<K>& polygon,
                           std::list<typename K::Point_2>& point_path)
{
    Shortest_path_in_polygon<K> path_in_polygon(polygon);
    path_in_polygon.shortest_path(source,target,point_path);
    return;
}
template <typename K>
void plan_path_in_polygon( const typename K::Point_2& source,
                           const typename K::Point_2& target,
                           const CGAL::Polygon_2<K>& polygon,
                           std::list<typename K::Point_2>& point_path)
{
  Shortest_path_in_polygon<K> path_in_polygon(polygon);
  path_in_polygon.shortest_path(source,target,point_path);
  return;
}
#endif //POLYGON_PATH_PLANNING_H