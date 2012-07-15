#ifndef POLYGON_UTILS_H
#define POLYGON_UTILS_H

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Bbox_2.h>

template<typename K>
typename K::FT get_squared_radius(const CGAL::Polygon_2<K>& p, const typename K::Point_2& center)
{
  typedef typename CGAL::Polygon_2<K>::Vertex_const_iterator  V_iterator;

  K::FT r_sq (0);
  for (V_iterator vit (p.vertices_begin()); vit != p.vertices_end(); ++vit)
  {
    K::FT dist_sq (CGAL::squared_distance (*vit,center));
    if (CGAL::compare (dist_sq , r_sq ) == CGAL::LARGER)
      r_sq = dist_sq;
  }
  return r_sq;
}


template <typename K>
typename K::FT get_area(const CGAL::Polygon_with_holes_2<K>& pgn)
{
  K::FT bounding_area = pgn.outer_boundary().area();
  K::FT inner_area = 0;

  CGAL::Polygon_with_holes_2<K>::Hole_const_iterator hi;
  for (hi = pgn.holes_begin (); hi != pgn.holes_end(); ++hi)
    inner_area += hi->area();

  return (bounding_area - inner_area);
}

template <typename K>
int get_num_of_vertices(const CGAL::Polygon_with_holes_2<K>& pgn)
{
  int num_of_vertices = pgn.outer_boundary().size();
  CGAL::Polygon_with_holes_2<K>::Hole_const_iterator hi;
  for (hi = pgn.holes_begin (); hi != pgn.holes_end(); ++hi)
    num_of_vertices += hi->size();

  return num_of_vertices;
}
template <typename K>
CGAL::Bbox_2 get_bbox(const CGAL::Polygon_with_holes_2<K>& pgn)
{
  return pgn.outer_boundary().bbox();
}

#endif //POLYGON_UTILS_H