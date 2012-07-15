#ifndef GET_CLOSEST_UTILS_H
#define GET_CLOSEST_UTILS_H

#include <CGAL\Polygon_2.h>
#include <CGAL\Polygon_with_holes_2.h>

template <typename K>
typename K::Point_2 point_in_segment(const typename K::Segment_2 seg)
{
  CGAL_precondition(seg.is_degenerate () == false);

  K::FT mid_x ( (seg.source().x() + seg.target().x()) / 2 );
  K::FT mid_y ( (seg.source().y() + seg.target().y()) / 2 );
  K::Point_2 p(mid_x,mid_y);

  CGAL_postcondition (seg.has_on(p));
  return p;    
}

template <typename K>
typename K::Point_2 get_closest_point(const typename K::Line_2&  l,
                                      const typename K::Point_2& p)
{
  //is point on the line
  if (l.has_on(p))
    return p;

  //compute intersection of line with perpendicular line at point
  K::Line_2     l_perp (l.perpendicular (p));
  CGAL::Object  obj (CGAL::intersection(l,l_perp));
  K::Point_2    intersection;
  if (!CGAL::assign(intersection, obj))
    CGAL_postcondition (false);

  return intersection;
}
template <typename K>
typename K::Point_2 get_closest_point(const typename K::Segment_2& seg,
                                      const typename K::Point_2& p, 
                                      typename K::FT& d_sq)
{
  K::Point_2  l_closest (get_closest_point<K> (K::Line_2(seg),p));
  if (seg.has_on(l_closest))
  {
    d_sq = CGAL::squared_distance(l_closest,p);
    return l_closest;
  }

  //cloesest point is either source or target
  K::FT s_sq_d (CGAL::squared_distance(seg.source(),p));
  K::FT t_sq_d (CGAL::squared_distance(seg.target(),p));

  if (CGAL::compare (s_sq_d,t_sq_d) == CGAL::SMALLER)
  {
    //source is closer to p than target
    d_sq = s_sq_d;
    return seg.source();
  }
  else //LARGER or EQUAL
  {
    //target is closer to p than source
    d_sq = t_sq_d;
    return seg.target();
  }
}

template <typename K>
typename K::Point_2 get_closest_vertex(const typename K::Point_2& source,
                                       const CGAL::Polygon_2<K>& polygon)
{
  CGAL::Polygon_2<K>::Vertex_iterator vi (polygon.vertices_begin ());
  K::FT      distance_sq (CGAL::squared_distance(source,*vi));
  K::Point_2 closest(*vi);
  
  vi++;
  for ( ; vi!=polygon.vertices_end() ; ++vi)
  {
    K::FT tmp_distance_sq (CGAL::squared_distance(source,*vi));
    if (tmp_distance_sq < distance_sq)
    {
      distance_sq = tmp_distance_sq;
      closest = *vi;
    }
  }
  return closest;
}

template <typename K>
typename K::Point_2 get_closest_point(const typename K::Point_2& p,
                                      const CGAL::Polygon_2<K>& polygon,
                                      typename K::FT& d_sq)
{
  //handle first edge
  CGAL::Polygon_2<K>::Edge_const_iterator it (polygon.edges_begin());
  K::Point_2     closest (get_closest_point<K> (*it,p,d_sq));
      
  ++it;
  for (; it != polygon.edges_end(); ++it)
  {
    K::FT       tmp_d_sq;
    K::Point_2  tmp_closest (get_closest_point<K> (*it,p,tmp_d_sq));
    
    if (tmp_d_sq < d_sq )
    {
      d_sq  = tmp_d_sq;
      closest = tmp_closest;
    }
  }
  return closest;
}
template <typename K>
typename K::Point_2 get_closest_point(const typename K::Point_2& p,
                                      const CGAL::Polygon_2<K>& polygon)
{
  K::FT d_sq;
  return  get_closest_point<K>(p,polygon,d_sq);
}
template <typename K>
typename K::Point_2 get_closest_point(const typename K::Point_2& p,
                                      const CGAL::Polygon_with_holes_2<K>& polygon,
                                      typename K::FT& d_sq)
{
  CGAL_precondition(polygon.is_unbounded () == false);

  K::Point_2  closest,tmp_closest;
  K::FT       tmp_d_sq;

  closest = get_closest_point(p,polygon.outer_boundary (),d_sq);

  CGAL::Polygon_with_holes_2<K>::Hole_const_iterator hi;
  for (hi = polygon.holes_begin (); hi != polygon.holes_end(); ++hi)
  {
    tmp_closest = get_closest_point(p,*hi,tmp_d_sq);
    if (tmp_d_sq < d_sq )
    {
      d_sq  = tmp_d_sq;
      closest = tmp_closest;
    }
  }

  return closest;
}

//get intersections of segment or line (intersecting_obj) and polygon boundary
template <typename K, typename Intersecting_obj, typename OutputIterator>
void get_intersections(const CGAL::Polygon_2<K>& pgn,
                       const Intersecting_obj& intersecting_obj,
                       OutputIterator& oi)
{
  for (CGAL::Polygon_2<K>::Edge_const_iterator it (pgn.edges_begin()); it != pgn.edges_end(); ++it)
  {
    CGAL::Object  obj (CGAL::intersection(intersecting_obj,*it));
    
    K::Point_2    intersection_p;
    if (CGAL::assign(intersection_p, obj))
      *oi++ = intersection_p;

    K::Segment_2  intersection_s;
    if (CGAL::assign(intersection_s, obj))
    {
      *oi++ = intersection_s.source();
      *oi++ = intersection_s.target();
    }
  }
  return;
}
//get closest point on polygon to p that is on seg
template <typename K>
typename K::Point_2 get_closest_point(const typename K::Point_2& p,
                                      const CGAL::Polygon_with_holes_2<K>& pgn,
                                      const typename K::Segment_2& seg)
{
  CGAL_precondition(pgn.is_unbounded () == false);
  CGAL_precondition(seg.has_on(p));
  //if point is in polygon it is the closest
  if (is_in_polygon<K> (p,pgn,true))
    return p;
  
  std::vector<K::Point_2> candidates;
  //check to see if the segment intersects the polygon
  get_intersections<K> (pgn.outer_boundary(),seg,std::back_inserter(candidates));
  CGAL::Polygon_with_holes_2<K>::Hole_const_iterator hi;
  for (hi = pgn.holes_begin (); hi != pgn.holes_end(); ++hi)
    get_intersections<K> (*hi,seg,std::back_inserter(candidates));

  CGAL_postcondition (candidates.empty() == false);


  std::vector<K::Point_2>::iterator iter (candidates.begin());
  K::FT       d_sq (CGAL::squared_distance(p,*iter));;
  K::Point_2  closest (*iter);
  while (++iter != candidates.end())
  {
    K::FT       tmp_d_sq (CGAL::squared_distance(p,*iter));;
    if (CGAL::compare (tmp_d_sq ,d_sq) == CGAL::SMALLER)
    {
      d_sq = tmp_d_sq ;
      closest = *iter;
    }
  }
  return closest;
}

template <typename K>
bool get_point_in_poly_on_seg(const typename K::Segment_2&                  seg, 
                              const typename CGAL::Polygon_with_holes_2<K>& pgn,
                              typename K::Point_2&                          p)
{
  typedef typename K::Point_2   Point;
  typedef typename K::Segment_2 Segment;

  CGAL_precondition (pgn.is_unbounded () == false);
  CGAL::Polygon_2<K> polygon (pgn.outer_boundary());

  //first try to check if source or target are in polygon
  if (is_in_polygon<K>(seg.source(),pgn,true))
  {
    p = seg.source();
    return true;
  }
  if (is_in_polygon<K>(seg.target(),pgn,true))
  {
    p = seg.target();
    return true;
  }
   
  //source and target are out of the polygon, 
  //intersect segment to find inner points
  std::vector <Point>  intersection_points;
  for (CGAL::Polygon_2<K>::Edge_const_iterator it (polygon.edges_begin()); it != polygon.edges_end(); ++it)
  {
    //intersection point of tangent lines
    CGAL::Object  obj (CGAL::intersection(*it,seg));
    Point         intersection_p;

    if (CGAL::assign(intersection_p, obj)==false)
      continue; //segment is on an edge

    CGAL_postcondition (seg.has_on(intersection_p));
    intersection_points.push_back(intersection_p);
    if (intersection_points.size() == 3)
      break;  //we have for sure a point in the polygon
  }

  int size(intersection_points.size());
  switch (size)
  {
  case 0: //no intersection point
        {
          return false;
        }
  case 1: //either source is in or target is in, should of been handled
        {          
          CGAL_postcondition(false);
          assert (false);
          return false;
        }
  case 2: // either both source and target are in polygon or are out 
        { // because first case should have been filtered return point between intersections
          p = point_in_segment<K>(Segment (intersection_points[0],intersection_points[1]));
          return true;
        }
  case 3: //either between first and second or between second and thirs;
        {
          Point tmp_p1 = point_in_segment<K>(Segment (intersection_points[0],intersection_points[1]));
          bool  b1 (is_in_polygon<K>(tmp_p1,pgn,true));
          if (b1)
          {
            p =  tmp_p1;
            return true;
          }

          Point tmp_p2 = point_in_segment<K>(Segment (intersection_points[1],intersection_points[2]));
          p = tmp_p2;
          return true;
        }
  }
  assert (false);
  return false;
}
  
#endif //GET_CLOSEST_UTILS_T_H