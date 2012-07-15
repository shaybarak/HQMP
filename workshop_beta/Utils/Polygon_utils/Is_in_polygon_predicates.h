#ifndef IS_IN_POLYGON_PREDICATES_H
#define IS_IN_POLYGON_PREDICATES_H

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

////////////////////////////
//point polygon relations //
////////////////////////////
template<typename K>
bool is_in_polygon(const typename K::Point_2& p, const CGAL::Polygon_2<K>& pgn, bool including_boundary)
{
  CGAL::Bounded_side b (pgn.bounded_side (p));
  if (b == CGAL::ON_UNBOUNDED_SIDE)
    return false;
  if ( (!including_boundary) && 
       (b == CGAL::ON_BOUNDARY) )
    return false;
  return true;
}

template<typename K>
bool is_in_polygon(const typename K::Point_2& p, const CGAL::Polygon_with_holes_2<K>& pgn ,bool including_boundary)
{	
  if (pgn.is_unbounded() == false)
  {
    //check point orientation with outer boundary
    CGAL::Bounded_side b (pgn.outer_boundary().bounded_side (p));
    if (b == CGAL::ON_UNBOUNDED_SIDE)
      return false;
    if (b == CGAL::ON_BOUNDARY)
      return including_boundary;
  }

  //point is inside outer_boundary polygon, chek that it is not in any hole
  for ( CGAL::Polygon_with_holes_2<K>::Hole_const_iterator iter = pgn.holes_begin() ; 
        iter != pgn.holes_end(); 
        ++iter)
  {
    if (is_in_polygon (p,*iter,!including_boundary))
      return false;
  }
  return true;

}

//////////////////////////////
//intersection predicates   //
//////////////////////////////
template<typename K>
bool do_intersect_inside(const typename K::Segment_2& seg, const CGAL::Polygon_2<K>& pgn)
{
  typedef typename K::FT FT;
  typedef typename K::Point_2 Point_2;
  if (  (is_in_polygon<K>(seg.source(), pgn, false)) || 
        (is_in_polygon<K>(seg.target(), pgn, false)) )
        return true;
  
  //ToDo - change implementation

  //simply find all intersection points between segment and edges.
  //order them lexicographically
  //choose a point between two intesection points, 
  //if one such point is in then return true, else return false

  std::vector<Point_2> intersection_points;
  for (CGAL::Polygon_2<K>::Edge_const_iterator it (pgn.edges_begin  ()); it != pgn.edges_end (); ++it)
  {
    CGAL::Object  obj (CGAL::intersection(seg,*it));
    Point_2    intersection;
    if (CGAL::assign(intersection, obj))
      intersection_points.push_back(intersection);
  }

  if (intersection_points.empty())
    return false;

  //stupid implementation;
  std::vector<FT> intersections;
  typename  K::Point_2 p;    
  if (seg.source().x() != seg.target().x())
  {
    BOOST_FOREACH (p, intersection_points)
      intersections.push_back( (p.x()            - seg.source().x()) / 
                               (seg.target().x() - seg.source().x()) );
  }
  else
  {
    BOOST_FOREACH (p, intersection_points)
      intersections.push_back( (p.y()            - seg.source().y()) / 
                               (seg.target().y() - seg.source().y()) );
  }

  std::sort(intersections.begin(), intersections.end());
  std::vector<Point_2> candidates;
  for (unsigned int i(0); i<intersections.size()-1; ++i)
  {
    FT r = (intersections[i] + intersections[i+1])/2;
    FT x = seg.source().x() + r*(seg.target().x() - seg.source().x());
    FT y = seg.source().y() + r*(seg.target().y() - seg.source().y());
    candidates.push_back(Point_2 (x, y));
  }

  BOOST_FOREACH (p, candidates)
    if (is_in_polygon<K>(p, pgn, false))
      return true;

  return false;
}
template<typename Segment_2>
bool do_intersect_inside(const Segment_2& seg1, const Segment_2& seg2)
{
  if (seg1.has_on (seg2.source()) || seg1.has_on (seg2.target()) ||
      seg2.has_on (seg1.source()) || seg2.has_on (seg1.target()) )
      return false;

  return CGAL::do_intersect(seg1, seg2);
}



//////////////////////////////
//segment polygon relations //
//////////////////////////////
template<typename K>
bool is_in_polygon(const typename K::Segment_2& seg, const CGAL::Polygon_2<K>& pgn, bool including_boundary)
{
  //check for intersections with all edges
  K::Point_2 s(seg.source());
  K::Point_2 t(seg.target());
  
  if (is_in_polygon<K>(s,pgn,including_boundary) == false)
    return false;
  if (is_in_polygon<K>(t,pgn,including_boundary) == false)
    return false;

  CGAL::Polygon_2<K>::Edge_const_iterator e_iter;
  for (e_iter = pgn.edges_begin(); e_iter != pgn.edges_end(); ++e_iter)
  {
    //segment is an edge
    if (e_iter->has_on(s) && e_iter->has_on(t) && including_boundary)
      return true;
    
    //handle touch with edge on an endpoint
    if ( (including_boundary == false) &&
         (e_iter->has_on(s) || e_iter->has_on(t) || seg.has_on(e_iter->source()) || seg.has_on(e_iter->target())) )
    {
        return false;
    }
    //check intersection with segment excluding endpoints
    if (do_intersect_inside(seg, *e_iter))
      return false;
  }
  CGAL::Polygon_2<K>::Vertex_const_iterator v_iter;
  std::vector<K::FT> intersections;
  intersections.push_back(0);
  intersections.push_back(1);
  for (v_iter = pgn.vertices_begin(); v_iter != pgn.vertices_end(); ++v_iter)
  {
    if (seg.has_on(*v_iter))
    {
      CGAL_precondition (including_boundary); //or else should have been filtered
      K::FT r;
      if (s.x() != t.x())
        r = (v_iter->x() - s.x()) / (t.x() - s.x());
      else //(s.x() == t.x())
        r = (v_iter->y() - s.y()) / (t.y() - s.y());      
      intersections.push_back(r);
    }
  }
  std::sort(intersections.begin(), intersections.end());
  for (unsigned int i(0); i <intersections.size() - 1; ++i)
  {
    //check that an inner point of the segment is in the polygon
    K::FT r = (intersections[i+1] + intersections[i]) / 2;
    K::Point_2 tmp(  (s.x() + r * (t.x() - s.x())),
                     (s.y() + r * (t.y() - s.y())) );
    if (is_in_polygon<K>(tmp, pgn, including_boundary) == false)
      return false;
  }
  return true;
}
template<typename K>
bool is_in_polygon(const typename K::Segment_2& seg, const CGAL::Polygon_with_holes_2<K>& pgn, bool including_boundary)
{
  CGAL_precondition (pgn.is_unbounded() == false);

  K::Point_2 s(seg.source());
  K::Point_2 t(seg.target());

  //check if inside bounding
  CGAL::Polygon_2<K> outer(pgn.outer_boundary ());
  if (is_in_polygon<K>(seg, outer, including_boundary) == false)
    return false;

  //check if inside hole
  for (CGAL::Polygon_with_holes_2<K>::Hole_const_iterator iter = pgn.holes_begin() ; iter != pgn.holes_end(); ++iter)
  {
    if ( (is_in_polygon<K> (s, *iter, false) == true) ||
         (is_in_polygon<K> (t, *iter, false) == true) )
        return false;
    if (is_in_polygon<K>(seg, *iter, !including_boundary))
        return false;
    K::Point_2 midpoint((s.x()+t.x())/2,(s.y()+t.y())/2);
    if (is_in_polygon<K> (midpoint,*iter,false) == true) 
        return false;
  }

  //check if intersects hole
  for (CGAL::Polygon_with_holes_2<K>::Hole_const_iterator iter = pgn.holes_begin() ; iter != pgn.holes_end(); ++iter)
  {
    for (CGAL::Polygon_2<K>::Edge_const_iterator edge_iter = iter->edges_begin (); edge_iter!= iter->edges_end (); ++edge_iter)
    {
      if (including_boundary)
      {
        if (do_intersect_inside(seg, *edge_iter))
          return false;
      }
      else //(!including_boundary)
      {
        if (CGAL::do_intersect(seg, *edge_iter))
          return false;
      }
    }
  }

  //check if intersects hole
  for (CGAL::Polygon_with_holes_2<K>::Hole_const_iterator iter = pgn.holes_begin() ; iter != pgn.holes_end(); ++iter)
  {
    if (do_intersect_inside(seg, *iter))
      return false;
  }
  return true;      
}


#endif  //IS_IN_POLYGON_PREDICATES_H