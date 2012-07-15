#ifndef DO_INTERSECT_PREDICATES_H
#define DO_INTERSECT_PREDICATES_H

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

#include "Utils\Polygon_utils\Is_in_polygon_predicates.h"
//////////////////////////////
//intersection predicates   //
//////////////////////////////
template<typename K>
bool do_intersect(const typename K::Segment_2& seg, const CGAL::Polygon_2<K>& pgn)
{
  if (  (is_in_polygon<K>(seg.source(),pgn,false)) || 
        (is_in_polygon<K>(seg.target(),pgn,false)) )
        return true;
  
  //source and target are out of the polygon
  //intersect segment to find inner points

  for (CGAL::Polygon_2<K>::Edge_const_iterator it (pgn.edges_begin  ()); it != pgn.edges_end (); ++it)
    if (CGAL::do_intersect(seg,*it))
      return true;

  return false;
}
template<typename K>
bool do_intersect(const typename K::Segment_2& seg, const CGAL::Polygon_with_holes_2<K>& pgn)
{
  //if source or target are in the polygon, 
  //then the segmen intersects the polygon
  if (  (is_in_polygon<K>(seg.source(),pgn,false)) || 
        (is_in_polygon<K>(seg.target(),pgn,false)) )
        return true;
  
  //if source or target are in a hole, and the other is outside
  //then the segmen intersects the polygon
  bool is_source_insde_boundary = is_in_polygon<K>(seg.source(), pgn.outer_boundary(), false);
  bool is_target_insde_boundary = is_in_polygon<K>(seg.target(), pgn.outer_boundary(), false);

  if (is_source_insde_boundary != is_target_insde_boundary)
    return true;
  
  if (is_source_insde_boundary == false)
  {
    CGAL_precondition (is_target_insde_boundary == false);
    //if both are out, the only way to intersect is with an outer boundary
    for ( CGAL::Polygon_2<K>::Edge_const_iterator it (pgn.outer_boundary().edges_begin()); 
          it != pgn.outer_boundary().edges_end(); 
          ++it)
    {
      if (CGAL::do_intersect(seg,*it))
        return true;
    }
    return false;
  }
  else //is_source_insde_boundary
  {
    CGAL_precondition (is_target_insde_boundary);
    //if both are out, the only way to intersect is with a hole
    for ( CGAL::Polygon_with_holes_2<K>::Hole_const_iterator iter = pgn.holes_begin() ; 
        iter != pgn.holes_end(); 
        ++iter)
    {
      for ( CGAL::Polygon_2<K>::Edge_const_iterator it (iter->edges_begin()); 
            it != iter->edges_end (); 
            ++it)
      {
        if (CGAL::do_intersect(seg,*it))
          return true;
      }
      return false;
    }
  }
  return false;
}

#endif  //DO_INTERSECT_PREDICATES_H