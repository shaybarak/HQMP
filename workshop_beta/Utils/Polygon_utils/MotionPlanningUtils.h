#ifndef MP_POLYGON_PREDICATES_H
#define MP_POLYGON_PREDICATES_H

#include <CGAL/Polygon_2.h>
#include "Utils\Geometry_utils\BoundingUtils.h"

////////////////////////
//predicates          //
////////////////////////
template<typename K>
typename K::Point_2 get_center(const CGAL::Polygon_2<K>& polygon)
{
  return get_bounding_circle_center<K> (polygon.vertices_begin(), polygon.vertices_end());
}

template<typename K>
typename K::Point_2 get_reference_point(const CGAL::Polygon_2<K>& polygon)
{
 //convention: reference point is always the midpoint point between first and second points
  //to this point we add a small step in the up /	down direction
  typedef typename K::Point_2                                 Point;
  typedef typename K::FT                                      NT;
  
  //get first two points
  typename CGAL::Polygon_2<K>::Vertex_const_iterator vi (polygon.vertices_begin());
  Point p1(*vi);
  vi++;
  Point p2(*vi);

  //get midpoint and both options
  double step = SMALL_STEP;
  Point midpoint((p1.x()+p2.x())/2, (p1.y()+p2.y())/2);
  Point above1(midpoint.x(), midpoint.y()+step);
  Point below1(midpoint.x(), midpoint.y()-step);
  Point above2(midpoint.x()+step, midpoint.y());
  Point below2(midpoint.x()-step, midpoint.y());

  bool above_in_polygon1 = polygon.has_on_bounded_side(above1);
  bool below_in_polygon1 = polygon.has_on_bounded_side(below1);
  bool above_in_polygon2 = polygon.has_on_bounded_side(above2);
  bool below_in_polygon2 = polygon.has_on_bounded_side(below2);

  while ( (above_in_polygon1 == below_in_polygon1) &&
          (above_in_polygon2 == below_in_polygon2) )
  {
    step = step /2;

    above1= Point(midpoint.x(), midpoint.y()+step);
    below1= Point(midpoint.x(), midpoint.y()-step);
    above2= Point(midpoint.x()+step, midpoint.y());
    below2= Point(midpoint.x()-step, midpoint.y());


    above_in_polygon1 = polygon.has_on_positive_side(above1);
    below_in_polygon1 = polygon.has_on_positive_side(below1);
    above_in_polygon2 = polygon.has_on_positive_side(above2);
    below_in_polygon2 = polygon.has_on_positive_side(below1);
  }
  CGAL_postcondition( (above_in_polygon1 != below_in_polygon1) ||
                      (above_in_polygon2 != below_in_polygon2) );

  //return relevant option
  if (above_in_polygon1 != below_in_polygon1)
    return  (above_in_polygon1 ) ? above1 : below1 ;
  else //(above_in_polygon2 != below_in_polygon2)
    return  (above_in_polygon2 ) ? above2 : below2 ;
}
////////////////////////
//Polygon movement    //
////////////////////////
template<typename K>
void negate_polygon(const CGAL::Polygon_2<K>& polygon, CGAL::Polygon_2<K>&  new_polygon)
{
  if (!new_polygon.is_empty ())
    new_polygon.clear ();
	
  typename CGAL::Polygon_2<K>::Vertex_const_iterator vit;
  for (vit = polygon.vertices_begin(); vit != polygon.vertices_end(); ++vit)
  {
    typename K::Point_2 p (- (vit->x()),- (vit->y()));
    new_polygon.push_back(p);
  }

  return;
}

template<typename K>
void center_polygon_at_zero(const CGAL::Polygon_2<K>& polygon, CGAL::Polygon_2<K>& centered_polygon)
{
  K::Point_2 center       (get_center<K> (polygon));
  K::Point_2 translation  (-center.x(), -center.y());
  translate_polygon (polygon, centered_polygon, translation);
  return ;
}
template<typename K>
void center_polygon_ref_point_at_zero(const CGAL::Polygon_2<K>& polygon, CGAL::Polygon_2<K>& centered_polygon)
{
  K::Point_2 reference_point (get_reference_point<K> (polygon));
  K::Point_2 minus_ref_point (-reference_point.x(),-reference_point.y());
  translate_polygon<K> (polygon, centered_polygon, minus_ref_point);
}

#endif  //MP_POLYGON_PREDICATES_H
