#ifndef SMART_POLYGON_H
#define SMART_POLYGON_H

#include <CGAL/Bbox_2.h>
#include "Utils\Polygon_utils\PolygonUtils.h"
#include "Utils\Polygon_utils\do_intersect_predicates.h"
#include "Utils\Geometry_utils\GetClosestUtils.h"

template <typename Kernel>
class Smart_polygon_with_holes
{
public:
  typedef typename Kernel::FT                    NT;
  typedef CGAL::Bbox_2                           Bbox;
  typedef typename Kernel::Point_2               Point;
  typedef typename Kernel::Segment_2             Segment;
  typedef typename Kernel::Iso_rectangle_2       Iso_rectangle;
  typedef CGAL::Polygon_2<Kernel>                Polygon;
  typedef CGAL::Polygon_with_holes_2<Kernel>     Polygon_with_holes;
private:
  Polygon_with_holes  _pgn;
  
  NT                  _area;
  bool                _computed_area;
  
  int                 _num_of_vertices;
  bool                _computed_num_of_vertices;
  
  Bbox                _bbox;
  Iso_rectangle       _iso_rectangle;
  bool                _computed_bbox;
public:
  Smart_polygon_with_holes()
    : _computed_area(false), _computed_num_of_vertices(false), _computed_bbox(false)  
  {}
  Smart_polygon_with_holes(const Polygon& polygon)
    : _computed_area(false), _computed_num_of_vertices(false), _computed_bbox(false),
      _pgn (polygon) 
  {}
  Smart_polygon_with_holes(const Polygon_with_holes& polygon)
    : _computed_area(false), _computed_num_of_vertices(false), _computed_bbox(false),
      _pgn (polygon) 
  {}
  Smart_polygon_with_holes(const Smart_polygon_with_holes& other)
    : _computed_area(false), 
      _computed_num_of_vertices(false), 
      _computed_bbox(false),
      _pgn (other.polygon()) 
  {}
  
  const Polygon_with_holes& polygon() const
  {
    return _pgn;
  }
  Polygon_with_holes& polygon()
  {
    return _pgn;
  }
  NT area()
  {
    CGAL_precondition (_pgn.is_unbounded()==false);
    if (!_computed_area)
    {
      _area = get_area(_pgn);
      _computed_area = true;
    }
    return _area;
  }
  int num_of_vertices()
  {
    if (!_computed_num_of_vertices)
    {
      _num_of_vertices = get_num_of_vertices(_pgn);
      _computed_num_of_vertices = true;
    }
    return _num_of_vertices;
  }
  Bbox& bbox()
  {
    CGAL_precondition (_pgn.is_unbounded()==false);
    if (!_computed_bbox)
    {
      _bbox = get_bbox(_pgn);
      _iso_rectangle = Iso_rectangle(Point(_bbox.xmin(),_bbox.ymin()),
                                     Point(_bbox.xmax(),_bbox.ymax()) );
      _computed_bbox = true;
    }
    return _bbox;
  }
  Iso_rectangle iso_rectangle()
  {
    CGAL_precondition (_pgn.is_unbounded()==false);
    if (!_computed_bbox)
    {
      _bbox = get_bbox(_pgn);
      _iso_rectangle = Iso_rectangle(Point(bbox().xmin(),bbox().ymin()),
                                     Point(bbox().xmax(),bbox().ymax()) );
      _computed_bbox = true;
    }
    return _iso_rectangle;
  }
  //indicators
  bool is_area_computed() const {return _computed_area;}
  bool is_num_of_vertices_computed() const {return _computed_num_of_vertices;}
  bool is_bbox_computed() const {return _computed_bbox;};
  //predicates
  bool is_in_polygon(const Point& p, bool including_boundary)
  {
    if (_pgn.is_unbounded())
       return ::is_in_polygon<Kernel>(p, _pgn,including_boundary);

    Bbox& box = bbox();
    double x = CGAL::to_double(p.x());
    if ( (x < box.xmin()) || (x > box.xmax()) )
      return false;
    double y = CGAL::to_double(p.y());
    if ( (y < box.ymin()) || (y > box.ymax()) )
      return false;

    return ::is_in_polygon<Kernel>(p, _pgn,including_boundary);
  }

  bool does_intersect(const Segment& seg)
  {
    if (CGAL::do_intersect(iso_rectangle(),seg) == false)
      return false;
    return ::do_intersect<Kernel>(seg, _pgn);
  }

  bool point_in_intersection(const Segment& seg, Point& p)
  {
    if (CGAL::do_intersect(iso_rectangle(),seg) == false)
      return false;
    return get_point_in_poly_on_seg(seg, _pgn, p);
  }


}; //Smart_polygon

#endif  //SMART_POLYGON_H