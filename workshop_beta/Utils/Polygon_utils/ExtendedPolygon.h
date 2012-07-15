#ifndef EXTENDED_POLYGON_H
#define EXTENDED_POLYGON_H

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

#include "Utils\Rotation_utils\Rotation.h"
#include "Utils\ReferencePoint.h"
#include "Utils\Polygon_utils\PolygonUtils.h"
#include "Utils\Polygon_utils\Polygon_translations.h"
#include "Utils\Polygon_utils\Polygon_rotations.h"
#include "Utils\Polygon_utils\MotionPlanningUtils.h"

//==============================================================================
//an extended polygon is a polygon which allows efficient translation & rotation
//the polygon is represented as a centerd polygon and a reference point:
//i.e. the Original Polygon p translated such that the center lies at the Origin
//	   the reference point is a pair (Point p,Rotation r) where:
//			p: the translated center
//			r: the rotation around translated center
//
//rotation & translation can be done relatively or absolutely with regrads to 
//the curent reference point 
//==============================================================================


template <typename K>
class Extended_polygon
{
public:
  typedef typename K::FT        NT;
  typedef typename K::Point_2   Point;
  typedef Rotation<NT>          Rotation;
  typedef CGAL::Polygon_2<K>    Polygon;  
  typedef Reference_point<K>    Reference_point;
public:
//constructors
  Extended_polygon() :
      center ( Point(0,0)), 
      r_sq (0) , 
      reference (Point(0,0) , 
      Rotation (0,1))
	{}

  Extended_polygon(Polygon p) :
      center ( Point(0,0)), 
      reference (Point(0,0) , 
      Rotation (0,1))
  {
    //center robot
    Point c (get_center<K> (p));
    Point translation (-c.x(),-c.y());
    translate_polygon (p, polygon,translation);

    //update DB
    r_sq = ::get_squared_radius<K>(p,c);
  }
  Extended_polygon(const Extended_polygon<K> & e): 
	polygon(e.polygon), 
	center(e.center),		//should alwase be zero
    r_sq(e.r_sq),
	reference(Reference_point(e.reference))
    {
      CGAL_precondition (center == Point(0,0));
    }

//destructors
  ~Extended_polygon(void){}
//assignment
  Extended_polygon& operator= (const Extended_polygon<K> &other)
  {
	if (this != &other) 
	{
	  polygon = other.polygon;
      r_sq = other.r_sq;
	  reference = other.reference;
	}
    return *this;
  }
//translation 
  void translate_absolute (const Point& p)
  {
    reference.set_location(p);
    return;
  }
  void translate_relative (const Point& p)
  {
    reference.set_location(Point(reference.first.x() + p.x(),reference.first.y() + p.y()));
    return;
  }
//rotation 
  void rotate_absolute (const Rotation& r)
  {
	reference.set_rotation(r);
	return;
  }
  void rotate_relative (const Rotation& r)
  {
    reference.set_rotation(r * reference.second);
    return;
  }

//move: translate + rotate
  void move_origin()
  {
    move_absolute(Reference_point(Point(0,0), Rotation(0,1)));
  }
  void move_absolute (const Reference_point& r)
  {
    reference=r;
    return;
  }
  void move_relative (const Reference_point& r)
  {
    reference.set_location ( Point( reference.get_location().x() + r.get_location().x(),
                                    reference.get_location().y() + r.get_location().y()) );
    reference.set_rotation ( reference.get_rotation() * r.get_rotation() );
    return;
  }
//get actual polygon
  const Polygon& get_relative_polygon() const
  {
	  return polygon;
  }
  Polygon get_absolute_polygon() const
  {
    Polygon original (polygon);
    Polygon rotated,translated,final;
    rotate_polygon<K>     (original,rotated,reference.get_rotation(),Point(0,0));
    translate_polygon<K>  (rotated,final, reference.get_location());
    return final;
  }
  Polygon get_original_polygon() const
  {
    return polygon;
  }

//center operations:
  Point get_absolute_center() const
  {
      return reference.get_location();
  }
  Point get_original_center() const
  {
      return center;
  }
//get squared radius 
  NT get_squared_radius() const
  {
    return r_sq;
  }
//get reference point
  Reference_point get_reference_point() const { return reference;}
//dbg  
  void print() const
  {
    Data_vec<K> data_vec (Data<K> ("center    ",center),
                          Data<K> ("r_sq      ",center),
                          Data<K> ("reference ",reference) );
    return;
  }
public:
  //polygons vertices are relative to center
  Polygon polygon;
  Point	  center;		//should alwase be zero
  NT      r_sq;
  Reference_point	reference;
};

#endif  //EXTENDED_POLYGON_H