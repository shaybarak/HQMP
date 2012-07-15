#ifndef RANDOM_UTILS_H
#define RANDOM_UTILS_H

#include <CGAL/Random.h>
#include <CGAL/Polygon_set_2.h>
#include "Utils\Rotation_utils\Rotation.h"
#include "Utils\Rotation_utils\RotationUtils.h"
#include "Utils\ReferencePoint.h"

template <typename K>
class Random_utils
{
public:
  typedef CGAL::Sign                      Sign;
  typedef typename K::FT                  NT;
  typedef typename K::Point_2             Point;
  typedef typename K::Segment_2           Segment;
  typedef Reference_point<K>              Reference_point;
  typedef typename Rotation<typename NT>  Rotation;

  typedef CGAL::Polygon_2<K>              Polygon;
  typedef CGAL::Polygon_with_holes_2<K>   Polygon_with_holes;
  typedef std::vector<K>                  Polygon_vec;
  typedef std::vector<Polygon_with_holes> Polygon_with_holes_vec;
  typedef CGAL::Polygon_set_2<K>          Polygon_set;
private:
  CGAL::Random random;
public: //CTR's  
  Random_utils() {};
  Random_utils(unsigned int seed):random(seed) {};
public: //zero-dimensional random functions 
  bool	  get_bool()
  {
	return random.get_bool();
  }
  Sign    get_random_sign()
  {
    return CGAL::sign(get_random_num(-1, 1));
  }
  template <typename Number_type>
  Number_type	get_random_num(double lower = -1, double upper = 1)
  {
    return Number_type (random.get_double (lower, upper));
  }
  int get_random_int(int lower, int upper)
  {
    return random.get_int(lower, upper);
  }

public: //geometric random functions
  Point   get_random_point( double lower_x = -1, double upper_x = 1,
                            double lower_y = -1, double upper_y = 1)
  {
    return Point( get_random_num<NT>(lower_x,upper_x),
                  get_random_num<NT>(lower_y,upper_y));
  }
  Segment get_random_segment(double lower_x = -1, double upper_x = 1,
                            double lower_y = -1, double upper_y = 1)
  {
    return Segment (get_random_point(lower_x,upper_x,lower_y,upper_y),
                    get_random_point(lower_x,upper_x,lower_y,upper_y));
  }

  Rotation get_random_rotation()
  {
    return to_rotation<NT> (random.get_double (0,360),DEG);
  }
  Reference_point get_random_ref_point()
  {
    return Reference_point(get_random_point(), get_random_rotation());
  }
public: //random point in a two-dimensional area 
  Point get_random_point_in_bbox(const CGAL::Bbox_2& bbox)
  {
    return get_random_point(  bbox.xmin(), bbox.xmax(),
                              bbox.ymin(), bbox.ymax());

  }
  Point get_random_point_in_polygon(const Polygon& pgn)
  {
    CGAL::Bbox_2  bbox(pgn.bbox());
    Point         p;
    do
    {
      p = get_random_point( bbox.xmin (), bbox.xmax (),
                            bbox.ymin (), bbox.ymax ()  );
    }while (is_in_polygon<K>(p ,pgn, true) == false);
    return p;
  }
  Point get_random_point_in_polygon(const Polygon_with_holes& pgn)
  {
    CGAL::Bbox_2  bbox(pgn.outer_boundary().bbox());
    Point         p;
    do
    {
      p = get_random_point( bbox.xmin (), bbox.xmax (),
                            bbox.ymin (), bbox.ymax ()  );
    }while (is_in_polygon<K>(p,pgn,true) == false);
    return p;
  }
  Point get_random_point_in_polygon_set (const Polygon_set& pgn_set)
  {
    //TODO implement better    
    std::vector<Polygon_with_holes> pgns;
    pgn_set.polygons_with_holes(std::back_inserter(pgns));  //get polygons in set

    std::vector<Polygon_with_holes>::iterator curr (pgns.begin());
    CGAL::Bbox_2  bbox(curr->bbox());               //get bounding box of first polygon

    while (++curr != pgns.end())
      bbox = bbox + curr->bbox();

    return get_random_point_in_polygon_set (pgn_set, bbox);
  }
  Point get_random_point_in_polygon_set (const Polygon_set& pgn_set, const CGAL::Bbox_2&  bbox)
  {
    Point         p;
    Polygon_with_holes tmp_pgn;
    do
    {
      p = get_random_point( bbox.xmin (), bbox.xmax (),
                            bbox.ymin (), bbox.ymax ()  );
    }while (pgn_set.locate(p, tmp_pgn)==false);
    return p;
  }
public: //random segment in a two-dimensional area   
  template <typename Pgn>
  Segment get_random_segment_in_polygon(const Pgn& pgn)
  {
	Segment seg;
	do 
    {
      seg = Segment ( get_random_point_in_polygon(pgn),
                      get_random_point_in_polygon(pgn));
    }while (!is_in_polygon<K>(seg, pgn, true));
    return seg;
  }
  Segment get_random_segment_in_polygon_set(const Polygon_set& pgn_set)
  {
    Segment seg;
    Point p (get_random_point_in_polygon_set (pgn_set));
    Polygon_with_holes pgn;
    pgn_set.locate (p,pgn);
    Polygon_set pgn_as_set(pgn);
    do 
    {
      seg = Segment ( p,
                      get_random_point_in_polygon_set (pgn_as_set));
    }while (!is_in_polygon<K>(seg, pgn, true));
    return seg;
  }
  Segment get_small_random_segment_in_polygon_set(const Polygon_set& pgn_set,double alpha = 0.3)
  {
    Segment seg (get_random_segment_in_polygon_set(pgn_set));
    Segment scaled_seg (scaled_down_segment(seg, alpha));
    return scaled_seg ;
  }
  Segment get_small_random_segment_in_polygon(const Polygon& pgn,double alpha = 0.3)
  {
    Segment seg (get_random_segment_in_polygon(pgn));
    Segment scaled_seg (scaled_down_segment(seg, alpha));
    return scaled_seg ;
  }
private:
  Segment scaled_down_segment(const Segment& seg,double alpha)
  {
    CGAL_precondition(alpha < 1);
    return (  get_bool ()   ?    //determine sign
              Segment(seg.source(),
                      Point(seg.target().x() - (1-alpha)*(seg.target().x()-seg.source().x()),
                            seg.target().y() - (1-alpha)*(seg.target().y()-seg.source().y())) ) :
              Segment(seg.target(),
                      Point(seg.source().x() - (1-alpha)*(seg.source().x()-seg.target().x()),
                            seg.source().y() - (1-alpha)*(seg.source().y()-seg.target().y())) ));
  }
}; //Random_utils
#endif //RANDOM_UTILS_H