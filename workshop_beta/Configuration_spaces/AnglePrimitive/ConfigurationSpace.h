#ifndef CONFIGURATION_SPACE_FTFR_H
#define CONFIGURATION_SPACE_FTFR_H

#include <CGAL/minkowski_sum_2.h>
#include <CGAL/Polygon_convex_decomposition_2.h>

#ifdef ANGLE_BISECTOR
#include <CGAL/Small_side_angle_bisector_decomposition_2.h>
#endif //ANGLE_BISECTOR

#include "Utils\Polygon_utils\Smart_polygon_with_holes.h"
#include "Utils\Polygon_utils\PolygonIO.h"

namespace FixedRotation
{
  ///////////////////////////////////////////////////
  //  compute the minkowski sum betwen two polygons
  ///////////////////////////////////////////////////
  template <typename K>
  CGAL::Polygon_with_holes_2<K> minkowski_sum(const CGAL::Polygon_2<K> & poly1,
                                              const CGAL::Polygon_2<K> & poly2)
  {
    CGAL::Polygon_with_holes_2<K> sum;
    
    try
    {

#ifdef MINKWOSKI_WITH_DECOMPOSITION
#ifdef ANGLE_BISECTOR
      CGAL::Small_side_angle_bisector_decomposition_2<K> decomp;
#endif //ANGLE_BISECTOR
#ifdef HERTEL_MELHORN
      CGAL::Hertel_Mehlhorn_convex_decomposition_2<K>     decomp;
#endif  //HERTEL_MELHORN
        sum  = minkowski_sum_2 (poly1, poly2,decomp);
#else //MINKWOSKI_WITH_DECOMPOSITION
		sum  = minkowski_sum_2 (poly1, poly2);
#endif //MINKWOSKI_WITH_DECOMPOSITION
    
    }
    catch (...)
    {
      std::cout<<"***************************************"<<std::endl;
      std::cout<<"***   bug in minkowski_sum         ****"<<std::endl;
      std::cout<<"***************************************"<<std::endl;      
    }
	return sum;
  }


  ///////////////////////////////////////////////////
  //  compute c_space_obstacle of robot and obstacle
  ///////////////////////////////////////////////////
  template <typename K> 
  void get_c_space_obstacle( const CGAL::Polygon_2<K> & robot,
                             const CGAL::Polygon_2<K> & obstacle, 
                             CGAL::Polygon_with_holes_2<K> & c_space_obstacle)
  {
    Polygon robot_in_the_middle, minus_robot_in_the_middle;
    center_polygon_ref_point_at_zero<K>(robot, robot_in_the_middle);
    negate_polygon<K>(robot_in_the_middle, minus_robot_in_the_middle);

    CGAL::Polygon_with_holes_2<K> sum;
    sum = minkowski_sum<Kernel>(minus_robot_in_the_middle, obstacle);

    //cannonicalize by translating each obstacle back
    Point reference_point (get_reference_point<K>(robot));
    Point translation     ( -reference_point.x(),-reference_point.y());
    translate_polygon<K> (sum, c_space_obstacle, translation);

    return;
  }
template<typename Kernel>
class Configuration_space
{
public:
  typedef Configuration_space<Kernel>         Self;
  typedef typename Kernel::Point_2            Point;
  typedef CGAL::Polygon_2 <Kernel>            Polygon;
  typedef std::vector<typename Polygon>       Polygon_vec;
  typedef CGAL::Polygon_with_holes_2<Kernel>  Polygon_with_holes;
  typedef Smart_polygon_with_holes<Kernel>    Smart_polygon;
  typedef std::vector<typename Smart_polygon> Smart_polygon_vec;
  typedef CGAL::Polygon_set_2<Kernel>         Polygon_set;

private:
  Polygon_set         free_space;
  Smart_polygon_vec   polygons;
public:
  Configuration_space () {};
  void copy(const Self &other, bool copy_smart_polygons) 
  {
    if (this == &other) // protect against invalid self-assignment
      return;
 
    // 1: deallocate old memory
    free_space.clear();
    polygons.clear();

    // 2: allocate new memory and copy the elements
    free_space = other.get_free_space();
    
    if (copy_smart_polygons)
    {
      BOOST_FOREACH(Smart_polygon polygon, other.get_polygons())
        polygons.push_back(polygon)
    }
    return;
  }
public:
  void add_c_space_obstacle(Polygon_with_holes& c_space_obstacle)
  {
    polygons.clear();

    free_space.complement();            //this is the forbidden set
    free_space.join(c_space_obstacle);  // added new obstacle
    free_space.complement();            //back to free space

    std::vector<Polygon_with_holes> raw_polygons;
    free_space.polygons_with_holes (std::back_inserter(raw_polygons));

    BOOST_FOREACH (Polygon_with_holes pgn, raw_polygons)
      polygons.push_back(Smart_polygon(pgn));
  }
  void add_obstacle(Polygon& robot, Polygon& obstacle)
  {
    Polygon_with_holes c_space_obs;
    get_c_space_obstacle(robot, obstacle, c_space_obs);
    add_c_space_obstacle(c_space_obs);
    return;
  }
  void decompose(const Polygon& robot, const Polygon_vec& workspace)
  {
    Polygon robot_in_the_middle,minus_robot_in_the_middle;
    center_polygon_ref_point_at_zero<Kernel>(robot,robot_in_the_middle);
    negate_polygon<Kernel>(robot_in_the_middle,minus_robot_in_the_middle);

    //compute sum for each obstacle
    std::vector<Polygon_with_holes> sums;
    BOOST_FOREACH(Polygon obstacle, workspace)
      sums.push_back(minkowski_sum<Kernel>(minus_robot_in_the_middle, obstacle));
    
    //cannonicalize by translating each obstacle back
    Point reference_point (get_reference_point<Kernel>(robot));
    Point translation     ( -reference_point.x(),-reference_point.y());
    std::vector<Polygon_with_holes> cannonicalized_sums;
    BOOST_FOREACH (Polygon_with_holes poly, sums)
    {
      Polygon_with_holes   translated;
      translate_polygon<Kernel> (poly, translated, translation );
      cannonicalized_sums.push_back(translated);
    }

    //free_space.join(sums.begin(),sums.end());
    free_space.join(cannonicalized_sums.begin(), cannonicalized_sums.end());
    free_space.complement ();

    std::vector<Polygon_with_holes> raw_polygons;
    free_space.polygons_with_holes (std::back_inserter(raw_polygons));

	  BOOST_FOREACH (Polygon_with_holes pgn, raw_polygons)
	  {
      //patch to overcome bug in polygon set
	    bool bug = false;
	    if (pgn.is_unbounded() == false)
		    if(pgn.outer_boundary().is_simple() == false)
			    bug = true;

	    if (bug) //bug in polygon_set simply ignore...
      {
	      std::cout<<"***   bug in polygon_set         ****"<<std::endl;
        continue;
      }
      //end of patch
      
      
      if (pgn.is_unbounded() == false)
        polygons.push_back(Smart_polygon(pgn));
    }

    return;
  }
public:
  const Polygon_set& get_free_space() const
  {
    return free_space;
  }
  const Smart_polygon_vec& get_polygons() const
  {
    return polygons;
  }
  const Smart_polygon& get_fsc(const int fsc_id)
  {
    CGAL_precondition ( (fsc_id >= 0) && (fsc_id < (int)polygons.size()) );
    return polygons[fsc_id];
  }
  int get_fsc_id(const Point& p)
  {
    for (unsigned int id =0; id < polygons.size(); ++id)
    {
      if (polygons[id].is_in_polygon(p,true))
        return id;
    }
    return mms::NO_ID; 
  }
  unsigned int num_of_fscs() const
  {
    return polygons.size();
  }    
public:
  bool is_free(const Point& p)
  {
    if (free_space.oriented_side(p) == CGAL::ON_NEGATIVE_SIDE)
      return false;
    
    Polygon_with_holes pgn;
    free_space.locate (p, pgn);
    return (pgn.is_unbounded() == false);
  }
}; //Configuration_space
} // namespace FixedRotation

#endif  //CONFIGURATION_SPACE_FTFR_H