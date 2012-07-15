#ifndef CRITICAL_VALUES_H
#define CRITICAL_VALUES_H

#include "Utils\Polygon_utils\ExtendedPolygon.h"
#include "Utils\Rotation_utils\Rotation.h"
#include "Configuration_spaces\PointPrimitive\CriticalValuesConstructor.h"

namespace FixedPoint
{

template <typename K, typename AK, typename AK_conversions>
class Critical_values
{
private:
  typedef Critical_values_constructor <K, AK, AK_conversions>     Critical_values_constructor;
public:
  typedef typename Critical_values_constructor::Point             Point;
  typedef typename Critical_values_constructor::Segment           Segment;
  typedef typename Critical_values_constructor::Rotation          Rotation;
  typedef typename Critical_values_constructor::Rotation_range    Rotation_range;

  typedef Extended_polygon<K>                       Extended_polygon;
  typedef typename Extended_polygon::Polygon        Polygon;

  typedef typename AK::Algebraic_real_1	            Algebraic;

  typedef std::vector <typename Point>              Point_vec;
  typedef std::vector <typename Segment>            Segment_vec;
  typedef std::vector <Algebraic>                   Algebraic_vec;

private:
  Polygon                     _robot;
  Critical_values_constructor _values_constructor;
public:
//constructors
  Critical_values (const Extended_polygon&  robot,
                   const Point&             p,
                   const Rotation_range&    rotation_range,
                   AK& ak)
                   :  _robot(robot.get_original_polygon()),
                      _values_constructor(robot.get_absolute_center(), p, rotation_range, ak)
  {}

  template <typename OutputIterator>
  void get_critical_values(const Point_vec& obstacle_vertices,
                           const Segment_vec& obstacle_edges,
                           OutputIterator& oi)
  {
    Polygon::Vertex_iterator      robot_vi;
    Segment_vec::const_iterator   obstacle_ei;

    for (robot_vi=_robot.vertices_begin (); robot_vi!=_robot.vertices_end(); ++robot_vi)
    {
      for (obstacle_ei=obstacle_edges.begin (); obstacle_ei!=obstacle_edges.end(); ++obstacle_ei)
      {
        _values_constructor.construct_values(*robot_vi, *obstacle_ei, oi);
      }
    }
   
    Polygon::Edge_const_iterator  robot_ei;
    Point_vec::const_iterator     obstacle_vi;
    
    for (robot_ei=_robot.edges_begin (); robot_ei!=_robot.edges_end(); ++robot_ei)
    {
      for (obstacle_vi=obstacle_vertices.begin (); obstacle_vi!=obstacle_vertices.end(); ++obstacle_vi)
      {
        _values_constructor.construct_values(*robot_ei, *obstacle_vi, oi);
      }
    }
    return;
  }
};  //Critical_values

}//FixedPoint

#endif // CRITICAL_VALUES_H