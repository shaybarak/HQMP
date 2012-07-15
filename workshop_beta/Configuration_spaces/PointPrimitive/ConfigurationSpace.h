#ifndef CONFIGURATION_SPACE_FP_H
#define CONFIGURATION_SPACE_FP_H

#include "Utils\Geometry_utils\GetClosestUtils.h"
#include "Utils\Polygon_utils\ExtendedPolygon.h"
#include "Utils\ReferencePoint.h"
#include "Utils\Interval_utils\IntervalSet.h"
#include "Configuration_spaces\PointPrimitive\CriticalValues.h"
#include "Configuration_spaces\PointPrimitive\Fixed_point_utils.h"

namespace FixedPoint
{

template <typename K, typename AK, typename AK_conversions>
class Configuration_space
{
private:
  typedef typename K::FT                        NT;
  typedef typename K::Point_2                   Point;
  typedef typename K::Segment_2                 Segment;
  typedef typename Rotation<NT>                 Rotation;
  typedef typename Rotation_range_absolute<NT>  Rotation_range;
  typedef typename Reference_point<K>           Ref_point;
public:
  typedef Configuration_space<typename K, typename AK, AK_conversions>  Self;
  typedef typename AK::Algebraic_real_1	                Algebraic;
  typedef Interval_set<Algebraic, AK_conversions >      Interval_set;
  typedef typename Interval_set::Interval               Interval;
private:
  typedef std::map<int, int>          Int_int_map;
  typedef Int_int_map::iterator       Int_int_map_iter;
  typedef std::multimap<int, int>     Int_int_multimap;
  typedef Int_int_multimap::iterator  Int_int_multimap_iter;

private:
  Point             _p;  
  Rotation_range    _rotation_range;

  std::vector<Algebraic>    _critical_values;     //subdivied the line into intervals
  std::vector<bool>         _is_hinted_interval;  //a bool for each interval noting if it was hinted
  Int_int_map               _internal_external_map;   //mapping between internal and external ids
  Int_int_multimap          _external_internal_mmap;  //mapping between external and internal ids
  
  int _id;
  AK& _ak;
  AK_conversions _ak_convertor;
public:
  //constructor
  Configuration_space(AK &ak) : _ak(ak), _id(0), _ak_convertor(ak){}
  void copy (const Self& other)
  {
    if (this == &other) // protect against invalid self-assignment
      return ;

    _p = other.p();
    _rotation_range = other.rotation_range();

    _critical_values.insert(_critical_values.begin(), 
                            other.critical_values().begin(),
                            other.critical_values().end());
    _is_hinted_interval.insert( _is_hinted_interval.begin(), 
                                other.is_hinted_interval().begin(),
                                other.is_hinted_interval().end());
    _internal_external_map.insert(other.internal_external_map().begin(),
                                  other.internal_external_map().end());
    _external_internal_mmap.insert(other.external_internal_mmap().begin(),
                                   other.external_internal_mmap().end());
  
    _id = other.id();
 
    return ;
  }
  void decompose( const Extended_polygon<K>& robot, const std::vector<CGAL::Polygon_2<K> >& workspace,
                  const Point& p, const Rotation_range& rotation_range = Rotation_range())
  {
    _p = p;
    _rotation_range = rotation_range;

    //Filter obtacles that are far from the translation line
    std::vector<Point>     obstacle_vertices;
    std::vector <Segment>  obstacle_edges;

    filter_obstacles( robot, workspace,                   //input         
                      obstacle_vertices, obstacle_edges); //output

    //Get the critical values
    //if the range is not full, insert values of theta = range.first, theta = range.second, 
    if (_rotation_range.is_full_range() == false)
    {
      NT cv_1 (get_parametrization_theta<K> (_rotation_range.get_start_rotation()));
      NT cv_2 (get_parametrization_theta<K> (_rotation_range.get_end_rotation()));

      _critical_values.push_back(_ak_convertor.convert(cv_1));
      _critical_values.push_back(_ak_convertor.convert(cv_2));
    }

    Critical_values<K, AK, AK_conversions> critical_values_computor(robot, _p, _rotation_range, _ak);
    critical_values_computor.get_critical_values(obstacle_vertices, obstacle_edges,
                                                 std::back_inserter(_critical_values));

    //  sort the critical values
    std::sort(_critical_values.begin(), _critical_values.end());

    //  initialize all intervals as unhinted
    unsigned int num_of_intervals = _critical_values.size() + 1;
    _is_hinted_interval.reserve(num_of_intervals);
    for (unsigned int i(0); i < num_of_intervals; ++i)
      _is_hinted_interval.push_back(false);

    return;
  }
  void add_obstacle(const Extended_polygon<K>& robot, CGAL::Polygon_2<K>& obstacle)
  {
    //reset data structures
    _is_hinted_interval.clear();      //a bool for each interval noting if it was hinted
    _internal_external_map.clear();   //mapping between internal and external ids
    _external_internal_mmap.clear();  //mapping between external and internal ids

    //Filter obtacles that are far from the translation line
    std::vector<Point>     obstacle_vertices;
    std::vector <Segment>  obstacle_edges;

    filter_obstacles( robot, obstacle,                      //input         
                      obstacle_vertices, obstacle_edges);   //output

    
    //Update the critical values
    Critical_values<K, AK, AK_conversions> critical_values_computor(robot, _p, _rotation_range, _ak);
    critical_values_computor.get_critical_values(obstacle_vertices, obstacle_edges,
                                                 std::back_inserter(_critical_values));

    //  sort the critical values
    std::sort(_critical_values.begin(), _critical_values.end());

    //  initialize all intervals as unhinted
    unsigned int num_of_intervals = _critical_values.size() + 1;
    
    _is_hinted_interval.reserve(num_of_intervals);
    for (unsigned int i(0); i < num_of_intervals; ++i)
      _is_hinted_interval.push_back(false);

    return;

  }
  int free_space_location_hint (const Ref_point& ref_point)
  {
    CGAL_precondition(ref_point.get_location() == _p);
    CGAL_precondition(_rotation_range.is_in_range(ref_point.get_rotation()));
    NT t = get_parametrization_theta<K>(ref_point.get_rotation());
    return free_space_location_hint(t);
  }
  
  void get_interval(unsigned int external_interval_id, Interval_set& interval)
  {
    CGAL_precondition (interval.is_unbounded());
    if (_critical_values.empty())
    {      
      CGAL_precondition (external_interval_id ==0);
      return;
    }

    CGAL_precondition (_external_internal_mmap.find(external_interval_id) != _external_internal_mmap.end());
    
    std::pair<Int_int_multimap_iter, Int_int_multimap_iter> iter_pair;
    iter_pair = _external_internal_mmap.equal_range(external_interval_id);
    std::vector<int> internal_interval_ids;
    while(iter_pair.first != iter_pair.second)
    {
      internal_interval_ids.push_back(iter_pair.first->second);
      iter_pair.first++;
    }

    CGAL_postcondition( (internal_interval_ids.size() == 1) ||
                        (internal_interval_ids.size() == 2) );

    if (internal_interval_ids.size() == 1)
    {
      int internal_interval_id = internal_interval_ids.front();
      CGAL_precondition(  (internal_interval_id != 0) &&
                          (internal_interval_id != _critical_values.size()) );
      Algebraic& a = _critical_values[internal_interval_id -1];
      Algebraic& b = _critical_values[internal_interval_id ];
      interval.insert_and(Interval(a, b));
    }
    else
    {
      CGAL_precondition (((internal_interval_ids.front() == 0) &&
                          (internal_interval_ids.back() == _critical_values.size())) ||
                          ((internal_interval_ids.back() == 0) &&
                          (internal_interval_ids.front() == _critical_values.size())));
      Algebraic& a = _critical_values.front();
      Algebraic& b = _critical_values.back();
      CGAL_postcondition (a != b);
      interval.insert_and_complement(Interval(a,b));
    }
    return;
  }
public:
  Point&  get_fixed_point()
  {
    return  _p;
  }
public: 
  
  const Point& p() const {return _p;}
  const Rotation_range& rotation_range() const {return _rotation_range;}
  const std::vector<Algebraic>& critical_values() const {return _critical_values;}
  const std::vector<bool>& is_hinted_interval() const {return _is_hinted_interval;}
  const Int_int_map& internal_external_map() const {return _internal_external_map;}
  const Int_int_multimap& external_internal_mmap() const {return _external_internal_mmap;}
  int id() const {return _id;}
  
private:
  void filter_obstacles(  const Extended_polygon<K>& placed_robot, 
                          const std::vector<CGAL::Polygon_2<K> >& workspace,
                          std::vector <Point>&   obstacle_vertices,
						  std::vector <Segment>& obstacle_edges)
{
  std::vector<CGAL::Polygon_2<K> >::const_iterator  oi;
  for (oi = workspace.begin(); oi!= workspace.end(); ++oi)
  {
    filter_obstacles( placed_robot, *oi, 
                      obstacle_vertices, obstacle_edges);
  }

  return;
}
  void filter_obstacles(  const Extended_polygon<K>& placed_robot, 
                          const CGAL::Polygon_2<K> & obstacle,
                          std::vector <Point>&   obstacle_vertices,
						  std::vector <Segment>& obstacle_edges)
  {
    NT sq_r (placed_robot.get_squared_radius());

    CGAL::Polygon_2<K>::Vertex_iterator               vi;
    CGAL::Polygon_2<K>::Edge_const_iterator           ei;

    for (vi=obstacle.vertices_begin(); vi!=obstacle.vertices_end(); ++vi)
    {
      if (CGAL::squared_distance(_p,*vi) <= sq_r) 
      {
        obstacle_vertices.push_back(*vi);
      }
    }
    
    for (ei=obstacle.edges_begin  (); ei!=obstacle.edges_end (); ++ei)
    {
      if (CGAL::squared_distance(_p, *ei) <= sq_r)
      {
        obstacle_edges.push_back(*ei);
      }
    }
    return;
  }
  int free_space_location_hint (const NT& t)
  {
    if (_critical_values.empty())
    {
      _is_hinted_interval.front() = true;
      return 0;
    }

    CGAL_precondition (_critical_values.empty() != true);
    Algebraic a = _ak_convertor.convert(t);
    unsigned int internal_interval_num = get_containing_interval(a);
    if (_is_hinted_interval[internal_interval_num] == false)
    {
      unsigned int external_interval_id = get_new_external_id();
      
      //mark interval as hinted and update maps
      _is_hinted_interval[internal_interval_num] = true;
      _internal_external_map .insert(std::make_pair(internal_interval_num, external_interval_id));
      _external_internal_mmap.insert(std::make_pair(external_interval_id, internal_interval_num));

      bool identified_interval = is_identified_interval(internal_interval_num);
     
      CGAL_postcondition ((identified_interval) ||
                          ((internal_interval_num != 0 ) && (internal_interval_num != _critical_values.size() )));

      if (is_identified_interval(internal_interval_num))
      {
        unsigned int identified_internal_interval_num;
        identified_internal_interval_num = (internal_interval_num == 0) ?
                                            _critical_values.size() : 0;
        _is_hinted_interval[identified_internal_interval_num] = true;

        _internal_external_map .insert(std::make_pair(identified_internal_interval_num, external_interval_id));
        _external_internal_mmap.insert(std::make_pair(external_interval_id, identified_internal_interval_num));
      }

      return external_interval_id;
    }
    else //(_is_hinted_interval[internal_interval_num] == true)
    {
      CGAL_precondition (_internal_external_map.find(internal_interval_num) != _internal_external_map.end());
      return _internal_external_map[internal_interval_num];
    }
  }
  bool is_identified_interval(int internal_interval_num)
  {
    if (_critical_values.empty())
    {
      CGAL_precondition (internal_interval_num ==0);
      return true;
    }

    if ((internal_interval_num == 0) || 
        (internal_interval_num == _critical_values.size()))
         return true;

    return false;
  }
  unsigned int get_containing_interval(Algebraic& t)
  {
    if (_critical_values.empty())
      return 0;
    
    int low = 0;
    int high = _critical_values.size() - 1 ;
    //CGAL_postcondition (low != high);

    if (CGAL::compare(t, _critical_values.front()) == CGAL::SMALLER)
      return 0;

    if (CGAL::compare(t, _critical_values.back()) == CGAL::LARGER)
      return _critical_values.size();

    CGAL_postcondition (CGAL::compare(t, _critical_values.front()) != CGAL::EQUAL);
    CGAL_postcondition (CGAL::compare(t, _critical_values.back()) != CGAL::EQUAL);


    while (low+1 < high)
    {
      int mid = static_cast<int> (low + high) / 2;
      Algebraic val = _critical_values[mid];
      CGAL_postcondition (CGAL::compare(val ,t) != CGAL::EQUAL);

      if (CGAL::compare(t, val) == CGAL::SMALLER)
        high = mid;
      else
        low = mid;
    }
    return high;
  }
  int get_new_external_id()
  {
    return _id++;
  }
};  //Configuration_space
} //namespace FixedPoint
#endif // CONFIGURATION_SPACE_FP_H