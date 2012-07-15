#ifndef MMS_EXAMPLE_H
#define MMS_EXAMPLE_H

#include "Manifolds\MMSTypedefs.h"
#include "Manifolds\Fsc_indx.h"
#include "Manifolds\Fixed_angle\Fixed_angle_manifold_container.h"
#include "Manifolds\Fixed_point\Fixed_point_manifold_container.h"
#include "Manifolds\Intersect_manifolds.h"
#include "Path_planning\PathPlanningUtils.h"
#include "heuristic_utils.h"
#include "FSC.h"
#include "Graph\Graph.h"
#include "Fsc_path_planning.h"

namespace mms{

template <typename K_ = Rational_kernel, 
          typename AK_ = CGAL::Algebraic_kernel_d_1 <typename CGAL::CORE_arithmetic_kernel::Integer>, 
          typename AK_conversions = Algebraic_kernel_d_1_conversions_rational<typename CGAL::CORE_arithmetic_kernel> >
class Mms_path_planner_example
{
public:
  typedef typename K_                             K;
  typedef typename AK_                            AK;

  typedef typename K::Point_2                     Point;
  typedef Rotation<typename K::FT>                Rotation;
  typedef typename Reference_point<K>             Reference_point;
  typedef Rotation_range_absolute<typename K::FT> Rotation_range;

  typedef CGAL::Polygon_2 <K>                     Polygon;
  typedef CGAL::Polygon_with_holes_2<K>           Polygon_with_holes;
  typedef typename Extended_polygon<K>            Extended_polygon;
  typedef typename Smart_polygon_with_holes<K>    Smart_polygon;
  
  typedef std::vector<typename Polygon>           Polygon_vec;
  typedef CGAL::Polygon_set_2<K>                  Polygon_set;

  typedef Motion_step_rotational<K>               Motion_step_rotational;
  typedef Motion_step_translational<K>            Motion_step_translational;
  typedef Motion_sequence<K>                      Motion_sequence;

  
public:
  typedef Fsc_indx<K>                             Fsc_indx;
  typedef FSC<K, AK, AK_conversions>              Fsc;

  typedef Fixed_angle_manifold_container<K>       Layers;
  typedef typename Layers::Manifold               Layer;

  typedef Fixed_point_manifold_container<K, AK, AK_conversions> C_space_lines;
  typedef typename C_space_lines::Manifold                      C_space_line;

  typedef Graph<Fsc_indx, Less_than_fsc_indx<K> > Connectiivity_graph;
  typedef Random_utils<K>                         Random_utils;  
private:
  Polygon_vec&            _workspace;
  Polygon_vec             _decomposed_workspace;
  CGAL::Bbox_2            _workspace_bbox;
  Extended_polygon&       _robot;
  
  Connectiivity_graph     _graph;

  std::vector<Rotation>   _rotations;
  Layers                  _layers;
  C_space_lines           _lines;
    
  Random_utils            _rand;
  AK                      _ak;
public:
  //constructor
  Mms_path_planner_example  (Polygon_vec &workspace, Extended_polygon& robot)
    : _workspace (workspace), _robot(robot),
      _graph(0,true), _rand(time(NULL))
  {
    compute_workspace_bbox();
    //the minkowski sum algorithm works faster on polygons are convex
    //hence we decompose the workspace obstacles to convex polygons
    decompose_workspace_into_convex_polygons();
  }
  //preprocess
  void preprocess   (const unsigned int num_of_angles = configuration.get_slices_granularity())
  {
    generate_rotations(num_of_angles);
    
    BOOST_FOREACH (Rotation rotation, _rotations)
      add_layer(rotation);
    global_tm.write_time_log(std::string("finished layers"));

    generate_connectors();    
    global_tm.write_time_log(std::string("finished connectors"));
    
    global_tm.write_time_log(std::string("finished preproccesing"));
    return;
  }
  //query
  bool query( const Reference_point& source, const Reference_point& target,
              Motion_sequence& motion_sequence) 
  {
    ////////////////////////////////////
    //connect source and target to graph
    ////////////////////////////////////
    Motion_sequence source_motion_sequence, target_motion_sequence;
    Reference_point perturbed_source = connect_to_graph(source, source_motion_sequence);
    Reference_point perturbed_target = connect_to_graph(target, target_motion_sequence);
    
    if (perturbed_source ==  Reference_point() || 
        perturbed_target ==  Reference_point())
    {
      std::cout <<"failed to connect to pre-processed configuration space"<<std::endl;
      return false;
    }

    target_motion_sequence.reverse_motion_sequence();

    ////////////////////////////////////
    //find path of fscs(if exists)
    ////////////////////////////////////
    Fsc_indx source_fsc_indx (get_containig_fsc(perturbed_source));
    CGAL_postcondition (source_fsc_indx != Fsc_indx());
    Fsc_indx target_fsc_indx (get_containig_fsc(perturbed_target));
    CGAL_postcondition (target_fsc_indx != Fsc_indx());
    
    std::list<Fsc_indx> fsc_indx_path;
    if (source_fsc_indx == target_fsc_indx)
      fsc_indx_path.push_back(source_fsc_indx);
    else
      _graph.find_path( source_fsc_indx, target_fsc_indx, fsc_indx_path);
    
    if (fsc_indx_path.empty())
      return false;

    ////////////////////////////////////
    //construct motion sequence
    ////////////////////////////////////
    //(1) add source motion
    motion_sequence.add_motion_sequence(source_motion_sequence);
    
    //(2) construct real motion sequence from connectivity graph
    int             curr_fsc_indx = 0;
    Reference_point curr_ref_p    = perturbed_source;

    std::list<Fsc_indx>::iterator curr, next;

    next = curr = fsc_indx_path.begin();
    ++next;
    while (next != fsc_indx_path.end())
    {
      Reference_point  next_ref_p = get_intersection(*curr, *next);

      Fsc* fsc_ptr = get_fsc(*curr);
      CGAL_postcondition(fsc_ptr->contains(curr_ref_p));
      CGAL_postcondition(fsc_ptr->contains(next_ref_p));
      CGAL_precondition ( (motion_sequence.get_sequence().empty()) || 
                          (motion_sequence.get_sequence().back()->target() == curr_ref_p) );
      plan_path(fsc_ptr, curr_ref_p, next_ref_p, motion_sequence);
      CGAL_precondition ( (motion_sequence.get_sequence().empty()) || 
                          (motion_sequence.get_sequence().back()->target() == next_ref_p) );
      
      curr++;
      next++;
      curr_ref_p = next_ref_p;
      delete fsc_ptr;
    }

    Fsc* fsc_ptr = get_fsc(*curr);
    plan_path(fsc_ptr, curr_ref_p, perturbed_target, motion_sequence);
    delete fsc_ptr;

    //(3) add source motion
    motion_sequence.add_motion_sequence(target_motion_sequence);
    return true;
  }
private: //layer methods
  void generate_rotations(const unsigned int num_of_angles)
  {
    std::vector<Rotation> tmp_rotations;
    double step ((double)360/num_of_angles);
    Rotation init_rotation(_rand.get_random_rotation());

    for (double r(0); r<360; r+=step)
      tmp_rotations.push_back(init_rotation* to_rotation<K::FT>(r,DEG,FINE));

    //maybe some rotations are the same as approximation is not good enough, remove duplicates 
    Less_than_rotation<K::FT> less_than;
    std::sort(tmp_rotations.begin(),tmp_rotations.end(), less_than);
    std::vector<Rotation>::iterator new_end = std::unique(tmp_rotations.begin(), tmp_rotations.end());

    //insert unique rotations (except maybe last) to the vector
    _rotations.clear();
    _rotations.insert(_rotations.begin(), tmp_rotations.begin(), new_end);

    //maybe last rotation is zero like the first rotation
    if (_rotations.back() == _rotations.front())
      _rotations.pop_back();

     return;
  }

  void add_layer(const Rotation& rotation)
  {
    //create layer
    Layer* layer_ptr = new Layer (Layer::Constraint(rotation));
    layer_ptr->decompose(_robot, _decomposed_workspace);
    int layer_id = _layers.add_manifold(layer_ptr);

    update_connectivity_graph_vertices(*layer_ptr, layer_id);
    return;
  }
private:
  void generate_connectors()
  {
    generate_connectors_random();
  }
  void generate_connectors_random()
  {
    for (int i(0); i < configuration.get_max_num_of_intra_connections(); ++i)
      generate_connector_random();
  }
  void generate_connector_random()
  {
    ////////////////////////////////////////////////////////////////
    //get free point in the configuration space on one of the layers
    ////////////////////////////////////////////////////////////////
    Rotation& r  = _rotations[_rand.get_random_int(0, _rotations.size()-1)]; //random layer
    int layer_id = _layers.get_containing_manifold_id(r);
    Layer* layer_ptr = _layers.get_manifold(layer_id);
    Point p = _rand.get_random_point_in_bbox(_workspace_bbox); 
	
	
    while (layer_ptr->is_free(p) == false)
		p = _rand.get_random_point_in_bbox(_workspace_bbox);

	int cell_id  = layer_ptr->get_containing_cell(p); 
	if (cell_id == NO_ID)
		return;// this should NOT be NO_ID but there is a patch in the configuration space of the angle primitive due to a bug in polygon_est_2
	


    C_space_line* c_space_line_ptr; 
    C_space_line::Constraint constraint;
    ////////////////////////////////////////////////////////////////
    //choose roi
    ////////////////////////////////////////////////////////////////
    double cell_size_ratio = get_size_percentage(layer_ptr->get_fsc(cell_id).cell() );
    CGAL_precondition (cell_size_ratio >=0 && cell_size_ratio <=1);

    if (configuration.get_use_region_of_interest() &&
        cell_size_ratio < 1 )
    {
      double small_rotation(configuration.get_rotation_range()/2);
      CGAL_precondition(small_rotation < 180);
      double half_range_double = small_rotation + cell_size_ratio * (180 - small_rotation);
      Rotation half_range = to_rotation<K::FT>(half_range_double, DEG);
      Rotation_range range(r*(-half_range), r*half_range);      
      
      constraint = C_space_line::Constraint(p, range);

    }
    else
    {
      constraint = C_space_line::Constraint(p);
    }

    ////////////////////////////////////////////////////////////////
    //attempt to filter
    ////////////////////////////////////////////////////////////////
    if (filter_out(constraint))
      return;

    ////////////////////////////////////////////////////////////////
    //create connector
    ////////////////////////////////////////////////////////////////
    c_space_line_ptr = new C_space_line (constraint, _ak);
    c_space_line_ptr->decompose(_robot, _decomposed_workspace);
    int c_space_line_id = _lines.add_manifold(c_space_line_ptr);

    ////////////////////////////////////////////////////////////////
    //update connectivity graph
    ////////////////////////////////////////////////////////////////
    update_connectivity_graph(c_space_line_id);
    return;

  }
private: //filtering methods
  bool filter_out(typename C_space_line::Constraint& constraint)
  {
    if (configuration.get_use_filtering() == false)
      return false;

    std::vector<int> intersecting_layer_ids;
    _layers.get_interseceting_manifolds(constraint.region_of_interest(), 
                                        std::back_inserter(intersecting_layer_ids));

    int cc_id = NO_ID;
    BOOST_FOREACH(int layer_id, intersecting_layer_ids)
    {
      Layer* layer_ptr = _layers.get_manifold(layer_id);
      const Rotation& rotation = layer_ptr->constraint().restriction();

      //find point in layer;
      int fsc_id = layer_ptr->get_containing_cell(constraint.restriction());
      if (fsc_id == NO_ID)
        continue;
      Fsc_indx curr_fsc_index(FIXED_ANGLE, layer_id, fsc_id);

      int curr_cc_id = _graph.get_cc_id(curr_fsc_index);
      if (cc_id == NO_ID)
        cc_id = curr_cc_id;
      else if (cc_id != curr_cc_id)
        return false;
    }

    return true;
  }

private: //Connectiivity_graph methods
  void update_connectivity_graph_vertices(Layer& layer, int layer_id)
  {
    for (int fsc_id(0); fsc_id<layer.num_of_fscs(); ++fsc_id)
    {
      Fsc_indx layer_fsc_indx(FIXED_ANGLE, layer_id, fsc_id);
      _graph.add_vertex(layer_fsc_indx);
    }
    return;
  }
  void update_connectivity_graph(int c_space_line_id)
  {
    CGAL_precondition (c_space_line_id != NO_ID);
    
    C_space_line* line_ptr = _lines.get_manifold(c_space_line_id);
    const Rotation_range& range = line_ptr->constraint().region_of_interest();
    std::vector<int> intersecting_layer_ids;

    _layers.get_interseceting_manifolds( range, std::back_inserter(intersecting_layer_ids));
    BOOST_FOREACH(int layer_id, intersecting_layer_ids)
    {
      Layer* layer_ptr = _layers.get_manifold(layer_id);
          
      Int_pair edges_ids;
      intersect<K> (*layer_ptr, *line_ptr, edges_ids);

      CGAL_postcondition (  ((edges_ids.first == NO_ID) && (edges_ids.second == NO_ID)) ||
                            ((edges_ids.first != NO_ID) && (edges_ids.second != NO_ID)) );
      if (edges_ids.first == NO_ID)
        continue;

      Fsc_indx layer_fsc_indx(FIXED_ANGLE, layer_id, edges_ids.first);
      Fsc_indx line_fsc_indx (FIXED_POINT, c_space_line_id, edges_ids.second);

      _graph.add_vertex(layer_fsc_indx);
      _graph.add_vertex(line_fsc_indx);
      _graph.add_edge(layer_fsc_indx, line_fsc_indx);
    }
    return;
  }  
private: //query related methods
  Reference_point connect_to_graph( const Reference_point& ref_p,
                                    Motion_sequence& motion_sequence)
  {
    //(1) find closest base layer
    Point location(ref_p.get_location());
    Rotation rotation(ref_p.get_rotation());
    
    int closest_layer_id = _layers.get_closest_layer_id(rotation);
    
    //(2) get the layer
    Layer* layer_ptr = _layers.get_manifold(closest_layer_id);    
    Rotation closest_rotation(layer_ptr->constraint().restriction());
    if (rotation == closest_rotation)
    {
        //no motion to do
        return ref_p;
    }
    
    if (layer_ptr->is_free(location) == false)
      return Reference_point();

    //(3) construct point predicate toward that layer
    C_space_line::Constraint constraint(location);
    C_space_line* line_ptr = new C_space_line (constraint, _ak);
    line_ptr->decompose(_robot, _decomposed_workspace);

    //(4) find fsc containing source point and target point
    int source_fsc_id = line_ptr->free_space_location_hint(ref_p);
    int target_fsc_id = line_ptr->free_space_location_hint(Reference_point(location, closest_rotation));
    if (source_fsc_id != target_fsc_id)
      return Reference_point();

    //(5) plan path on line
    std::vector<K::FT> tau_path;
    plan_path_in_interval<K>( FixedPoint::get_parametrization_theta<K>(ref_p.get_rotation()),
                              FixedPoint::get_parametrization_theta<K>(closest_rotation),
                              line_ptr->get_fsc(source_fsc_id).cell(),
                              std::back_inserter(tau_path));
    CGAL_postcondition(tau_path.size() == 3); //source, target, mid
    
  
    //(6) convert point path to Motion_step_rotational
    CGAL::Orientation orientation = get_orientation(tau_path[0], tau_path[1], tau_path[2]);  
    Motion_step_rotational* motion_step_ptr = new Motion_step_rotational( location, 
                                                                          ref_p.get_rotation(), closest_rotation, 
                                                                          orientation);
    motion_sequence.add_motion_step(motion_step_ptr);
    
    delete line_ptr;
    return Reference_point(location, closest_rotation);
  }
private: //Fsc_indx related methods
  Fsc_indx get_containig_fsc(const Reference_point& ref_p)
  {
    int manifold_id = _layers.get_containing_manifold_id(ref_p.get_rotation());
    if (manifold_id == NO_ID)
      return Fsc_indx();

    int fsc_id = _layers.get_manifold(manifold_id)->get_fsc_id(ref_p);

    if (fsc_id == NO_ID)
      return Fsc_indx();

    return Fsc_indx(FIXED_ANGLE,      //_type
                    manifold_id,      //_manifold_id
                    fsc_id            //_fsc_id;
                    );
  }
  Fsc* get_fsc(const Fsc_indx& fsc_indx)
  {
    Fsc* fsc_ptr;
    if (fsc_indx._type == FIXED_ANGLE)
    {
      Layer* layer_ptr = _layers.get_manifold(fsc_indx._manifold_id);
      fsc_ptr = new Fsc(  layer_ptr->get_fsc(fsc_indx._fsc_id),
                          layer_ptr->constraint());
    }
    else if (fsc_indx._type == FIXED_POINT)
    {
      C_space_line* c_space_line_ptr = _lines.get_manifold(fsc_indx._manifold_id);
      fsc_ptr = new Fsc(  c_space_line_ptr->get_fsc(fsc_indx._fsc_id),
                          c_space_line_ptr->constraint());
    }
    return fsc_ptr;
  }

  Reference_point  get_intersection(const Fsc_indx& fsc_indx_1, const Fsc_indx& fsc_indx_2)
  {
    CGAL_precondition(  ((fsc_indx_1._type == FIXED_ANGLE) && (fsc_indx_2._type == FIXED_POINT)) || 
                        ((fsc_indx_1._type == FIXED_POINT) && (fsc_indx_2._type == FIXED_ANGLE)) );

    Rotation r;
    Point p;
    if (fsc_indx_1._type == FIXED_ANGLE)
    {
      r = _layers.get_manifold(fsc_indx_1._manifold_id)->constraint().restriction();
      p = _lines .get_manifold(fsc_indx_2._manifold_id)->constraint().restriction();
    }
    else //(fsc_indx_2._type == FIXED_ANGLE)
    {
      r = _layers.get_manifold(fsc_indx_2._manifold_id)->constraint().restriction();
      p = _lines .get_manifold(fsc_indx_1._manifold_id)->constraint().restriction();
    }

    return Reference_point(p,r);
  }
private:
  void compute_workspace_bbox()
  {
    _workspace_bbox = _workspace[0].bbox();
    for (unsigned int i(1); i < _workspace.size(); ++i)
      _workspace_bbox = _workspace_bbox + _workspace[i].bbox();
    return;
  }
  void decompose_workspace_into_convex_polygons()
  {
    BOOST_FOREACH(Polygon polygon, _workspace)
      decompose_into_convex_polygons(polygon, std::back_inserter(_decomposed_workspace) );
  }
  template <typename OutputIterator>
  void decompose_into_convex_polygons(const Polygon& polygon, OutputIterator& oi)
  {
    CGAL::Small_side_angle_bisector_decomposition_2<K> decomp;
    decomp( polygon, oi);
    return;
  }

};  //Mms_path_planner_example

} //mms
#endif //MMS_EXAMPLE_H