#ifndef MY_PLANNER_H
#define MY_PLANNER_H

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
#include "Path_planning\ShortestPathInPolygon.h"
#include <CGAL/Boolean_set_operations_2.h>

typedef Environment<> Env;
typedef Env::Reference_point              Ref_p;
typedef Env::Reference_point_vec          Ref_p_vec;

enum VALIDATION {
	OK,				// Motion step is valid
	PATH_BLOCKED,	// Path is blocked but destination is otherwise reachable
	DST_BLOCKED,	// Destination is unreachable at the moment
};

namespace mms{

	template <typename K_ = Rational_kernel, 
		typename AK_ = CGAL::Algebraic_kernel_d_1 <typename CGAL::CORE_arithmetic_kernel::Integer>, 
		typename AK_conversions = Algebraic_kernel_d_1_conversions_rational<typename CGAL::CORE_arithmetic_kernel> >
	class MyPlanner
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
		typedef typename K::Segment_2				    Segment;

		typedef std::vector<typename Polygon>           Polygon_vec;
		typedef CGAL::Polygon_set_2<K>                  Polygon_set;

		typedef Motion_step_base<K>						MS_base;
		typedef Motion_step_rotational<K>               MS_rotational;
		typedef Motion_step_translational<K>            MS_translational;
		typedef Motion_sequence<K>                      Motion_sequence;

		typedef Fsc_indx<K>                             Fsc_indx;
		typedef std::vector<typename Fsc_indx>			Fsc_indx_vec;
		typedef FSC<K, AK, AK_conversions>              Fsc;
		

		typedef Fixed_angle_manifold_container<K>       Layers;
		typedef typename Layers::Manifold               Layer;

		typedef Fixed_point_manifold_container<K, AK, AK_conversions> C_space_lines;
		typedef typename C_space_lines::Manifold                      C_space_line;

		typedef Graph<Fsc_indx, Less_than_fsc_indx<K> > Connectivity_graph;
		typedef Random_utils<K>                         Random_utils;

	private:
		Polygon_vec&             _workspace;
		Polygon_vec              _decomposed_workspace;
		CGAL::Bbox_2             _workspace_bbox;
		Extended_polygon&        _robot;

		Connectivity_graph      _graph;

		std::vector<Rotation>   _rotations;
		Layers                  _layers;
		C_space_lines           _lines;

		Random_utils            _rand;
		AK                      _ak;

	public:
		//constructor
		MyPlanner  (Polygon_vec &workspace, Extended_polygon& robot)
			: _workspace (workspace), _robot(robot),
			_graph(0,true), _rand(time(NULL)) {
				compute_workspace_bbox();
				//the minkowski sum algorithm works faster on polygons are convex
				//hence we decompose the workspace obstacles to convex polygons
				decompose_workspace_into_convex_polygons();
		}

		//preprocess
		void preprocess(const unsigned int num_of_angles = configuration.get_slices_granularity()) {
			TIMED_TRACE_ENTER("preprocess");
			generate_random_rotations(num_of_angles);
			PRINT_ROTATIONS();
			BOOST_FOREACH (Rotation rotation, _rotations) {
				add_layer(rotation);
			}
			generate_random_connectors();    
			PRINT_CONNECTORS();
			TIMED_TRACE_EXIT("preprocess");
			return;
		}

		//do preprocess for additional points
		void preprocess_targets(Ref_p_vec& ref_points) {
			TIMED_TRACE_ENTER("preprocess_targets");

			generate_target_rotations(ref_points);
			PRINT_ROTATIONS();
			BOOST_FOREACH (Rotation rotation, _rotations) {
				add_layer(rotation);
			}
			generate_target_connectors(ref_points);
			PRINT_CONNECTIVITY_GRAPH();

			TIMED_TRACE_EXIT("preprocess_targets");
			return;
		}


		//query
		bool query( const Reference_point& source, const Reference_point& target,
			Motion_sequence& motion_sequence, double& motion_time, double motion_time_limit) 
		{

			TIMED_TRACE_ENTER("query");
			////////////////////////////////////
			//connect source and target to graph
			////////////////////////////////////
			Motion_sequence source_motion_sequence, target_motion_sequence;

			Reference_point perturbed_source = connect_to_graph(source, source_motion_sequence);
			if (perturbed_source == Reference_point()) {
				TIMED_TRACE_EXIT("query: failed to connect source to pre-processed configuration space");
				return false;
			}

			if (!source_motion_sequence.get_sequence().empty()) {
				motion_time += Motion_sequence::step_time(
					(Motion_sequence::MS_base_ptr)*source_motion_sequence.get_sequence().begin());
			}

			if (motion_time >= motion_time_limit) {
				TIMED_TRACE_EXIT("query: connecting source, not the closest point");
				return false;
			}

			Reference_point perturbed_target = connect_to_graph(target, target_motion_sequence);
			if (perturbed_target == Reference_point()) {
				TIMED_TRACE_EXIT("query: failed to connect target to pre-processed configuration space");
				return false;
			}
			if (!target_motion_sequence.get_sequence().empty()) {
				motion_time += Motion_sequence::step_time(
					(Motion_sequence::MS_base_ptr)*target_motion_sequence.get_sequence().begin());
			}

			if (motion_time >= motion_time_limit) {
				TIMED_TRACE_EXIT("query: connecting target, not the closest point");
				return false;
			}

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

			if (fsc_indx_path.empty()) {
				TIMED_TRACE_EXIT("query: fsc index path is empty");
				return false;
			}

			////////////////////////////////////
			//construct motion sequence
			////////////////////////////////////
			//(1) add source motion
			motion_sequence.add_motion_sequence(source_motion_sequence);

			//(2) construct real motion sequence from connectivity graph
			int             curr_fsc_indx = 0;
			Reference_point curr_ref_p    = perturbed_source;

			std::list<Fsc_indx>::iterator curr, next;


			//skip computing source step
			int time_index = 1;

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

				//TODO: there still seems to be a loop in plan_path__fixed_[angle/point]
				//However seems like heaviest part in plan_path is before. May consider do more delicate "return"
				//from this functions
				int current_size = motion_sequence.get_sequence().size();
				motion_time += motion_sequence.motion_time_between(time_index, current_size-1);
				if (motion_time >= motion_time_limit) {
					TIMED_TRACE_EXIT("query, main loop, not the closest point");
					return false;
				}

				time_index = current_size;

				CGAL_precondition ( (motion_sequence.get_sequence().empty()) || 
					(motion_sequence.get_sequence().back()->target() == next_ref_p) );

				curr++;
				next++;
				curr_ref_p = next_ref_p;
				delete fsc_ptr;
			}

			Fsc* fsc_ptr = get_fsc(*curr);
			plan_path(fsc_ptr, curr_ref_p, perturbed_target, motion_sequence);
			motion_time += motion_sequence.motion_time_between(time_index, motion_sequence.get_sequence().size()-1);
			if (motion_time >= motion_time_limit) {
				TIMED_TRACE_EXIT("query: connecting perturbed target, not the closest point");
				return false;
			}

			delete fsc_ptr;

			//(3) add target motion
			//time for this step is already computed!
			target_motion_sequence.reverse_motion_sequence();
			motion_sequence.add_motion_sequence(target_motion_sequence);
			TIMED_TRACE_EXIT("query");
			return true;
		}


		struct less_than_pair_double_int {
			bool operator()(const pair<double, int>& p1, const pair<double, int>& p2) const {
				return (p1.first < p2.first);
			}
		};

		//Returns closest point to target, according to motion_Sequence.motion_time.
		//Sets the motion_sequence to the point to motion_sequence.
		bool query_closest_point(
			const Reference_point& source, 
			Ref_p_vec& target_configurations,
			int& closest_target_index,
			Motion_sequence& motion_sequence) {

				TIMED_TRACE_ENTER("query_closest_point");

				std::vector<pair<double, int>> aerialTimeToIndex;

				for (unsigned int i=0; i<target_configurations.size(); i++) {
					double aerial_time = source.get_aerial_time(target_configurations[i]);
					aerialTimeToIndex.push_back(pair<double, int>(aerial_time, i));
				}

				less_than_pair_double_int less_than;
				std::sort(aerialTimeToIndex.begin(), aerialTimeToIndex.end(), less_than);

				Ref_p closest_point;
				bool path_found = false;
				Motion_sequence seq1, seq2;
				Motion_sequence *current = &seq1, *shortest = &seq2, *temp = NULL;
				double shortest_time = INFINITY, current_time, current_aerial_time = 0;

				cout << endl << "Time: " << global_tm.timer.time() << " SOURCE POINT: ";
				source.print();
				cout << endl;

				for (std::vector<pair<double, int>>::iterator it = aerialTimeToIndex.begin(); it!=aerialTimeToIndex.end(); it++) {
					current_time = 0;

					int point_index = it->second;

					double current_aerial_time = it->first;
					Reference_point target(target_configurations[point_index]);

					cout << endl << "Time: " << global_tm.timer.time() << " Query point: " << point_index << " ";
					target.print();
					cout << endl << "AERIAL TIME TO POINT: "<< current_aerial_time << endl;

					if (current_aerial_time > shortest_time) {
						//since vector is already sorted, all points from here are further. End of iterations.
						TIMED_TRACE_ACTION("query_closest_point", "aerial_time > shortest_time, finishing query");
						break;
					}

					if (!query(source, target, *current, current_time, shortest_time)) {
						continue;
					}
					path_found = true;

					std::cout << endl << "Found path to point, motion time: " << current_time << endl;

					if (current_time < shortest_time) {
						cout << "Best time till now: " << shortest_time << endl << endl;
						// Exchange current and shortest
						temp = current;
						current = shortest;
						shortest = temp;
						shortest_time = current_time;
						closest_target_index = point_index;
					} else {
						current->clear();
					}
				}

				motion_sequence.add_motion_sequence(*shortest);
				TIMED_TRACE_EXIT("query_closest_point");
				return path_found;
		}

		VALIDATION validate_step(MS_base& ms, Extended_polygon& robot, Extended_polygon& obstacle) {
			if (ms.type() == MS_base::TRANSLATION) {
				return validate_step(static_cast<MS_translational&>(ms), robot, obstacle);
			} else if (ms.type() == MS_base::ROTATION) {
				return validate_step(static_cast<MS_rotational&>(ms), robot, obstacle);
			} else {
				return OK;
			}
		}

		VALIDATION validate_step(MS_translational& ms, Extended_polygon& robot, Extended_polygon& obstacle) {
			TIMED_TRACE_ENTER("validate_step translational");
			// Find the layer that the step was plotted in
			Layer* layer = _layers.get_manifold(ms.target().get_rotation());

			// Make a copy of the original layer
			Layer updatedLayer = Layer(Fixed_angle_constraint<K>());
			updatedLayer.copy(*layer, false);

			// Add the dynamic obstacle to the layer copy
			updatedLayer.add_obstacle(robot, obstacle.get_absolute_polygon());

			// Check that the destination is reachable
			int target_fsc_id = updatedLayer.get_containing_cell(ms.target().get_location());
			if (target_fsc_id == NO_ID) {
				TIMED_TRACE_EXIT("validate_step: DST_BLOCKED");
				return DST_BLOCKED;
			}

			// Check that there is still a path between the source and the destination
			int source_fsc_id = updatedLayer.get_containing_cell(ms.source().get_location());
			if (source_fsc_id != target_fsc_id) {
				TIMED_TRACE_EXIT("validate_step: PATH_BLOCKED");
				return PATH_BLOCKED;
			}

			// Verify that target is still visible from source
			Smart_polygon& fsc_polygon = updatedLayer.get_fsc(target_fsc_id).cell();
			if (!is_in_polygon(Segment(ms.source().get_location(), ms.target().get_location()), fsc_polygon.polygon(), true)) {
				TIMED_TRACE_EXIT("validate_step: PATH_BLOCKED");
				return PATH_BLOCKED;
			}

			TIMED_TRACE_EXIT("validate_step: OK");
			return OK;
		}

		VALIDATION validate_step(MS_rotational& ms, Extended_polygon& robot, Extended_polygon& obstacle) {
			TIMED_TRACE_ENTER("validate_step rotational");
			// Find the original line that the step was taken from
			C_space_line* line = _lines.get_manifold(ms.target().get_location());

			// Make a copy of the original line
			C_space_line updatedLine = C_space_line(Fixed_point_constraint<K>(), _ak);
			updatedLine.copy(*line, false);

			// Add the dynamic obstacle to the layer copy
			updatedLine.add_obstacle(robot, obstacle.get_absolute_polygon());

			// Check that the destination is reachable
			int target_fsc_id = updatedLine.get_fsc_id(ms.target());
			if (target_fsc_id == NO_ID) {
				TIMED_TRACE_EXIT("validate_step: DST_BLOCKED");
				return DST_BLOCKED;
			}

			// Check that there is still a path between the source and the destination
			int source_fsc_id = updatedLine.get_fsc_id(ms.source());
			CGAL_precondition(source_fsc_id != NO_ID);
			if (source_fsc_id != target_fsc_id) {
				TIMED_TRACE_EXIT("validate_step: PATH_BLOCKED");
				return PATH_BLOCKED;
			}

			// Find correct rotation orientation
			C_space_line::Fsc& fsc = updatedLine.get_fsc(target_fsc_id);
			std::vector<typename K::FT> tau_path;
			plan_path_in_interval<K>(FixedPoint::get_parametrization_theta<K>(ms.source().get_rotation()),
				FixedPoint::get_parametrization_theta<K>(ms.target().get_rotation()),
				fsc.cell(),
				std::back_inserter(tau_path));
			CGAL::Orientation orientation = get_orientation(tau_path[0], tau_path[1], tau_path[2]);
			// Flip orientation if necessary
			if (ms.orientation() != orientation) {
				ms.reverse_orientation();
			}
			TIMED_TRACE_EXIT("validate_step: OK");
			return OK;
		}

		bool create_connecting_points(Ref_p_vec& connecting_points) {
			PRINT_CONNECTIVITY_GRAPH();
			std::vector<Fsc_indx_vec> ccs_vec = _graph.get_connected_components();
			Fsc_indx_vec fsc_indx_vec, fsc_indx_vec1, fsc_indx_vec2;
			Layer *layer_ptr1, *layer_ptr2;
			Point p;
			int fa_fsc_index = 1;
			int fa_fsc_count = 0;
  
			for (std::vector<Fsc_indx_vec>::iterator cc_iter = ccs_vec.begin() ; cc_iter != ccs_vec.end(); cc_iter++) {
				fsc_indx_vec = *cc_iter;
				for (Fsc_indx_vec::iterator fsc_iter = fsc_indx_vec.begin(); fsc_iter != fsc_indx_vec.end(); fsc_iter++) {
					if (fsc_iter->_type == FIXED_ANGLE) {
						fa_fsc_count++;
					}
				}
			}

			//go over all connectivity components
			for (std::vector<Fsc_indx_vec>::iterator cc_iter1 = ccs_vec.begin() ; cc_iter1 != ccs_vec.end(); cc_iter1++) {
				//iterate over all fsc within connectivity component
				fsc_indx_vec1 = *cc_iter1;
				for (Fsc_indx_vec::iterator fsc_iter1 = fsc_indx_vec1.begin(); fsc_iter1 != fsc_indx_vec1.end(); fsc_iter1++) {
					if (fsc_iter1->_type == FIXED_POINT) {
						continue;
					}
					int ccp_count = 0;
					cout << "fsc " << fa_fsc_index << " of " << fa_fsc_count << " ";
					print_fixed_angle_fsc(*fsc_iter1);		
					fa_fsc_index++;
					layer_ptr1 = _layers.get_manifold(fsc_iter1->_manifold_id);
					Polygon_with_holes& pgn1 = layer_ptr1->get_fsc(fsc_iter1->_fsc_id).cell().polygon();
					
					for (std::vector<Fsc_indx_vec>::iterator cc_iter2 = cc_iter1+1 ; cc_iter2 != ccs_vec.end(); cc_iter2++) {
						fsc_indx_vec2 = *cc_iter2;
						for (Fsc_indx_vec::iterator fsc_iter2 = fsc_indx_vec2.begin(); fsc_iter2 != fsc_indx_vec2.end(); fsc_iter2++) {
							if (fsc_iter2->_type == FIXED_POINT) {
								continue;
							}	

							//don't try to connect components from same fixed_angle_manifold
							if (fsc_iter1->_manifold_id == fsc_iter2->_manifold_id) {
								continue;
							}
							
							layer_ptr2 = _layers.get_manifold(fsc_iter2->_manifold_id);
							Polygon_with_holes& pgn2 = layer_ptr2->get_fsc(fsc_iter2->_fsc_id).cell().polygon();

							//now try to create ref_point
							if (!CGAL::do_intersect(pgn1, pgn2)) {
								continue;
							}
							bool generated = _rand.generate_random_point_in_polygons(p, pgn1, pgn2, 1000);
							if (!generated) {
								cout << "Failed generating point, skipping connection" << endl;
								continue;
							}
							Ref_p generated_point(p, layer_ptr1->constraint().restriction());
							connecting_points.push_back(generated_point);
							ccp_count++;
						}
					}
					if (ccp_count != 0) {
						cout << " created " << ccp_count << " points";
					}
					cout << endl;
				}
			}
			return false;
		}

		void additional_preprocessing(Ref_p& source, Ref_p_vec& targets) {
			TIMED_TRACE_ENTER("additional_reprocessing");

			Ref_p_vec connecting_points;
			create_connecting_points(connecting_points);
			generate_target_connectors(connecting_points);
			cout << "GENERATING " << connecting_points.size() << "CONNECTORS" << endl;
			TIMED_TRACE_EXIT("additional_preprocessing: graph is not connected");
		}

	///////////////////////////////////////////////////////////////////////////////////////////////////

	private: //layer methods

		void generate_random_rotations(const unsigned int num_of_angles) {
			_rotations.clear();
			double step ((double)360/num_of_angles);
			Rotation initial_rotation(_rand.get_random_rotation());
			for (double r(0); r<360; r+=step) {
				_rotations.push_back(initial_rotation * to_rotation<K::FT>(r,DEG,FINE));
			}
			uniquify_rotations();
		}

		void generate_target_rotations(Ref_p_vec& ref_points) {
			_rotations.clear();
			for (Ref_p_vec::iterator iter = ref_points.begin(); iter != ref_points.end(); iter++){
				_rotations.push_back((*iter).get_rotation());
			}
			uniquify_rotations();
		}

		void uniquify_rotations() {
			//maybe some rotations are the same as approximation is not good enough, remove duplicates 
			Less_than_rotation<K::FT> less_than;
			std::sort(_rotations.begin(), _rotations.end(), less_than);
			std::vector<Rotation>::iterator unique_end = std::unique(_rotations.begin(), _rotations.end());
			_rotations.erase(unique_end, _rotations.end());

			//maybe last rotation is zero like the first rotation
			if (_rotations.size() > 1 && _rotations.back() == _rotations.front()) {
				_rotations.pop_back();
			}
		}

		void add_layer(const Rotation& rotation)
		{
			//if layere already exists, return
			if (_layers.get_containing_manifold_id(rotation) != NO_ID) {
				return;
			}

			//create layer
			Layer* layer_ptr = new Layer (Layer::Constraint(rotation));
			layer_ptr->decompose(_robot, _decomposed_workspace);
			int layer_id = _layers.add_manifold(layer_ptr);

			update_connectivity_graph_vertices(*layer_ptr, layer_id);
			return;
		}

	private:

		void generate_random_connectors() {
			TIMED_TRACE_ENTER("generate_random_connectors");
			for (int i(0); i < configuration.get_max_num_of_intra_connections(); ++i) {
				generate_connector();
			}
			TIMED_TRACE_EXIT("generate_random_connectors");
		}

		void generate_target_connectors(Ref_p_vec& ref_points) {
			for (Ref_p_vec::iterator iter = ref_points.begin(); iter != ref_points.end(); iter++){
				generate_connector(&(*iter));
			}
		}

		void generate_connector(Ref_p* ref_point = NULL) {
			Rotation r;
			Point p;
			Layer* layer_ptr;

			if (ref_point != NULL) {
				p = ref_point->get_location();
				r = ref_point->get_rotation();
				int layer_id = _layers.get_containing_manifold_id(r);
				CGAL_precondition(layer_id != NO_ID);
				layer_ptr = _layers.get_manifold(layer_id);

			} else {
				// Generate a free point in the configuration space on one of the layers
				r  = _rotations[_rand.get_random_int(0, _rotations.size()-1)]; //random layer
				int layer_id = _layers.get_containing_manifold_id(r);
				layer_ptr = _layers.get_manifold(layer_id);
				do {
					p = _rand.get_random_point_in_bbox(_workspace_bbox);
				} while (layer_ptr->is_free(p) == false);
			}

			int cell_id  = layer_ptr->get_containing_cell(p); 
			if (cell_id == NO_ID) {
				return; // this should NOT be NO_ID but there is a patch in the configuration space of the angle primitive due to a bug in polygon_set_2
			}

			C_space_line* c_space_line_ptr; 
			C_space_line::Constraint constraint;

			// Choose roi
			double cell_size_ratio = get_size_percentage(layer_ptr->get_fsc(cell_id).cell() );
			CGAL_precondition (cell_size_ratio >=0 && cell_size_ratio <=1);

			if (configuration.get_use_region_of_interest() && cell_size_ratio < 1 ) {
				double small_rotation(configuration.get_rotation_range()/2);
				CGAL_precondition(small_rotation < 180);
				double half_range_double = small_rotation + cell_size_ratio * (180 - small_rotation);
				Rotation half_range = to_rotation<K::FT>(half_range_double, DEG);
				Rotation_range range(r * (-half_range), r * half_range);
				constraint = C_space_line::Constraint(p, range);

			} else {
				constraint = C_space_line::Constraint(p);
			}

			// Filter out random points that don't contribute to connectivity
			if (ref_point == NULL && filter_out(constraint)) {
				return;
			}

			// Create connector
			c_space_line_ptr = new C_space_line (constraint, _ak);
			c_space_line_ptr->decompose(_robot, _decomposed_workspace);
			int c_space_line_id = _lines.add_manifold(c_space_line_ptr);

			// Update connectivity graph
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

	private: //Connectivity_graph methods
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
			TIMED_TRACE_ENTER("connect_to_graph");
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
				TIMED_TRACE_EXIT("connect_to_graph: no motion to do");
				return ref_p;
			}

			if (layer_ptr->is_free(location) == false) {
				TIMED_TRACE_EXIT("connect_to_graph: cannot connect layer");
				return Reference_point();
			}

			//(3) construct point predicate toward that layer
			C_space_line::Constraint constraint(location);
			C_space_line* line_ptr = new C_space_line (constraint, _ak);
			line_ptr->decompose(_robot, _decomposed_workspace);

			//(4) find fsc containing source point and target point
			int source_fsc_id = line_ptr->free_space_location_hint(ref_p);
			int target_fsc_id = line_ptr->free_space_location_hint(Reference_point(location, closest_rotation));
			if (source_fsc_id != target_fsc_id) {
				TIMED_TRACE_EXIT("connect_to_graph: no fsc contains source and target");
				return Reference_point();
			}

			//(5) plan path on line
			std::vector<K::FT> tau_path;
			plan_path_in_interval<K>( FixedPoint::get_parametrization_theta<K>(ref_p.get_rotation()),
				FixedPoint::get_parametrization_theta<K>(closest_rotation),
				line_ptr->get_fsc(source_fsc_id).cell(),
				std::back_inserter(tau_path));
			CGAL_postcondition(tau_path.size() == 3); //source, target, mid


			//(6) convert point path to Motion_step_rotational
			CGAL::Orientation orientation = get_orientation(tau_path[0], tau_path[1], tau_path[2]);  
			MS_rotational* motion_step_ptr = new MS_rotational( location, 
				ref_p.get_rotation(), closest_rotation, 
				orientation);
			motion_sequence.add_motion_step(motion_step_ptr);

			delete line_ptr;
			TIMED_TRACE_EXIT("connect_to_graph");
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

		//debugging methods
		void print_rotations() {
			cout << "Generated " << _rotations.size() << " rotations:" << endl;
			BOOST_FOREACH(Rotation rotation, _rotations) {
				cout << rotation.to_angle() << ", ";
			}
			cout << endl;
		}

		void print_connectors() {
			cout << "Generated " << _lines.manifold_id_iterator_end() << " connectors:" << endl; 
			for (int i = _lines.manifold_id_iterator_begin(); i < _lines.manifold_id_iterator_end(); i++) {
				print_point_nice<K>(_lines.get_manifold(i)->constraint().restriction());
				cout << " ";
			}
			cout << endl;
		}

		void print_connectivity_graph() {
			//_graph.print();
			_graph.print_connected_components();
		}

		void print_fixed_angle_fsc(Fsc_indx& fsc_indx) {
			Layer* layer_ptr = _layers.get_manifold(fsc_indx._manifold_id);
			Polygon_with_holes& pgn = layer_ptr->get_fsc(fsc_indx._fsc_id).cell().polygon();
			cout << "manifold id: " << fsc_indx._manifold_id << " fsc id: " << fsc_indx._fsc_id
				<< " angle = " << layer_ptr->constraint().restriction().to_angle();
			//print_my_polygon_with_holes(pgn, 10);
		}

		void print_workspace() {
			cout << "WORKSPACE:" << endl;
			print_polygon_vector(_workspace);
			cout << "DECOMPOSED WORKSPACE:" << endl;
			print_polygon_vector(_decomposed_workspace);
		}

		void print_my_polygon(Polygon polygon, int vertex_limit) {
			cout <<"Polygon vertices: " << polygon.size() << endl;
			int vertex_index = 0;
			for (Polygon::Vertex_const_iterator vi = polygon.vertices_begin(); vi!= polygon.vertices_end (); ++vi) {
				if (vertex_index = vertex_limit) {
					cout << (polygon.size() - vertex_limit) << " more..." << endl;
					break;
				}
				std::cout << "(" << CGAL::to_double(vi->x()) << "  ,  " << CGAL::to_double(vi->y()) << ")" << std::endl; 
				vertex_index++;
			}
		}

		void print_my_polygon_with_holes(Polygon_with_holes polygon, int vertex_limit) {
			cout << "********************" << endl;
			cout <<"Polygon outer boundary: " << endl;
			print_my_polygon(polygon.outer_boundary(), vertex_limit);
			cout <<"Polygon holes: " << endl;
			for (Polygon_with_holes::Hole_const_iterator hi = polygon.holes_begin(); hi!= polygon.holes_end(); ++hi) {
				print_my_polygon((*hi), vertex_limit);
			}
			cout << "********************" << endl;
		}
	}; 


} //mms
#endif //MY_PLANNER_H
