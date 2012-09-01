#ifndef MY_PLANNER_H
#define MY_PLANNER_H

//#define DEBUG_PLANNER

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

		// TODO: there is much bardak here, fix this
		std::vector<Rotation>   _rotations;
		Layers                  _layers;
		C_space_lines           _lines;

		Random_utils            _rand;
		AK                      _ak;
		//int						_layers_level; //0,1,2,3
		//bool					_added_res_2_connectors;
		int						_state;

	public:
		//constructor
		MyPlanner  (Polygon_vec &workspace, Extended_polygon& robot)
			: _workspace (workspace), _robot(robot),
			_graph(0,true), _rand(configuration.get_seed()), _state(0) {
				compute_workspace_bbox();
				//the minkowski sum algorithm works faster on polygons are convex
				//hence we decompose the workspace obstacles to convex polygons
				decompose_workspace_into_convex_polygons(_workspace);
		}

		// Copy planner with an additional obstacle
		MyPlanner(MyPlanner& other, Extended_polygon& obstacle) :
			_workspace(other._workspace), _robot(other._robot),
			_workspace_bbox(other._workspace_bbox),
			_graph(0, true), _rand(configuration.get_seed()), _state(other._state) {

				TIMED_TRACE_ENTER("MyPlanner: copying");
				Polygon_vec updated_workspace(_workspace);
				updated_workspace.push_back(obstacle.get_absolute_polygon());
				decompose_workspace_into_convex_polygons(updated_workspace);

				// Copy all layers with the extra obstacle
				Layers& other_layers = other._layers;
				for (int id = other_layers.manifold_id_iterator_begin(); id != other_layers.manifold_id_iterator_end(); id = other_layers.manifold_id_iterator_next()) {
					// Memory leak here (but all of the planner code is already leaking like crazy so I'm not fixing this)
					Layer* new_layer = new Layer(Fixed_angle_constraint<K>());
					copy_layer(*(other_layers.get_manifold(id)), *new_layer, _robot, obstacle);
					int new_id = _layers.add_manifold(new_layer);
					update_connectivity_graph_layer(new_id);
				}

				// Copy all lines with the extra obstacle
				C_space_lines& other_lines = other._lines;
				for (int id = other_lines.manifold_id_iterator_begin(); id != other_lines.manifold_id_iterator_end(); id = other_lines.manifold_id_iterator_next()) {
					// Memory leak here (but all of the planner code is already leaking like crazy so I'm not fixing this)
					C_space_line* new_line = new C_space_line(Fixed_point_constraint<K>(), _ak);
					copy_line(*(other_lines.get_manifold(id)), *new_line, _robot, obstacle);
					int new_id = _lines.add_manifold(new_line);
					update_connectivity_graph_line(new_id);
				}
				TIMED_TRACE_EXIT("MyPlanner: copying");
		}

		void initialize(Ref_p& source, Ref_p_vec& targets) {
			TIMED_TRACE_ENTER("initialize");
			ASSERT_CONDITION((_state == 0),"Initialize called with state != 0");

			/* enter state 1 */
			_state = 1;
			preprocess_generate_layers(configuration.get_layers_res_1());
			Ref_p_vec all_conf = targets;
			all_conf.push_back(source);
			preprocess_targets(all_conf);
			preprocess_generate_connectors(2, false, false);
			TIMED_TRACE_EXIT("initialize: 0->1");
		}

	/*	will be called when there is time to do static planning. Returns whether it should be called again if time permits.
		If state < 3, increase state, perform action at new state and return true (call me again if time permits).
		If state == 3, perform state 3 action again and return false.
		If state == 4, go to state 5 performing action 5 and return true.
		If state == 5, redo state 5 action and return false. */
		bool preprocess_plan() {
			TIMED_TRACE_ENTER("preprocess_plan");

			switch(_state) {
			case 1:
				/* enter state 2 */
				_state++;
				preprocess_generate_layers(configuration.get_layers_res_2());
				//TODO: connect existing layers!, and also check what ROI matters
				preprocess_generate_connectors(2, false, true);
				TIMED_TRACE_EXIT("preprocess_plan: 1->2");
				return true;
			
			case 2:
				/* enter state 3 */
				_state++;
				preprocess_generate_connectors(5, true, true);
				TIMED_TRACE_EXIT("preprocess_plan: 2->3");
				return true;

			case 3:
				/* keep state 3 */
				preprocess_generate_connectors(5, true, true);
				TIMED_TRACE_EXIT("preprocess_plan: 3->3");
				return false;

			case 4:
				/* enter state 5 */
				_state++;
				preprocess_generate_connectors(1, true, true);
				TIMED_TRACE_EXIT("preprocess_plan: 4->5");
				return true;

			case 5:
				/* keep state 5 */
				preprocess_generate_connectors(1, true, true);
				TIMED_TRACE_EXIT("preprocess_plan: 5->5");
				return false;

			default:
				ASSERT_CONDITION(false, "preprocess plan: invalid state");
				TIMED_TRACE_EXIT("preprocess_plan: ?->?");
				return false;
			}
		}

	/*	will be called when a move needs to be made but the planner cannot satisfy any requests. 
		Guaranteed to be called only if there are no reachable targets.
		Always takes us from state n to state n+1 with the exception of 5.*/
		bool preprocess_move() {
			switch(_state) {
			case 1:
				/* enter state 2 */
				_state++;
				preprocess_generate_layers(configuration.get_layers_res_2());
				//TODO: connect existing layers!, and also check what ROI matters
				preprocess_generate_connectors(2, false, true);
				TIMED_TRACE_EXIT("preprocess_move: 1->2");
				return true;
			
			case 2:
				/* enter state 3 */
				_state++;
				preprocess_generate_connectors(5, true, true);
				TIMED_TRACE_EXIT("preprocess_move: 2->3");
				return true;

			case 3:
				/* enter state 4 */
				_state++;
				preprocess_generate_connectors(1, true, true);
				TIMED_TRACE_EXIT("preprocess_move: 3->4");
				return true;

			case 4:
				_state++;
				preprocess_generate_layers(configuration.get_layers_res_3());
				preprocess_generate_connectors(1, true, true);
				TIMED_TRACE_EXIT("preprocess_move: 4->5");
				return true;

			case 5:
				preprocess_generate_connectors(1, true, true);
				TIMED_TRACE_EXIT("preprocess_move: 5->5");
				return true;

			default:
				ASSERT_CONDITION(false, "preprocess plan: invalid state");
				TIMED_TRACE_EXIT("preprocess_plan: ?->?");
				return false;
			}
		}

		void preprocess_generate_layers(int layers_res) {
			TIMED_TRACE_ENTER("preprocess_generate_layers");
			generate_random_rotations(layers_res);
			PRINT_ROTATIONS();
			BOOST_FOREACH (Rotation rotation, _rotations) {
				add_layer(rotation);
			}
			TIMED_TRACE_EXIT("preprocess_generate_layers");
		}
		
		void preprocess_generate_connectors(int cc_limit, bool use_filter, bool use_roi) {
			TIMED_TRACE_ENTER("preprocess_generate_connectors");
			Ref_p_vec connecting_points;
			create_connecting_points(connecting_points, cc_limit);
			int generated_connectors = generate_target_connectors(use_filter, use_roi, connecting_points);
			PLAYBACK_PRINT_FIXED_ANGLE_FSCS();
			TIMED_TRACE_EXIT("preprocess_generate_connectors");
		}

		//old version, kept for backward compatibility
		void preprocess(const unsigned int num_of_angles = configuration.get_slices_granularity()) {
			TIMED_TRACE_ENTER("preprocess");
			generate_random_rotations(num_of_angles);
			PRINT_ROTATIONS();
			BOOST_FOREACH (Rotation rotation, _rotations) {
				add_layer(rotation);
			}

			Ref_p_vec connecting_points;
			create_connecting_points(connecting_points, 2);
			int generated_connectors = generate_target_connectors(false, false, connecting_points);
			PRINT_GENERATED_CONNECTORS(generated_connectors);
			PRINT_CONNECTORS();
			PLAYBACK_PRINT_CONNECTORS();
			PLAYBACK_PRINT_FIXED_ANGLE_FSCS();

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
			generate_target_connectors(false, false, ref_points);
			PRINT_CONNECTIVITY_GRAPH();

			TIMED_TRACE_EXIT("preprocess_targets");
			return;
		}

		void additional_preprocessing(Ref_p& source, Ref_p_vec& targets) {
			TIMED_TRACE_ENTER("additional_reprocessing");

			Ref_p_vec connecting_points;
			create_connecting_points(connecting_points, 10);
			PRINT_TRY_TO_GENERATE_CONNECTORS(connecting_points.size());
			int connectors_count = generate_target_connectors(true, true, connecting_points);
			PRINT_GENERATED_CONNECTORS(connectors_count);
			PLAYBACK_PRINT_CONNECTORS();
			TIMED_TRACE_EXIT("additional_preprocessing");
		}

		bool exist_reachable_target(const Ref_p& source, const Ref_p_vec& targets) {
			TIMED_TRACE_ENTER("exist_reachable_target");
			Fsc_indx source_fsc_indx (get_containig_fsc(source));
			Fsc_indx target_fsc_indx;

			if (source_fsc_indx == Fsc_indx()) {
				TIMED_TRACE_EXIT("exist_reachable_target: source is not connected to graph!");
				return false;
			}
			
			BOOST_FOREACH(Ref_p target, targets) {
				target_fsc_indx = get_containig_fsc(target);
				if (target_fsc_indx == Fsc_indx()) {
					// Target is not free
					continue;
				}
				if (_graph.is_in_same_cc(source_fsc_indx, target_fsc_indx)) {
					TIMED_TRACE_EXIT("exist_reachable_target: true");
					return true;
				}
			}
			TIMED_TRACE_EXIT("exist_reachable_target: false");
			return false;
		}
		
		bool exist_unreachable_target(const Ref_p& source, const Ref_p_vec& targets) {
			TIMED_TRACE_ENTER("exist_reachable_target");
			Fsc_indx source_fsc_indx (get_containig_fsc(source));
			Fsc_indx target_fsc_indx;

			if (source_fsc_indx != Fsc_indx()) {
				TIMED_TRACE_EXIT("exist_UNreachable_target: source is not connected to graph!(true)");
				return true;
			}
			
			BOOST_FOREACH(Ref_p target, targets) {
				target_fsc_indx = get_containig_fsc(target);
				if (target_fsc_indx == Fsc_indx()) {
					TIMED_TRACE_EXIT("exist_UNreachable_target: target is not connected to graph!(true)");
					return true;
				}
				if (!_graph.is_in_same_cc(source_fsc_indx, target_fsc_indx)) {
					TIMED_TRACE_EXIT("exist_UNreachable_target: true");
					return true;
				}
			}
			TIMED_TRACE_EXIT("exist_UNreachable_target: false");
			return false;
		}

		//query
		bool query(const Reference_point& source, const Reference_point& target,
			Motion_sequence& motion_sequence, double& motion_time, double motion_time_limit) {

			TIMED_TRACE_ENTER("query");
			
			//connect source and target to graph
			Motion_sequence source_motion_sequence, target_motion_sequence;

			//compute source->perturbed_source motion sequence + time
			Reference_point perturbed_source = connect_to_graph(source, source_motion_sequence);
			if (perturbed_source == Reference_point()) {
				TIMED_TRACE_EXIT("query: failed to connect source to pre-processed configuration space");
				return false;
			}
			motion_time += source_motion_sequence.motion_time();
			if (motion_time >= motion_time_limit) {
				TIMED_TRACE_EXIT("query: connecting perturbed source, not the closest point");
				return false;
			}

			//compute perturbed_target->target motion sequence + time
			Reference_point perturbed_target = connect_to_graph(target, target_motion_sequence);
			if (perturbed_target == Reference_point()) {
				TIMED_TRACE_EXIT("query: failed to connect target to pre-processed configuration space");
				return false;
			}
			motion_time += target_motion_sequence.motion_time();
			if (motion_time >= motion_time_limit) {
				TIMED_TRACE_EXIT("query: connecting perturbed target, not the closest point");
				return false;
			}

			//find path of fscs(if exists)
			Fsc_indx source_fsc_indx (get_containig_fsc(perturbed_source));
			CGAL_postcondition (source_fsc_indx != Fsc_indx());
			Fsc_indx target_fsc_indx (get_containig_fsc(perturbed_target));
			CGAL_postcondition (target_fsc_indx != Fsc_indx());

			std::list<Fsc_indx> fsc_indx_path;
			if (source_fsc_indx == target_fsc_indx) {
				fsc_indx_path.push_back(source_fsc_indx);
			} else {
				_graph.find_path(source_fsc_indx, target_fsc_indx, fsc_indx_path);
			}
			if (fsc_indx_path.empty()) {
				TIMED_TRACE_EXIT("query: fsc index path is empty");
				return false;
			}

			//construct motion sequence
			//(1) add source motion
			motion_sequence.add_motion_sequence(source_motion_sequence);

			//(2) construct real motion sequence from connectivity graph
			int curr_fsc_indx = 0;
			Reference_point curr_ref_p = perturbed_source;

			std::list<Fsc_indx>::iterator curr, next;

			int time_index = motion_sequence.get_sequence().size();

			next = curr = fsc_indx_path.begin();
			++next;
			while (next != fsc_indx_path.end()) {
				Reference_point  next_ref_p = get_intersection(*curr, *next);

				Fsc* fsc_ptr = get_fsc(*curr);
				CGAL_postcondition(fsc_ptr->contains(curr_ref_p));
				CGAL_postcondition(fsc_ptr->contains(next_ref_p));
				CGAL_precondition ( (motion_sequence.get_sequence().empty()) || 
					(motion_sequence.get_sequence().back()->target() == curr_ref_p) );
				plan_path(fsc_ptr, curr_ref_p, next_ref_p, motion_sequence);

				int current_size = motion_sequence.get_sequence().size();
				motion_time += motion_sequence.motion_time_between(time_index, current_size-1);
				//stop planning path as soon as we know that target is not the closest
				if (motion_time >= motion_time_limit) {
					delete fsc_ptr;
					TIMED_TRACE_EXIT("query: connecting fscs, not the closest point");
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

			//(3) add target motion, this sequence time was already added to motion time
			target_motion_sequence.reverse_motion_sequence();
			motion_sequence.add_motion_sequence(target_motion_sequence);

			ASSERT_CONDITION(CGAL::abs(motion_time - motion_sequence.motion_time()) < 0.001, 
				"query: motion_time composed = " << motion_time 
				<< " motion_time total = " << motion_sequence.motion_time());

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
				PRINT_SOURCE_POINT();
				for (std::vector<pair<double, int>>::iterator it = aerialTimeToIndex.begin(); it!=aerialTimeToIndex.end(); it++) {
					current_time = 0;

					int point_index = it->second;

					double current_aerial_time = it->first;
					Reference_point target(target_configurations[point_index]);
					PRINT_TARGET_POINT_AERIAL_TIME();

					if (current_aerial_time > shortest_time) {
						//since vector is already sorted, all points from here are further. End of iterations.
						TIMED_TRACE_ACTION("query_closest_point", "aerial_time > shortest_time, no more queries needed");
						break;
					}

					if (!query(source, target, *current, current_time, shortest_time)) {
						current->clear();
						continue;
					}
					path_found = true;
					PRINT_BEST_MOTION_TIME();

					if (current_time < shortest_time) {	
						// Exchange current and shortest
						temp = current;
						current = shortest;
						shortest = temp;
						shortest_time = current_time;
						closest_target_index = point_index;
					}
					current->clear();
				}

				motion_sequence.add_motion_sequence(*shortest);
				PRINT_SELECTED_TARGET();
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

		void copy_layer(Layer& oldLayer, Layer& newLayer, Extended_polygon& robot, Extended_polygon& obstacle) {
			newLayer.copy(oldLayer, false);
			// Add the dynamic obstacle to the layer copy
			newLayer.add_obstacle(robot, obstacle.get_absolute_polygon());
		}

		VALIDATION validate_step(MS_translational& ms, Extended_polygon& robot, Extended_polygon& obstacle) {
			TIMED_TRACE_ENTER("validate_step translational");
			// Find the layer that the step was plotted in
			Layer* layer = _layers.get_manifold(ms.target().get_rotation());

			// Make a copy of the original layer
			Layer updatedLayer = Layer(Fixed_angle_constraint<K>());
			copy_layer(*layer, updatedLayer, robot, obstacle);

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

		void copy_line(C_space_line& oldLine, C_space_line& newLine, Extended_polygon& robot, Extended_polygon& obstacle) {
			newLine.copy(oldLine, false);
			// Add the dynamic obstacle to the layer copy
			newLine.add_obstacle(robot, obstacle.get_absolute_polygon());
		}

		VALIDATION validate_step(MS_rotational& ms, Extended_polygon& robot, Extended_polygon& obstacle) {
			TIMED_TRACE_ENTER("validate_step rotational");
			// Find the original line that the step was taken from
			C_space_line* line = _lines.get_manifold(ms.target().get_location());

			// Make a copy of the original line
			C_space_line updatedLine = C_space_line(Fixed_point_constraint<K>(), _ak);
			copy_line(*line, updatedLine, robot, obstacle);

			// Check that the destination is reachable
			int target_fsc_id = updatedLine.free_space_location_hint(ms.target());
			if (target_fsc_id == NO_ID) {
				TIMED_TRACE_EXIT("validate_step: DST_BLOCKED");
				return DST_BLOCKED;
			}

			// Check that there is still a path between the source and the destination
			int source_fsc_id = updatedLine.free_space_location_hint(ms.source());
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

		int get_fixed_angle_fsc_count() {
			Fsc_indx_vec fsc_indx_vec;
			int fa_fsc_count = 0;
			std::vector<Fsc_indx_vec> ccs_vec = _graph.get_connected_components();
			for (std::vector<Fsc_indx_vec>::iterator cc_iter = ccs_vec.begin() ; cc_iter != ccs_vec.end(); cc_iter++) {
				fsc_indx_vec = *cc_iter;
				for (Fsc_indx_vec::iterator fsc_iter = fsc_indx_vec.begin(); fsc_iter != fsc_indx_vec.end(); fsc_iter++) {
					if (fsc_iter->_type == FIXED_ANGLE) {
						fa_fsc_count++;
					}
				}
			}
			return fa_fsc_count;
		}

		bool create_connecting_points(Ref_p_vec& connecting_points, int max_cc_to_cc_connections) {

			TIMED_TRACE_ENTER("create_connecting_points");
			PRINT_CONNECTIVITY_GRAPH();
			std::vector<Fsc_indx_vec> ccs_vec = _graph.get_connected_components();
			Fsc_indx_vec fsc_indx_vec1, fsc_indx_vec2;
			Layer *layer_ptr1, *layer_ptr2;
			Point p;
			int fa_fsc_count = get_fixed_angle_fsc_count();
			int fa_fsc_index = 1;
			int fa_fsc_to_fa_fsc_count = 0;

			//go over all connectivity components
			for (std::vector<Fsc_indx_vec>::iterator cc_iter1 = ccs_vec.begin() ; cc_iter1 != ccs_vec.end(); cc_iter1++) {
				//iterate over all fsc within connectivity component
				fsc_indx_vec1 = *cc_iter1;
				int cc_to_cc_count = 0;
				for (Fsc_indx_vec::iterator fsc_iter1 = fsc_indx_vec1.begin(); fsc_iter1 != fsc_indx_vec1.end(); fsc_iter1++) {
					if (fsc_iter1->_type == FIXED_POINT) {
						continue;
					}
					int ccp_count = 0;
					PRINT_FA_FSC_PREFIX();		
					fa_fsc_index++;
					layer_ptr1 = _layers.get_manifold(fsc_iter1->_manifold_id);
					Polygon_with_holes& pgn1 = layer_ptr1->get_fsc(fsc_iter1->_fsc_id).cell().polygon();
					if (pgn1.is_unbounded()) {
						PRINT_POLYGON_UNBOUNDED_SKIPPING();
						continue;
					}
					
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
							if (pgn2.is_unbounded()) {
								//PRINT_POLYGON_UNBOUNDED_SKIPPING();
								continue;
							}

							//now try to create ref_point
							if (!CGAL::do_intersect(pgn1, pgn2)) {
								continue;
							}
							bool generated = _rand.generate_random_point_in_polygons(p, pgn1, pgn2, 1000);
							if (!generated) {
								PRINF_FA_FSC_FAILED_GENERATING_POINT();
								continue;
							}
							Ref_p generated_point(p, layer_ptr1->constraint().restriction());
							connecting_points.push_back(generated_point);
							ccp_count++;
							cc_to_cc_count++;
							
							//create only up to parameter limit cross cc connectors
							if (cc_to_cc_count >= max_cc_to_cc_connections) {
								//Assume this CC is well connected to outer loop's cc.
								break;
							}
						}
						//TODO: checking this condition twice, looks like bad programming...
						if (cc_to_cc_count >= max_cc_to_cc_connections) {
							//Assume this CC is well connected to outer loop's cc.
							break;
						}
					}
					PRINT_FA_FSC_SUFFIX();
				}
			}
			TIMED_TRACE_EXIT("create_connecting_points");
			return false;
		}

	//layer methods
	int get_layers_level() {
		return _layers_level;
	}

	void set_layers_level(int layers_level) {
		_layers_level = layers_level;
	}


	///////////////////////////////////////////////////////////////////////////////////////////////////

	private: 

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

			update_connectivity_graph_layer(layer_id);
			return;
		}

	private:

		//returns number of connectors created
		int generate_random_connectors() {
			TIMED_TRACE_ENTER("generate_random_connectors");
			int generated = 0;
			for (int i(0); i < configuration.get_max_num_of_intra_connections(); ++i) {
				if (generate_connector(true, true)) {
					generated++;
				}
			}
			TIMED_TRACE_EXIT("generate_random_connectors");
			return generated;
		}

		//returns number of connectors created
		int generate_target_connectors(bool use_filter, bool use_roi, Ref_p_vec& ref_points) {
			TIMED_TRACE_ENTER("generate_target_connectors");
			int generated = 0;
			for (Ref_p_vec::iterator iter = ref_points.begin(); iter != ref_points.end(); iter++){
				if (generate_connector(use_filter, use_roi, &(*iter))) {
					generated++;
				}
			}
			PRINT_CONNECTIVITY_GRAPH();
			PLAYBACK_PRINT_CONNECTORS();
			TIMED_TRACE_EXIT("generate_target_connectors");
			return generated;
		}

		bool generate_connector(bool use_filter, bool use_roi, Ref_p* ref_point = NULL) {
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
				return false; // this should NOT be NO_ID but there is a patch in the configuration space of the angle primitive due to a bug in polygon_set_2
			}

			C_space_line* c_space_line_ptr; 
			C_space_line::Constraint constraint;

			// Choose roi
			double cell_size_ratio = get_size_percentage(layer_ptr->get_fsc(cell_id).cell() );
			CGAL_precondition (cell_size_ratio >=0 && cell_size_ratio <=1);

			if (use_roi && cell_size_ratio < 1 ) {
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
			if (use_filter && filter_out(constraint)) {
				return false;
			}

			// Create connector
			c_space_line_ptr = new C_space_line (constraint, _ak);
			c_space_line_ptr->decompose(_robot, _decomposed_workspace);

			int c_space_line_id = _lines.add_manifold(c_space_line_ptr);

			// Update connectivity graph
			update_connectivity_graph_line(c_space_line_id);
			return true;
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

		void update_connectivity_graph_layer(int layer_id) {
			CGAL_precondition (layer_id != NO_ID);
			Layer* layer_ptr = _layers.get_manifold(layer_id);

			//add all fixed poinr fscs
			for (int fsc_id(0); fsc_id<layer_ptr->num_of_fscs(); ++fsc_id) {
				Fsc_indx layer_fsc_indx(FIXED_ANGLE, layer_id, fsc_id);
				_graph.add_vertex(layer_fsc_indx);
			}

			//iterate over all lines, find intersections
			for (int iter = _lines.manifold_id_iterator_begin(); iter != _lines.manifold_id_iterator_end(); iter++) {
				Int_pair edges_ids;
				C_space_line* line_ptr = _lines.get_manifold(iter);
				//will address the correct cell, no need to iterate over all cells
				intersect<K> (*layer_ptr, *line_ptr, edges_ids);
				if ((edges_ids.first == NO_ID) || (edges_ids.second == NO_ID)) {
					continue;
				}

				Fsc_indx layer_fsc_indx(FIXED_ANGLE, layer_id, edges_ids.first);
				Fsc_indx line_fsc_indx (FIXED_POINT, iter, edges_ids.second);
				
				_graph.add_vertex(line_fsc_indx);
				_graph.add_edge(layer_fsc_indx, line_fsc_indx);

			}
			return;
		}

		void update_connectivity_graph_line(int c_space_line_id)
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
			if (rotation == closest_rotation) {
				//no motion to do
				return ref_p;
			}

			if (layer_ptr->is_free(location) == false) {
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
				TIMED_TRACE_EXIT("connect_to_graph: cannot connect source to a target");
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
		void decompose_workspace_into_convex_polygons(Polygon_vec& workspace)
		{
			BOOST_FOREACH(Polygon polygon, workspace)
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
			cout << "There exist " << _rotations.size() << " rotations: ";
			for (int iter = _layers.manifold_id_iterator_begin(); iter != _layers.manifold_id_iterator_end; iter++) {
				cout << _layers.get_manifold(iter)->constraint().restriction().to_angle();
			}
			cout << endl;
		}

		void print_connectors_as_configurations(){
			cout << "Printing " << _lines.manifold_id_iterator_end() << " connectors as configurations: " << endl;
			for (int i = _lines.manifold_id_iterator_begin(); i < _lines.manifold_id_iterator_end(); i++) {
				Point p = _lines.get_manifold(i)->constraint().restriction();
				double x = CGAL::to_double(p.x());
				double y = CGAL::to_double(p.y());
				cout << x << " " << y << " 0" << endl;
			}
		}

		void print_connectors() {
			print_connectors_count();
			for (int i = _lines.manifold_id_iterator_begin(); i < _lines.manifold_id_iterator_end(); i++) {
				print_point_nice<K>(_lines.get_manifold(i)->constraint().restriction());
				cout << " ";
			}
			cout << endl;
		}

		void print_connectors_count() {
			cout << "There exist " << _lines.manifold_id_iterator_end() << " connectors" << endl; 
		}

		void print_connectivity_graph_count() {
			cout << "Connectivity Graph: " 
				<< _graph.get_vertex_count() << " nodes, "
				<< _graph.get_edge_count() << " edges " 
				<< _graph.get_connected_components().size() << " connected components" << endl;
		}

		void print_connectivity_graph() {
			print_connectivity_graph_count();
			_graph.print_connected_components();
		}

		void print_fixed_angle_fscs(bool print_polygons) {
			Fsc_indx_vec fsc_indx_vec;
			std::vector<Fsc_indx_vec> ccs_vec = _graph.get_connected_components();
			for (std::vector<Fsc_indx_vec>::iterator cc_iter = ccs_vec.begin() ; cc_iter != ccs_vec.end(); cc_iter++) {
				fsc_indx_vec = *cc_iter;
				for (Fsc_indx_vec::iterator fsc_iter = fsc_indx_vec.begin(); fsc_iter != fsc_indx_vec.end(); fsc_iter++) {
					if (fsc_iter->_type == FIXED_ANGLE) {
						print_fixed_angle_fsc(*fsc_iter, print_polygons);
					}
				}
			}
		}

		void print_fixed_angle_fsc(Fsc_indx& fsc_indx, bool print_polygon) {
			Layer* layer_ptr = _layers.get_manifold(fsc_indx._manifold_id);
			cout << "manifold id: " << fsc_indx._manifold_id << " fsc id: " << fsc_indx._fsc_id
				<< " angle = " << layer_ptr->constraint().restriction().to_angle();
			if (print_polygon) {
				cout << endl;
				Polygon_with_holes& pgn = layer_ptr->get_fsc(fsc_indx._fsc_id).cell().polygon();
				print_my_polygon_with_holes(pgn, 100);
			}
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
				if (vertex_index == vertex_limit) {
					cout << (polygon.size() - vertex_limit) << " more..." << endl;
					break;
				}
				//format for GUI copy paste
				std::cout << CGAL::to_double(vi->x()) << " " << CGAL::to_double(vi->y()) << std::endl; 
				vertex_index++;
			}
		}

		void print_my_polygon_with_holes(Polygon_with_holes polygon, int vertex_limit) {
			cout << "********************" << endl;
			cout << "Polygon boundary is convex: " << polygon.outer_boundary().is_convex() << endl;
			cout << "Polygon has holes: " << (polygon.number_of_holes() != 0) << endl;
			cout <<"Polygon outer boundary: " << endl;
			print_my_polygon(polygon.outer_boundary(), vertex_limit);
			if (polygon.number_of_holes() != 0) {	
				cout <<"Polygon holes: " << endl;
				for (Polygon_with_holes::Hole_const_iterator hi = polygon.holes_begin(); hi!= polygon.holes_end(); ++hi) {
					print_my_polygon((*hi), vertex_limit);
				}
			}
			cout << "********************" << endl;
		}
	}; 


} //mms
#endif //MY_PLANNER_H
