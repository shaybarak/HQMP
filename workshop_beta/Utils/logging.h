#pragma once

#ifdef DEBUG_PRINT_TIME_LOG

	#define TIMED_TRACE_ENTER(x) timed_trace(x, __FILE__, __LINE__, "ENTERING")
	#define TIMED_TRACE_EXIT(x) timed_trace(x, __FILE__ ,__LINE__, "EXITING")
	#define TIMED_TRACE_ACTION(x, y) timed_trace(x, __FILE__, __LINE__, y)
	#define PRINT_ROTATIONS() print_rotations()
	#define PRINT_WORKSPACE() print_workspace()
	#define ASSERT_CONDITION(x, message) if (!(x)) { cout << message << endl; __debugbreak(); }
	#define PRINT_TRY_TO_GENERATE_CONNECTORS(x) cout << "try to generate " << (x) << " connectors" << endl
	#define PRINT_GENERATED_CONNECTORS(x) cout << "generated " << (x) << " connectors" << endl
	#define PRINT_SOURCE_POINT() cout << "SOURCE POINT: "; source.print(); cout << endl
	#define PRINT_TARGET_POINT_AERIAL_TIME() cout << "Query target: "; target.print(); cout << " Aerial time to target: "<< current_aerial_time << endl
	#define PRINT_BEST_MOTION_TIME() cout << "Best motion time till now: " << shortest_time << ", found path to target, motion time: " << current_time << endl
	#define PRINT_SELECTED_TARGET()	cout << "Selected target: "; target_configurations[closest_target_index].print(); cout << " time to target: " << shortest_time << endl;

#else

	#define TIMED_TRACE_ENTER(x)
	#define TIMED_TRACE_EXIT(x)
	#define TIMED_TRACE_ACTION(x, y)
	#define PRINT_ROTATIONS()
	#define PRINT_WORKSAPCE()
	#define ASSERT_CONDITION(x, message)
	#define PRINT_TRY_TO_GENERATE_CONNECTORS(x)
	#define PRINT_GENERATED_CONNECTORS(x)
	#define PRINT_SOURCE_POINT()
	#define PRINT_TARGET_POINT_AERIAL_TIME()
	#define PRINT_BEST_MOTION_TIME()
	#define PRINT_SELECTED_TARGET()

#endif

#ifdef DEBUG_PLANNER
	#define PRINT_CONNECTORS() print_connectors()
	#define PRINT_CONNECTIVITY_GRAPH() print_connectivity_graph()
	#define PRINT_FA_FSC_PREFIX() cout << "FA fsc " << fa_fsc_index << " of " << fa_fsc_count << " "; print_fixed_angle_fsc(*fsc_iter1, false)
	#define PRINT_FA_FSC_SUFFIX() if (ccp_count != 0) {cout << " created " << ccp_count << " points";}cout << endl
	#define	PRINF_FA_FSC_FAILED_GENERATING_POINT()	cout << endl << "Failed generating point, skipping connection"
	#define PRINT_POLYGON_UNBOUNDED_SKIPPING() cout << "polygon is unbounded, skipping connection" << endl

#else
	#define PRINT_CONNECTORS()
	#define PRINT_CONNECTIVITY_GRAPH()
	#define PRINT_FA_FSC_PREFIX()
	#define PRINT_FA_FSC_SUFFIX()
	#define PRINF_FA_FSC_FAILED_GENERATING_POINT()
	#define PRINT_POLYGON_UNBOUNDED_SKIPPING()

#endif

#ifdef PLAYBACK_MODE
	#define PLAYBACK_PRINT_CONNECTORS() print_connectors_as_configurations()
	#define PLAYBACK_PRINT_FIXED_ANGLE_FSCS() print_fixed_angle_fscs(true)

#else
	#define PLAYBACK_PRINT_CONNECTORS()
	#define PLAYBACK_PRINT_FIXED_ANGLE_FSCS()
#endif

void timed_trace(const char* function_name, const char* full_path, int line, const char* action);
void timed_message(const char* message);
