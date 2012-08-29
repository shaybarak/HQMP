#pragma once

#ifdef DEBUG_PRINT_TIME_LOG

#define TIMED_TRACE_ENTER(x) timed_trace(x, __FILE__, __LINE__, "ENTERING")
#define TIMED_TRACE_EXIT(x) timed_trace(x, __FILE__ ,__LINE__, "EXITING")
#define TIMED_TRACE_ACTION(x, y) timed_trace(x, __FILE__, __LINE__, y)

#define PRINT_ROTATIONS() print_rotations()
#define PRINT_CONNECTORS() print_connectors()
#define PRINT_CONNECTIVITY_GRAPH() print_connectivity_graph()
#define PRINT_WORKSPACE() print_workspace()

#define ASSERT_CONDITION(x, message) if (!(x)) { cout << message << endl; __debugbreak(); }

#else

#define TIMED_TRACE_ENTER(x)
#define TIMED_TRACE_EXIT(x)
#define TIMED_TRACE_ACTION(x, y)
#define PRINT_ROTATIONS()
#define PRINT_CONNECTORS()
#define PRINT_CONNECTIVITY_GRAPH()
#define PRINT_WORKSAPCE()
#define ASSERT_CONDITION(x, message)

#endif

#ifdef PLAYBACK_MODE
#define PLAYBACK_PRINT_CONNECTORS() print_connectors_as_configurations()
#define PLAYBACK_PRINT_FIXED_ANGLE_FSCS() print_fixed_angle_fscs(true)

#else
#define PLAYBACK_PRINT_CONNECTORS()
#define PLAYBACK_PRINT_FIXED_ANGLE_FSCS()
#endif

void timed_trace(const char* function_name, const char* full_path, int line, const char* action);
