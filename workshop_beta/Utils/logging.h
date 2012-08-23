#pragma once

#ifdef DEBUG_PRINT_TIME_LOG

#define TIMED_TRACE_ENTER(x) timed_trace(x, __FILE__, __LINE__, "ENTERING")
#define TIMED_TRACE_EXIT(x) timed_trace(x, __FILE__ ,__LINE__, "EXITING")
#define TIMED_TRACE_ACTION(x, y) timed_trace(x, __FILE__, __LINE__, y)

//Use only in XxxPlanner.cpp
#define PRINT_ROTATIONS() print_rotations()
#define PRINT_CONNECTORS() print_connectors()

#else
#define TIMED_TRACE_ENTER(x)
#define TIMED_TRACE_EXIT(x)
#define TIMED_TRACE_ACTION(x, y)
#define PRINT_ROTATIONS()
#define PRINT_CONNECTORS()

#endif

void timed_trace(const char* function_name, const char* full_path, int line, const char* action);
