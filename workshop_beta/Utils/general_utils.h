#pragma once

#ifdef DEBUG_PRINT_TIME_LOG

#define TIMED_TRACE_ENTER(x) timed_trace_function(x, "ENTERING")
#define TIMED_TRACE_EXIT(x) timed_trace_function(x, "EXITING")
#define TIMED_TRACE_ACTION(x, y) timed_trace_function(x, y)
#else
#define TIMED_TRACE_ENTER(x)
#define TIMED_TRACE_EXIT(x)
#define TIMED_TRACE_ACTION(x, y)

#endif

void timed_trace_function(char* function_name, char* action);
