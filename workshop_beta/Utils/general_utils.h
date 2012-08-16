#pragma once

#ifdef DEBUG_PRINT_TIME_LOG

#define TIMED_TRACE_ENTER(x) timed_trace_function(x, "ENTERING")
#define TIMED_TRACE_EXIT(x) timed_trace_function(x, "EXITING")
#else
#define TIMED_TRACE_ENTER(x)
#define TIMED_TRACE_EXIT(x)

#endif

void timed_trace_function(char* function_name, char* action);
