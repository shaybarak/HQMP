#pragma once

#include "stdafx.h"

//.cannot use __FUNCTION __, sine id contains class name and print is very 

void timed_trace_function(char* function_name, char* action) {
	std::cout << "Time: " << global_tm.timer.time() << " client, thread id " <<boost::this_thread::get_id()<<" " 
		<< "in function " << function_name
		<< " " <<action
		<<std::endl;
};