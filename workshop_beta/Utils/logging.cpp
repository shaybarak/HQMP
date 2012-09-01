#pragma once

#include "stdafx.h"

void timed_trace(const char* function_name, const char* full_path, int line, const char* action) {
	char filename[_MAX_FNAME];
	_splitpath_s(full_path, 
		NULL, 0, 
		NULL, 0, 
		filename, _MAX_FNAME,
		NULL, 0);
	// Time:
	std::cout << global_tm.timer.time() << ": "
		// File:Line
		<< filename << ":" << line << " - "
		// function_name, action
		<< function_name << ", " << action << std::endl;

};

void timed_message(const char* message) {
	std::cout << global_tm.timer.time() << ": " << message << endl;
}
