#pragma once

#include "stdafx.h"

void timed_trace(const char* function_name, const char* file, int line, const char* action) {
	// Time:
	std::cout << global_tm.timer.time() << ": "
		// File:Line
		<< file << ":" << line << " - "
		// function_name, action
		<< function_name << ", " << action << std::endl;
};
