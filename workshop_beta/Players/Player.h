#pragma once

#include "Path_planning\Motion_sequence.h"
#include "Planner\NaivePlanner.h"

typedef Environment<>                     Env;
typedef mms::NaivePlanner<>				  Planner;
typedef Motion_sequence<Planner::K>		  Motion;
//typedef Env::Reference_point              Ref_p;
//typedef Env::Reference_point_vec          Ref_p_vec;

/**
 * Base class for all tournament player implementations.
 */
class Player {
public:
	static void log(char* label, char* message);
	Player(Env* env, Configuration* config);
	void set_dynamic_obstacle_config(Ref_p& dynamic_obstacle);
	void update_target_configs(Ref_p_vec* target_configs);
	virtual void plan(double deadline) = 0;
	virtual void move(double deadline, Motion& motion_sequence) = 0;
protected:
	Env* env;
	Configuration* config;
	Ref_p dynamic_obstacle;
	Ref_p_vec* target_configs;
};
