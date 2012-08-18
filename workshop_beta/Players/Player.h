#pragma once

#include "Path_planning\Motion_sequence.h"
#include "Planner\MyPlanner.h"

typedef Environment<>                     Env;
typedef mms::MyPlanner<>				  Planner;
typedef Motion_sequence<Planner::K>		  Motion;

/**
 * Base class for all tournament player implementations.
 */
class Player {
public:
	Player(Env* env, Configuration* config);
	void set_dynamic_obstacle_config(Ref_p& dynamic_obstacle);
	virtual void plan(double deadline) = 0;
	virtual void move(double deadline, Motion& motion_sequence) = 0;
	virtual bool is_game_over() = 0;

protected:
	Env* env;
	Configuration* config;
	Ref_p dynamic_obstacle;
};
