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
	Player(Env* env, Configuration* config);
	void set_dynamic_obstacle_config(Ref_p& dynamic_obstacle);
	virtual void plan(double deadline) = 0;
	// Returns true if there is nowhere left to move (all targets have been collected)
	virtual bool move(double deadline, Motion& motion_sequence) = 0;
protected:
	Env* env;
	Configuration* config;
	Ref_p dynamic_obstacle;
};
