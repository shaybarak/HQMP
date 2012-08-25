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
	// Plan ahead. Returns whether this method should be called again this turn (if time permits).
	virtual bool plan(double deadline) = 0;
	// Generate motion. Returns whether this method should be called again this turn (if time permits).
	virtual bool move(double deadline, Motion& motion_sequence) = 0;
	virtual bool is_game_over() = 0;
	virtual void additional_targets_preprocessing(Ref_p_vec& additional_targets) = 0;

protected:
	Env* env;
	Configuration* config;
	Ref_p dynamic_obstacle;
};
