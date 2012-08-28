#pragma once

#include "Player.h"
#include "Path_planning\Motion_sequence.h"
#include "Planner\NaivePlanner.h"

/**
 * Player that sleeps throughout the game.
 */
class MyPlayer : protected Player {
	typedef Planner::Reference_point	Reference_point;
	typedef Planner::Extended_polygon	Extended_polygon;
	typedef Planner::MS_base			MS_base;

public:
	MyPlayer(Env* env, Configuration* config);
	virtual bool plan(double deadline);
	virtual bool move(double deadline, Motion& motion_sequence);
	virtual bool is_game_over();
	virtual void additional_targets_preprocessing(Ref_p_vec& additional_targets);

protected:
	Planner planner;

private:
	Motion pending_motion;
	Reference_point pending_motion_end;
	// Caches whether the plan method should perform another planner query the next time it is called
	bool last_query_succeeded;
	bool planner_initialized;
	bool initialize();
	bool buffer_motion_ahead(const Reference_point& source);
};
