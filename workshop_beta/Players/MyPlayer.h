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
	typedef Env::Reference_point_vec	Reference_point_vec;

public:
	MyPlayer(Env* env, Configuration* config);
	virtual bool plan(double deadline);
	virtual bool move(double deadline, Motion& motion_sequence);
	virtual bool is_game_over();
	virtual void additional_targets_preprocessing(Ref_p_vec& additional_targets);

protected:
	Planner planner;

private:
	// Buffer of motion planned ahead (to execute later)
	std::deque<Motion> motion_buffer;
	// End of buffered motion
	Reference_point buffer_end;
	// Targets in buffer
	std::deque<Reference_point> buffered_targets;
	// Caches whether the last query attempt succeeded
	bool last_query_succeeded;

	bool planner_initialized;
	bool initialize();
	bool move_to_closest_target(Reference_point& source, Reference_point_vec& targets, Motion& motion);
};
