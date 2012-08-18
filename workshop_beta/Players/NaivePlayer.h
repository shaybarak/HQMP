#pragma once

#include "Player.h"
#include "Path_planning\Motion_sequence.h"
#include "Planner\NaivePlanner.h"

/**
 * Player that sleeps throughout the game.
 */
class NaivePlayer : protected Player {
public:
	NaivePlayer(Env* env, Configuration* config);
	virtual void plan(double deadline);
	virtual void move(double deadline, Motion& motion_sequence);
	virtual bool is_game_over();

protected:
	Planner planner;

private:
	Planner::Reference_point q_s;
	Planner::Reference_point last_point_in_path;
	Motion remaining_motion;
	virtual void plan_future_motion_seq();
	bool planned;
};
