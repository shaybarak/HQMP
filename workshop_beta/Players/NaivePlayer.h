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
	virtual bool move(double deadline, Motion& motion_sequence);

protected:
	Planner planner;

private:
	Planner::Reference_point q_s;
	Planner::Reference_point q_t;
	Motion remaining_motion;
	bool planned;
};
