#pragma once

#include "Player.h"
#include "Path_planning\Motion_sequence.h"
#include "Planner\NaivePlanner.h"

/**
 * Player that sleeps throughout the game.
 */
class MyPlayer : protected Player {
public:
	MyPlayer(Env* env, Configuration* config);
	virtual void plan(double deadline);
	virtual void move(double deadline, Motion& motion_sequence);
	virtual bool is_game_over();

protected:
	Planner planner;

private:
	Motion pending_motion;
	Planner::Reference_point pending_motion_end;
	bool initialize();
	bool buffer_motion_ahead();
};
