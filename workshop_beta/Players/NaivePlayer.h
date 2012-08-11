#pragma once

#include "Player.h"
#include "Path_planning\Motion_sequence.h"
#include "Mms_example.h"
#include "Planner\NaivePlanner.h"

//typedef mms::NaivePlanner<>             NaivePlanner;

/**
 * Player that sleeps throughout the game.
 */
class NaivePlayer : protected Player {
public:
	NaivePlayer(Env* env, Configuration* config);
	virtual void plan(double deadline);
	virtual void move(double deadline, Motion& motion_sequence);

protected:
	Planner planner;
	//Planner planner;

private:
	bool planned;
	bool moved;
};
