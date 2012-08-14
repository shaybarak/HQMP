#pragma once

#include "Player.h"

/**
 * Player that sleeps throughout the game.
 */
class SleepingPlayer : protected Player {
public:
	SleepingPlayer(Env* env, Configuration* config) : Player(env, config) {};
	virtual void plan(double deadline);
	virtual bool move(double deadline, Motion& motion_sequence);
};
