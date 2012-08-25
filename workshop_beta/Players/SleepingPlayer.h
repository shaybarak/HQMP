#pragma once

#include "Player.h"

/**
 * Player that sleeps throughout the game.
 */
class SleepingPlayer : protected Player {
public:
	SleepingPlayer(Env* env, Configuration* config) : Player(env, config) {};
	virtual bool plan(double deadline);
	virtual bool move(double deadline, Motion& motion_sequence);
	virtual bool is_game_over();
	virtual void additional_targets_preprocessing(Ref_p_vec& additional_targets);
};
