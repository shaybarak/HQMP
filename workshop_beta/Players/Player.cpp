#include "stdafx.h"
#include "Player.h"
#include "..\Utils\logging.h"

Player::Player(Env* env, Configuration* config) {
	this->env = env;
	this->config = config;
}

void Player::set_dynamic_obstacle_config(Ref_p& dynamic_obstacle) {
	this->dynamic_obstacle = dynamic_obstacle;
}

void Player::reject_last_move(Motion& motion_sequence) {
	// Default behavior is to crash in flames...
	// Derived classes should override this and handle the corner case correctly
	ASSERT_CONDITION(false,
		"OMFG MOVE REJECTED!");
}
