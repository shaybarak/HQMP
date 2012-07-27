#include "stdafx.h"
#include "NaivePlayer.h"

NaivePlayer::NaivePlayer(Env* env, Configuration* config) : Player(env, config), planner(env->get_workspace(), env->get_robot_a()) {
	planned = false;
	moved = false;
};

void NaivePlayer::plan(double deadline) {
	// Don't allow more than one planning turn
	if (!planned) {
		// Assume we have enough time to preprocess
		planner.preprocess();
		planned = true;
	}
}

void NaivePlayer::move(double deadline, Motion& motion_sequence) {
	// Don't allow more than one movement turn
	if (!moved) {
		// Assume we have enough time to perform the entire motion
		// Get source the destination
		Planner::Reference_point q_s (env->get_source_configuration_a());
		Planner::Reference_point q_t (env->get_target_configurations().front());

		//perform query
		bool found_path = planner.query(q_s, q_t, motion_sequence);
		moved = true;
	}
}
