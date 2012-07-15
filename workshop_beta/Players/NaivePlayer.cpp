#include "stdafx.h"
#include "NaivePlayer.h"

NaivePlayer::NaivePlayer(Env* env, Configuration* config) : Player(env, config), planner(env->get_workspace(), env->get_robot_a()) {
};

void NaivePlayer::plan(double deadline) {
	// Assume we have enough time to preprocess
	planner.preprocess();
}

void NaivePlayer::move(double deadline, Motion& motion_sequence) {
	// Assume we have enough time to perform the entire motion
	// Get source the destination
	Planner::Reference_point q_s (env->get_source_configuration_a());
	Planner::Reference_point q_t (env->get_target_configurations().front());

	//perform query
	bool found_path = planner.query(q_s, q_t, motion_sequence);
}
