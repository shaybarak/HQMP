#include "stdafx.h"
#include "NaivePlayer.h"

NaivePlayer::NaivePlayer(Env* env, Configuration* config) :
	Player(env, config),
	planner(env->get_workspace(),
	env->get_robot_a()),
	q_s(env->get_source_configuration_a()),
	planned(false) {
}

void NaivePlayer::plan(double deadline) {
	if (!planned) {
		// Assume we have enough time to preprocess
		planner.preprocess();
		planned = true;
	}
}

bool NaivePlayer::move(double deadline, Motion& motion_sequence) {

	if (!planned) {
		plan(deadline);
	}

	CGAL::Timer timer;
	timer.start();
	
	Ref_p_vec::iterator iter_to_closest;

	if (!remaining_motion.empty()) {
		// Continue a previous motion
		remaining_motion.cut(deadline - timer.time(), motion_sequence);
		return false;
	}
	
	if (env->get_target_configurations().empty()) {
		// No more motion to plot
		return true;
	}
	
	// Find motion to closest target
	bool path_found = planner.query_closest_point(
		q_s, 
		env->get_target_configurations(),
		iter_to_closest,
		remaining_motion);
	
	if (path_found) {
		q_s = *iter_to_closest;
		// Erase next target
		env->get_target_configurations().erase(iter_to_closest);
		// Cut the motion according to the deadline
		remaining_motion.cut(deadline - timer.time(), motion_sequence);
	}

	return false;
}
